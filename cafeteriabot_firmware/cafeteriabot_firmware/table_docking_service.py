#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

# custom interface
from cafeteriabot_interface.action import DockToTable
from cafeteriabot_interface.msg import DockToTableFeedback
from cafeteriabot_interface.msg import TableGeometry
from cafeteriabot_interface.srv import TableAvailable


class TableDockingServiceNode(Node):
    def __init__(self):
        super().__init__("table_docking_service_node")
        # member variables
        self.center = None
        self.midpoints = None
        self.px = None
        self.py = None
        self.yaw = None
        self.stage = None

        self.goal_handle: ServerGoalHandle = None
        self.abort_handle = False

        # node parameters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_base_footprint").value
        self.retry_duration = self.declare_parameter("retry_duration", 5).value
        self.distance_away = self.declare_parameter("distance_away", 0.5).value
        self.step_size = self.declare_parameter("step_size", 0.5).value
        self.node_rate = self.declare_parameter("node_rate", 10.0).value
        self.goal_tolerance = self.declare_parameter("goal_tolerance", 0.1).value
        self.yaw_tolerance = self.declare_parameter("yaw_tolerance", 0.1).value

        # node modification
        self.loop_rate = self.create_rate(self.node_rate)

        # PID controller for linear and angular controls
        self.linear_pid = PIDController(
            Kp=0.25,
            Ki=0.005,
            Kd=0.0,
            output_min=-1.50,
            output_max=1.50,
            integrator_min=-0.25,
            integrator_max=0.25,
            dt=0.1,
        )
        self.angular_pid = PIDController(
            Kp=0.75,
            Ki=0.005,
            Kd=0.0,
            output_min=-1.0,
            output_max=1.0,
            integrator_min=-0.25,
            integrator_max=0.25,
            dt=0.1,
        )

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback groups
        self.callback_g1 = MutuallyExclusiveCallbackGroup()
        self.callback_g2 = ReentrantCallbackGroup()

        # ros objects
        self.server_dock = ActionServer(
            self,
            DockToTable,
            "/dock_to_table",
            goal_callback=self.callback_goal,
            execute_callback=self.callback_execute,
            cancel_callback=self.callback_cancel,
            callback_group=self.callback_g2,  # multi goal policy by default
        )
        self.client_table = self.create_client(
            TableAvailable, "/is_available",
        )
        self.client_path = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose",
        )
        self.subscriber_table = self.create_subscription(
            TableGeometry, "/table", self.subscriber_table_callback, 10,
        )
        self.publisher_path = self.create_publisher(
            Path, "/table_path", 10
        )
        self.publisher_feedback = self.create_publisher(
            DockToTableFeedback, "/dock_to_table_feedback", 10
        )
        self.publisher_velocity = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        self.publisher_pose = self.create_publisher(
            Pose2D, '/current_pose', 10
        )
        self.timer_transformation = self.create_timer(
            0.2, self.timer_transformation_callback, callback_group=self.callback_g1,
        )

        # parameter handler
        self.add_on_set_parameters_callback(self.on_parameter_changed)

        # node acknowledgement
        self.get_logger().info("Server started successfully at /dock_to_table.")

    def on_parameter_changed(self, params):
        for param in params:
            if hasattr(self, param.name):
                current_value = getattr(self, param.name)
                if current_value != param.value:
                    self.get_logger().info(f"Parameter '{param.name}' changed from {current_value} to {param.value}")
                    setattr(self, param.name, param.value)
            else:
                self.get_logger().warn(f"Attempting to set an unknown parameter '{param.name}'")
        return SetParametersResult(successful=True)

    def timer_transformation_callback(self):
        try:
            # wait for the transformation to be available
            transform = self.tf_buffer.lookup_transform(self.f_ref, self.t_ref, rclpy.time.Time())

            # extract position
            translation = transform.transform.translation
            self.px = translation.x
            self.py = translation.y

            # extract orientation
            orientation = transform.transform.rotation
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w
            self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

            # define message
            message = Pose2D()

            # define pose value
            message.x = self.px
            message.y = self.py
            message.theta = self.yaw

            # publish pose to topic
            self.publisher_pose.publish(message)

        except Exception as e:
            self.px, self.py, self.yaw = None, None, None
            self.get_logger().warn(
                f"Could not transform from '{self.f_ref}' to '{self.t_ref}'", skip_first=True, throttle_duration_sec=5
            )

    def subscriber_table_callback(self, message):
        # extract table center
        self.center = (message.center.x, message.center.y)

        # extract table midpoints
        self.midpoints = [(p.x, p.y) for p in message.midpoints]

    def check_table_availability(self):
        # define service request
        request = TableAvailable.Request()
        request.duration = self.retry_duration

        try:
            # check if service available
            if not self.client_table.wait_for_service(timeout_sec=2.0):
                raise ValueError("Unable to find service.")

            # send service call, wait for the result
            future_response = self.client_table.call_async(request)
            self.spin_until_future_complete(future_response)

            # check if the table was successfully found
            result = future_response.result()
            if result.success:
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Service call did not complete successfully: {e}")
            return False

    def validate_external_point(self, pose):
        # define action goal request
        request = ComputePathToPose.Goal()
        request.goal = pose
        request.use_start = False  # assuming the current robot pose as start

        try:
            # check if server available
            if not self.client_path.wait_for_server(timeout_sec=2.0):
                raise ValueError("Unable to find server.")

            # send action call, wait for the server to accept the goal
            future_request = self.client_path.send_goal_async(request)
            self.spin_until_future_complete(future_request)

            if not future_request.done():
                raise ValueError("Goal submission not completed within the expected time.")

            goal_handle = future_request.result()
            if not goal_handle.accepted:
                raise ValueError("Goal sent, but rejected by server.")

            # goal was accepted, wait for the result
            future_response = goal_handle.get_result_async()
            self.spin_until_future_complete(future_response)

            # check if the path was successfully found
            result = future_response.result().result
            if len(result.path.poses) > 0:
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Action call did not complete successfully: {e}")
            return False

    def callback_goal(self, goal_request: DockToTable.Goal):
        self.get_logger().info(f"Service /dock_to_table received a new goal request: {goal_request.dock}")

        # Goal Policy: refuse new goal if current goal still active
        if self.goal_handle is not None:
            if self.goal_handle.is_active:
                self.get_logger().warn("Goal rejected: the current goal is still active.")
                return GoalResponse.REJECT

        # accept the goal request
        self.get_logger().info("Goal accepted: processing new goal request.")
        return GoalResponse.ACCEPT

    def callback_execute(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Verifying feasibility, before goal execution process.")

        # register active goal and verify
        self.goal_handle = goal_handle
        self.stage = 0

        try:
            # check table availability and raise an exception if a table is not available
            if not self.check_table_availability():
                raise ValueError("No table found in the given time.")

            self.stage += 1
            self.publish_feedback(self.stage, 1, False, "active")

            # retrieve positions at execution time (t)
            cx, cy = self.center
            px, py = self.px, self.py

        except Exception as e:
            # send aborted state, based on goal state machine
            return self.process_next_goal_and_result("aborted", f"{e}")

        for index, (mx, my) in enumerate(self.midpoints):
            # calculate external point
            ex, ey, yaw = self.calculate_external_point(px, py, cx, cy, mx, my)

            # validate external point from (planner_server)
            dock_pose = self.create_pose(ex, ey, yaw)
            if self.validate_external_point(dock_pose):
                valid_index = index  # index of the valid point
                self.get_logger().info(
                    f"External Point {valid_index + 1} Pose ({ex:.2f}, {ey:.2f}, Yaw {yaw:.2f}) accepted by planner."
                )

                self.stage += 1
                self.publish_feedback(self.stage, 1, False, "active")
                break
            else:
                self.get_logger().info(
                    f"External Point {index} Pose ({ex:.2f}, {ey:.2f}, Yaw {yaw:.2f}) rejected by planner."
                )
        else:
            # send aborted state, based on goal state machine
            message = "No valid external point found after checking all midpoints."
            return self.process_next_goal_and_result("aborted", message)

        # complete path to table center from current position
        path_points = [
            (ex, ey),  # external point
            (mx, my),  # table mid point
            (cx, cy),  # table center
        ]
        self.publish_complete_path(path_points)

        # check goal request
        if not self.goal_handle.request.dock:
            # send succeed state, based on goal state machine
            message = "As requested docking not performed post feasibility check."
            return self.process_next_goal_and_result("succeed", message)

        self.get_logger().info("Table available, beginning goal execution process.")

        # iterate over each target point in path_points
        for index, (tx, ty) in enumerate(path_points):
            self.get_logger().info(f"Starting navigation to target point {index+1}: ({tx:.2f}, {ty:.2f})")

            # reset controller
            self.linear_pid.reset_controller()
            self.angular_pid.reset_controller()

            # start the execution loop
            distance_aligned = False
            while rclpy.ok():
                # check if goal has been canceled
                if self.goal_handle.is_cancel_requested:
                    # send cancel state, based on goal state machine
                    message = "The goal was canceled upon user or system request"
                    return self.process_next_goal_and_result("canceled", message)

                distance_delta, distance_variable, angle_delta, angle_variable = self.calculate_delta(tx, ty)
                orientation_delta = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))

                # check if distance is aligned
                if not distance_aligned:
                    if not self.is_goal_aligned(distance_delta):
                        # move to align distance
                        linear, angular = self.calculate_velocity(
                            distance_delta, angle_delta, distance_variable, angle_variable
                        )
                        self.publish_velocity(linear, angular)
                    else:
                        # set flag to true when distance is aligned
                        distance_aligned = True
                else:
                    # if distance is aligned, align angle
                    if not self.is_angle_aligned(orientation_delta):
                        # rotate to align angle
                        linear, angular = self.calculate_velocity(
                            distance_delta, orientation_delta, distance_variable, angle_variable
                        )
                        self.publish_velocity(0.0, angular)
                    else:
                        self.stage += 1
                        self.publish_feedback(self.stage, 1, False, "active")
                        self.get_logger().info("Navigation to point completed.")
                        break

                # spin all threads
                self.loop_rate.sleep()

        # send succeed state, based on goal state machine
        message = "Robot is under the table."
        return self.process_next_goal_and_result("succeed", message)

    def callback_cancel(self, goal_handle: ServerGoalHandle):  # thread 2
        self.get_logger().info("Received goal cancel request.")
        return CancelResponse.ACCEPT  # or REJECT

    def process_next_goal_and_result(self, state, message):
        # action server response
        result = DockToTable.Result()
        result.message = message
        result.success = True

        if state == "succeed":
            self.publish_feedback(self.stage, 4, True, "succeed")
            self.goal_handle.succeed()
        elif state == "canceled":
            self.publish_feedback(self.stage, 2, True, "canceled")
            self.goal_handle.canceled()
        elif state == "aborted":
            self.publish_feedback(self.stage, 3, True, "aborted")
            self.goal_handle.abort()
        else:
            self.get_logger().error(f"Unknown state '{state}' provided. No action taken.")
            return None

        # reset the goal handle and clear path
        self.goal_handle = None
        self.publish_empty_path()

        # log status and return
        self.get_logger().info(f"Goal '{state}': {message}")
        return result

    def publish_feedback(self, stage, status, success, message):
        # goal feedback
        feedback = DockToTable.Feedback()

        feedback.stage = stage
        feedback.status = status
        feedback.success = success
        feedback.message = message

        # publish action feedback
        self.goal_handle.publish_feedback(feedback)

        # define message
        feedback = DockToTableFeedback()

        feedback.stage = stage
        feedback.status = status
        feedback.success = success
        feedback.message = message

        # publish message to topic
        self.publisher_feedback.publish(feedback)

    def is_goal_aligned(self, distance):
        # check if current delta distance less then tolerance
        if distance < self.goal_tolerance:
            # halt robot motion, return true
            self.get_logger().info(f"Distance to target aligned with Δ {distance:.2f} m.")
            self.publish_velocity(0.0, 0.0)
            return True

        # return false by default
        return False

    def is_angle_aligned(self, angle):
        # check if current delta angle less then tolerance
        if angle < self.yaw_tolerance:
            # halt robot motion, return true
            self.get_logger().info(f"Orientation to target aligned with Δ {angle:.2f} rad.")
            self.publish_velocity(0.0, 0.0)
            return True

        # return false by default
        return False

    def calculate_delta(self, tx, ty):
        # calculate delta (error)
        dx = tx - self.px
        dy = ty - self.py

        # distance and angle measurement
        distance_variable = math.sqrt(self.px**2 + self.py**2)
        angle_variable = self.yaw

        # calculate distance and angle
        distance_delta = math.sqrt(dx**2 + dy**2)
        angle_delta = math.atan2(dy, dx) - self.yaw
        angle_prime = math.atan2(math.sin(angle_delta), math.cos(angle_delta))

        # log information and return
        self.get_logger().info(
            f"Position Update - Δ(x, y): ({dx:.2f}, {dy:.2f}), Distance: {distance_delta:.2f} m, Angle: {angle_delta:.2f} rad.",
            throttle_duration_sec=5.0,
        )
        return distance_delta, distance_variable, angle_prime, angle_variable

    def calculate_velocity(self, distance_delta, angle_delta, distance_variable, angle_variable):
        # PID : calculate linear and angular velocities
        linear, _, _, _ = self.linear_pid.get_total_gain(distance_delta, distance_variable)
        angular, _, _, _ = self.angular_pid.get_total_gain(angle_delta, angle_variable)

        # return velocity
        return linear, angular

    def publish_velocity(self, linear, angular):
        # define message
        message = Twist()

        # define linear and angular velocity
        message.linear.x = linear
        message.angular.z = angular

        # publish velocity to topic
        self.get_logger().info(
            f" Set Velocity - Linear: {linear:5.2f} m/s, Angular {angular:5.2f} rad/s.", throttle_duration_sec=1.0
        )
        self.publisher_velocity.publish(message)

    def calculate_external_point(self, px, py, cx, cy, mx, my):
        # calculate direction vector and its magnitude
        vector = (mx - cx, my - cy)
        magnitude = math.sqrt(vector[0] ** 2 + vector[1] ** 2)

        # calculate unit vector
        unit_vector = (vector[0] / magnitude, vector[1] / magnitude)

        # calculate new point magnitude from centroid
        distance = self.distance_away + magnitude

        # generate potential new points along the unit vector, starting from the centroid
        point1 = (cx + unit_vector[0] * distance, cy + unit_vector[1] * distance)
        point2 = (cx - unit_vector[0] * distance, cy - unit_vector[1] * distance)

        # determine which new point is closer to current position
        distance1 = math.sqrt((point1[0] - px) ** 2 + (point1[1] - py) ** 2)
        distance2 = math.sqrt((point2[0] - px) ** 2 + (point2[1] - py) ** 2)

        # select the point and find its orientation
        position = point1 if distance1 < distance2 else point2
        orientation = math.atan2(cy - position[1], cx - position[0])

        return position[0], position[1], orientation

    def publish_complete_path(self, points):
        # check if there are at least two points to form a path
        if len(points) < 2:
            return

        # path message header
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # iterate through each pair of points
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]

            # calculate distance and steps between the current pair of points
            dx, dy = end_point[0] - start_point[0], end_point[1] - start_point[1]
            distance = math.sqrt(dx**2 + dy**2)
            steps = int(distance / 0.2)

            # generate intermediate points between the current pair of points
            for step in range(steps + 1):
                pose = PoseStamped()
                pose.header = path.header
                ratio = step / float(steps)
                pose.pose.position.x = start_point[0] + ratio * dx
                pose.pose.position.y = start_point[1] + ratio * dy
                path.poses.append(pose)

        # publish and return path
        self.publisher_path.publish(path)

    def publish_empty_path(self):
        # path message header
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # publish and return path
        self.publisher_path.publish(path)

    def create_pose(self, x, y, yaw):
        # create ros2 position-orientation-stamped-message (PoseStamped)
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()

        # position
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 0.0

        # orientation
        r = self.quaternion_from_euler(0, 0, yaw)
        message.pose.orientation.x = r[0]
        message.pose.orientation.y = r[1]
        message.pose.orientation.z = r[2]
        message.pose.orientation.w = r[3]
        return message

    def quaternion_from_euler(self, roll, pitch, yaw):
        # calculate vector components
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # calculate quaternion (x, y, z, w)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr  # x
        q[1] = sy * cp * sr + cy * sp * cr  # y
        q[2] = sy * cp * cr - cy * sp * sr  # z
        q[3] = cy * cp * cr + sy * sp * sr  # w
        return q

    def spin_until_future_complete(self, future, expire=True):
        # wait for a future to complete
        start_time = time.time()
        while rclpy.ok() and not future.done():
            # if the retry duration expire
            if expire and (time.time() - start_time) >= self.retry_duration:
                self.get_logger().info("Retry duration exceeded, closing the wait loop.")
                return False

            # sleep to prevent busy waiting
            self.loop_rate.sleep()
        else:
            return True

    def publish_transformation(self, frame, x, y):
        # transform header definition
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.f_ref
        transform.child_frame_id = frame

        # position and orientation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.w = 1.0

        # broadcast transform
        self.tf_broadcaster.sendTransform(transform)


class PIDController:
    def __init__(self, Kp, Ki, Kd, output_min, output_max, integrator_min, integrator_max, dt):
        # controller gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # output limits
        self.output_min = output_min
        self.output_max = output_max

        # integrator state and limits
        self.integrator = 0
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max
        self.previous_error = 0

        # differentiator state
        self.differentiator = 0
        self.previous_measurement = 0

        # sample time
        self.dt = dt

    def reset_controller(self):
        # reset controller history
        self.integrator = 0
        self.differentiator = 0
        self.previous_measurement = 0

    def get_proportional_gain(self, error):
        # calculate proportional gain
        return self.Kp * error

    def get_integral_gain(self, error):
        # calculate integral gain
        value = self.integrator + (0.5 * self.Ki * (error + self.previous_error) * self.dt)

        # anti-wind-up via integrator clamping
        if value > self.integrator_max:
            self.integrator = self.integrator_max
        elif value < self.integrator_min:
            self.integrator = self.integrator_min
        else:
            self.integrator = value
        self.previous_error = error

        return self.integrator

    def get_differential_gain(self, measurement):
        # calculate differential gain (derivative on measurement)
        value = (measurement - self.previous_measurement) / self.dt
        self.previous_measurement = measurement

        return self.Kd * value

    def get_total_gain(self, error, measurement):
        # get proportional, integral, and differential gains
        p = self.get_proportional_gain(error)
        i = self.get_integral_gain(error)
        d = self.get_differential_gain(measurement)

        # calculate total gain
        output = p + i + d

        # output with limits
        if output > self.output_max:
            return self.output_max
        elif output < self.output_min:
            return self.output_min
        else:
            return output, p, i, d


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = TableDockingServiceNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
