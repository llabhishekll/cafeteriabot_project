#!/usr/bin/env python3
import math
import time
import yaml
import threading
import pathlib

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

# custom interface
from cafeteriabot_interface.action import DockToTable
from cafeteriabot_interface.srv import TableAvailable
from cafeteriabot_interface.srv import RobotCommand


class CafeteriaRobotControlNode(Node):
    def __init__(self, navigator):
        super().__init__("cafeteria_robot_control_node")
        self.navigator: BasicNavigator = navigator

        # member variables
        self.current_index = 0
        self.waypoints = []
        self.px = None
        self.py = None
        self.yaw = None

        # state machine
        self.state_thread = None
        self.previous_state = None
        self.current_state = None
        self.next_state = "state_ideal"
        self.transition_requested = True

        # thread variables
        self.thread_lock = threading.Lock()
        self.cancel_event = threading.Event()

        # node parameters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_base_footprint").value
        self.yaml_filename = self.declare_parameter("yaml_filename", "/").value
        self.retry_duration = self.declare_parameter("retry_duration", 5).value
        self.node_rate = self.declare_parameter("node_rate", 10.0).value
        self.marker_duration = self.declare_parameter("marker_duration", 0).value
        self.cancel_duration = self.declare_parameter("marker_duration", 5).value

        # node modification
        self.waypoints_data = self.load_waypoints_yaml(self.yaml_filename)
        self.loop_rate = self.create_rate(self.node_rate)

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # callback groups
        self.callback_g1 = MutuallyExclusiveCallbackGroup()
        self.callback_g2 = ReentrantCallbackGroup()

        # ros objects
        self.server_patrol = self.create_service(
            RobotCommand, "/robot_command", self.service_command_callback
        )
        self.client_table = self.create_client(
            TableAvailable, "/is_available"
        )
        self.client_elevator = self.create_client(
            SetBool, "/elevator_control"
        )
        self.client_dock = ActionClient(
            self, DockToTable, "/dock_to_table"
        )
        self.publisher_array = self.create_publisher(
            MarkerArray, "/waypoints_marker_array", 10
        )
        self.timer_transformation = self.create_timer(
            0.2, self.timer_transformation_callback, callback_group=self.callback_g1
        )
        self.timer_transition = self.create_timer(
            1.0, self.manage_state_transition, callback_group=self.callback_g2
        )

        # parameter handler
        self.add_on_set_parameters_callback(self.on_parameter_changed)

        # node acknowledgement
        self.get_logger().info("Cafeteriabot control node started successfully.")

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

        except Exception as e:
            self.px, self.py, self.yaw = None, None, None
            self.get_logger().warn(
                f"Could not transform from '{self.f_ref}' to '{self.t_ref}'",
                skip_first=True,
                throttle_duration_sec=5,
            )

    def service_command_callback(self, request, response):
        # process the incoming request
        command = request.command.lower()

        if command == "patrol":
            # update waypoint and start index
            self.waypoints = self.get_patrol_points("patrol")
            self.current_index = self.find_closest_waypoint()

            # set the next state, and indicate a transition
            self.request_transition("state_patrol")

            # service response
            success = True
            message = f"Patrolling started: {len(self.waypoints)} waypoints initialized."

        elif command == "home_to_drop":
            # update waypoint and start index
            self.waypoints = self.get_patrol_points("home_to_drop")
            self.current_index = self.find_closest_waypoint()

            # set the next state, and indicate a transition
            self.request_transition("state_move")

            # service response
            success = True
            message = f"Moving to `Drop` {len(self.waypoints)} waypoints initialized."

        elif command == "drop_to_home":
            # update waypoint and start index
            self.waypoints = self.get_patrol_points("drop_to_home")
            self.current_index = self.find_closest_waypoint()

            # set the next state, and indicate a transition
            self.request_transition("state_move")

            # service response
            success = True
            message = f"Moving to `Home` {len(self.waypoints)} waypoints initialized."

        elif command == "dock":
            # set the next state, and indicate a transition
            self.request_transition("state_dock")

            # service response
            success = True
            message = "Action goal sent."

        elif command == "cancel":
            # set the next state, and indicate a transition
            self.cancel_event.set()

            # service response
            success = True
            message = "Action goal canceled."

        elif command == "stop":
            # set the next state, and indicate a transition
            self.cancel_event.set()

            # service response
            success = True
            message = "Robot stopped: All waypoints cleared."

        else:
            # command is not recognized; set the response accordingly
            success = False
            message = f"Invalid command '{command}' received. No action taken."

        # return service response
        response.success = success
        response.message = message
        return response

    def manage_state_transition(self):
        # if no transition has been requested, return
        if not self.transition_requested:
            return

        # if there is an ongoing thread, join it first to clean up
        if self.state_thread and self.state_thread.is_alive():
            self.state_thread.join()

        # update the previous state and transition to the new state
        self.previous_state = self.current_state
        self.current_state = self.next_state
        self.transition_requested = False

        # start the new state in a new thread
        self.state_thread = threading.Thread(target=self.run_current_state)
        self.state_thread.start()

    def run_current_state(self):
        try:
            # call the current state's method
            state_method = getattr(self, self.current_state)
            state_method()
        except Exception as e:
            self.get_logger().error(f"Exception in {self.current_state}: {str(e)}")
            self.request_transition("state_halt")

    def request_transition(self, state):
        self.next_state = state
        self.transition_requested = True
        return True

    def state_ideal(self):
        self.get_logger().info("State 'IDEAL': Ready for transition.")

        # clear any earlier event
        self.cancel_event.clear()
        self.control_elevator(False)

        # set the next state, and indicate no transition
        self.next_state = None
        self.previous_state = None
        self.current_state = None
        self.transition_requested = False

    def state_patrol(self):
        self.get_logger().info("State 'PATROL': Beginning patrol operations.")

        # start checking for table periodically
        self.future_table = self.check_table_availability()

        while rclpy.ok():
            # fetch the next waypoint, and send to navigation stack
            next_waypoint = self.get_next_waypoint()
            self.navigator.goToPose(next_waypoint)

            # update the submit_time timestamp to the current time
            self.submit_time = self.get_clock().now()

            # sleep for few sec before looping
            time.sleep(2)

            while not self.navigator.isTaskComplete():
                if self.evaluate_navigation_feedback():
                    break

                # check if table has been found
                if self.process_table_availability():
                    return

                # cancellation event has been triggered
                if self.check_cancel_event():
                    return

                # sleep for few sec before rechecking
                time.sleep(1)

    def state_dock(self):
        self.get_logger().info("State 'DOCK': Docking robot to table.")

        # move robot under table
        if not self.move_robot_under(True):
            return self.request_transition("state_halt")

        if not self.control_elevator(True):
            return self.request_transition("state_halt")

        # move robot back
        self.navigator.backup(backup_dist=0.45, backup_speed=0.10, time_allowance=10)

        while not self.navigator.isTaskComplete():
            # cancellation event has been triggered
            if self.check_cancel_event():
                return

            # sleep for few sec before rechecking
            time.sleep(1)

        # update waypoints for path to follow
        self.waypoints = self.get_patrol_points("home_to_drop")
        self.current_index = self.find_closest_waypoint()

        # set the next state, and indicate a transition
        return self.request_transition("state_move")

    def state_move(self):
        self.get_logger().info("State 'MOVE': Moving to destination.")

        for _ in range(self.current_index, len(self.waypoints)):
            # fetch the next waypoint, and send to navigation stack
            next_waypoint = self.get_next_waypoint()
            self.navigator.goToPose(next_waypoint)

            # update the submit_time timestamp to the current time
            self.submit_time = self.get_clock().now()

            # sleep for few sec before looping
            time.sleep(2)

            while not self.navigator.isTaskComplete():
                if self.evaluate_navigation_feedback():
                    break

                # cancellation event has been triggered
                if self.check_cancel_event():
                    return

                # sleep for few sec before rechecking
                time.sleep(1)

        # set the next state, and indicate a transition
        if self.previous_state == "state_dock":
            return self.request_transition("state_undock")
        else:
            return self.request_transition("state_halt")

    def state_undock(self):
        self.get_logger().info("State 'UNDOCK': Undocking robot from table.")

        # undock robot to table, update footprint
        if not self.control_elevator(False):
            return self.request_transition("state_halt")

        # move robot back
        self.navigator.backup(backup_dist=0.45, backup_speed=0.10, time_allowance=10)

        while not self.navigator.isTaskComplete():
            # cancellation event has been triggered
            if self.check_cancel_event():
                return

            # sleep for few sec before rechecking
            time.sleep(1)

        # update waypoints for path to follow
        self.waypoints = self.get_patrol_points("drop_to_home")
        self.current_index = self.find_closest_waypoint()

        # set the next state, and indicate a transition
        return self.request_transition("state_move")

    def state_halt(self):
        self.get_logger().info("State 'HALT': Robot halted as system requested.")

        # delete all previous marker
        self.publish_delete_all_marker()

        # update waypoint and start index
        self.waypoints = []
        self.current_index = 0

        # set the next state, and indicate a transition
        return self.request_transition("state_ideal")

    def check_table_availability(self):
        try:
            # define service request
            request = TableAvailable.Request()
            request.duration = self.retry_duration

            # check if service available
            if not self.client_table.wait_for_service(timeout_sec=2.0):
                raise ValueError("Unable to find service.")

            # send service call, return future
            future_result = self.client_table.call_async(request)
            return future_result
        except Exception as e:
            self.get_logger().error(f"Service call did not complete successfully: {str(e)}")
            return None

    def process_table_availability(self):
        if self.future_table is not None and self.future_table.done():
            # if the table is successfully found
            result = self.future_table.result()
            if result.success:
                self.get_logger().info("Table located, transitioning to 'DOCK' state.")
                self.navigator.cancelTask()

                # set the next state, and indicate a transition
                return self.request_transition("state_dock")
            else:
                # send request again if table not found
                self.get_logger().debug("Table not found, sending request again.")
                self.future_table = self.check_table_availability()
        return False

    def evaluate_navigation_feedback(self):
        feedback = self.navigator.getFeedback()
        # if robot is unable to recover and complete task
        if feedback.number_of_recoveries > 5:
            self.get_logger().warning(f"Patrol cancelled after {feedback.number_of_recoveries} recovery attempts.")
            self.navigator.cancelTask()
            return True

        # check if distance is within goal target
        distance = feedback.distance_remaining
        seconds = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9

        # calculate the elapsed time since the last update
        current_time = self.get_clock().now()
        elapsed_submit_time = current_time - self.submit_time

        if distance < 0.125:
            if elapsed_submit_time.nanoseconds * 1e-9 > self.cancel_duration:
                self.get_logger().info(f"Goal cancelled as distance within reach in {seconds:.2f} sec.")
                self.navigator.cancelTask()
                return True
        return False

    def check_cancel_event(self):
        if self.cancel_event.is_set():
            self.get_logger().info("Task cancellation requested, transitioning to 'HALT' state.")
            self.navigator.cancelTask()

            # set the next state, and indicate a transition
            return self.request_transition("state_halt")
        return False

    def move_robot_under(self, dock):
        # define action goal request
        request = DockToTable.Goal()
        request.dock = dock

        try:
            # check if server available
            if not self.client_dock.wait_for_server(timeout_sec=2.0):
                raise ValueError("Unable to find server.")

            # send action call, wait for the server to accept the goal
            future_request = self.client_dock.send_goal_async(
                request,
                feedback_callback=lambda feedback: (
                    self.get_logger().info(
                        f"Stage: {feedback.feedback.stage}, "
                        f"Status: {feedback.feedback.status}, "
                        f"Message: '{feedback.feedback.message}'"
                    ),
                ),
            )
            self.spin_until_future_complete(future_request)

            if not future_request.done():
                raise ValueError("Goal submission not completed within the expected time.")

            goal_handle = future_request.result()
            if not goal_handle.accepted:
                raise ValueError("Goal sent, but rejected by server.")

            # goal was accepted, wait for the result
            future_response = goal_handle.get_result_async()
            if not self.spin_until_future_complete(future_response, expire=False):
                _ = goal_handle.cancel_goal_async()
                raise ValueError("Goal canceled as requested.")

            # check if the dock was successfully
            result = future_response.result().result
            if result.success:
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Action call did not complete successfully: {e}")
            return False

    def control_elevator(self, value):
        # define service request
        request = SetBool.Request()
        request.data = value

        try:
            # check if service available
            if not self.client_elevator.wait_for_service(timeout_sec=2.0):
                raise ValueError("Unable to find service.")

            # send service call, wait for the result
            future_response = self.client_elevator.call_async(request=request)
            self.spin_until_future_complete(future_response)

            # process the result after the call completes
            result = future_response.result()
            if result.success:
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Service call did not complete successfully: {e}")
            return False

    def spin_until_future_complete(self, future, expire=True):
        # wait for a future to complete
        start_time = time.time()
        while rclpy.ok() and not future.done():
            # if the retry duration expire
            if expire and (time.time() - start_time) >= self.retry_duration:
                self.get_logger().info("Retry duration exceeded, closing the wait loop.")
                return False

            # check if cancellation requested
            if self.cancel_event.is_set():
                return False

            # sleep to prevent busy waiting
            self.loop_rate.sleep()
        else:
            return True

    def load_waypoints_yaml(self, file_path):
        if pathlib.Path(file_path).is_file():
            # load waypoints from yaml file
            with open(file_path, "r") as file:
                waypoints = yaml.safe_load(file)

            self.get_logger().info("YAML file successfully loaded.")
            return waypoints
        else:
            self.get_logger().error(f"Error: The file '{file_path}' does not exist.")
            return []

    def get_patrol_points(self, mode):
        # determine environment key
        environment_key = "simulation" if self.use_sim_time else "real"

        # delete all previous marker
        self.publish_delete_all_marker()

        # get correct correct set of waypoints
        waypoints = self.waypoints_data[environment_key][mode]
        self.get_logger().info(f"Updated waypoints: {waypoints}")

        # publish new waypoints marker
        points = [Point(x=x, y=y, z=1.0) for x, y, _ in waypoints]
        self.publish_text_marker("robot_waypoints", points, (0.0, 0.0, 1.0))

        return waypoints

    def find_closest_waypoint(self):
        closest_index = -1
        min_distance = float("inf")

        # find the closest point index based on euclidean distance
        for index, (wx, wy, _) in enumerate(self.waypoints):
            distance = math.sqrt((wx - self.px) ** 2 + (wy - self.py) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_index = index

        return closest_index

    def get_next_waypoint(self):
        # fetch the next waypoint pose
        next_waypoint = self.waypoints[self.current_index]
        self.current_index = (self.current_index + 1) % len(self.waypoints)

        # return a pose-stamped message
        return self.create_pose(*next_waypoint)

    def create_pose(self, x, y, yaw):
        # create ros2 position-orientation-stamped-message (PoseStamped)
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.navigator.get_clock().now().to_msg()

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

    def create_basic_marker(self, name, mtype):
        # marker header definition
        marker = Marker()
        marker.header.frame_id = self.f_ref
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.MODIFY

        # identification properties
        marker.ns = name
        marker.id = 0
        marker.type = mtype
        marker.lifetime = Duration(sec=self.marker_duration)

        # return base marker
        return marker

    def publish_text_marker(self, name, points, cmap):
        # define marker array
        marker_array = MarkerArray()

        for index, point in enumerate(points):
            # generate base marker
            marker = self.create_basic_marker(name, Marker.TEXT_VIEW_FACING)
            marker.id = index

            # visual properties
            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

            # position and orientation
            marker.pose.position = point
            marker.pose.orientation = Quaternion(w=1.0)
            marker.text = "x"

            # add marker to list
            marker_array.markers.append(marker)

        # publish marker
        self.publisher_array.publish(marker_array)

    def publish_delete_all_marker(self):
        # define marker array
        marker_array = MarkerArray()

        # marker header definition
        marker = Marker()
        marker.header.frame_id = self.f_ref
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL

        # add marker to list
        marker_array.markers.append(marker)

        # publish marker
        self.publisher_array.publish(marker_array)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    navigator = BasicNavigator(node_name="cafeteriabot_navigator")
    node = CafeteriaRobotControlNode(navigator=navigator)

    try:
        # add node to executor and spin
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # signals the thread to finish
        node.cancel_event.set()

        # cancel any active navigator task
        navigator.cancelTask()
        navigator.destroyNode()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
