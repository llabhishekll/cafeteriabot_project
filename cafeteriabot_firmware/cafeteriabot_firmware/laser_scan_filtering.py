#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformListener, Buffer

from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan


class LaserScanFilteringNode(Node):
    def __init__(self):
        super().__init__("laser_scan_filtering_node")
        # member variables
        self.map_data = None
        self.px = None
        self.py = None
        self.yaw = None

        # node parameters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_front_laser_base_link").value
        self.min_distance = self.declare_parameter("min_distance", 0.1).value
        self.max_distance = self.declare_parameter("max_distance", 2.0).value

        # node modification
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.orientation_factor = -1 if self.use_sim_time else 1

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # callback groups
        self.callback_g1 = MutuallyExclusiveCallbackGroup()
        self.callback_g2 = ReentrantCallbackGroup()

        # ros objects
        self.subscriber_map = self.create_subscription(
            OccupancyGrid, "/map", self.subscriber_map_callback, qos_profile,
        )
        self.subscriber_scan = self.create_subscription(
            LaserScan, "/scan", self.subscriber_scan_callback, 10, callback_group=self.callback_g2,
        )
        self.publisher_filtered = self.create_publisher(
            LaserScan, "/scan_filtered", 10
        )
        self.timer_transformation = self.create_timer(
            0.2, self.timer_transformation_callback, callback_group=self.callback_g1,
        )

        # parameter handler
        self.add_on_set_parameters_callback(self.on_parameter_changed)

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
                f"Could not transform from '{self.f_ref}' to '{self.t_ref}'", skip_first=True, throttle_duration_sec=5
            )

    def subscriber_map_callback(self, message):
        self.map_data = message
        self.get_logger().info(f"Received map data: {message.info.height}x{message.info.width}")

    def subscriber_scan_callback(self, message):
        if not (self.map_data and self.px and self.py and self.yaw):
            self.get_logger().warn("Awaiting map data and pose information.", skip_first=True, throttle_duration_sec=1)
            return

        # initialize modified_ranges with 0.0 for all distances
        modified_ranges = [0.0] * len(message.ranges)

        for i, distance in enumerate(message.ranges):
            # check if distance is within the threshold value
            if self.min_distance <= distance <= self.max_distance:
                # convert polar to (x, y) coordinates
                angle = self.orientation_factor * (message.angle_min + i * message.angle_increment)
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                # convert (x, y) point to map coordinates frame
                px = round(self.px + (math.cos(self.yaw) * x - math.sin(self.yaw) * y), 2)
                py = round(self.py + (math.sin(self.yaw) * x + math.cos(self.yaw) * y), 2)

                # determine point type and update distance
                if self.get_point_type(px, py) == "free":
                    modified_ranges[i] = distance

        # publish filtered ranges
        message.ranges = modified_ranges
        self.publisher_filtered.publish(message)
        self.get_logger().info("Publishing the filtered data to /scan_filtered topic.", once=True)

    def get_point_type(self, x, y):
        # convert map coordinates to grid coordinates
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # determines whether a point is 'free', 'occupied', or 'other'
        if 0 <= mx < self.map_data.info.width and 0 <= my < self.map_data.info.height:
            # calculate the index and value in the flat array
            index = mx + my * self.map_data.info.width
            occupancy_value = self.map_data.data[index]
            # compare value to find status of the point
            if occupancy_value == 0:
                return "free"
            elif occupancy_value >= 0:
                return "occupied"
        return "other"


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = LaserScanFilteringNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
