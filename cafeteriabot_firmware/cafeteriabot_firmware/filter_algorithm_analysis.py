#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformListener, Buffer

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class AnalyzeFilterAlgorithmNode(Node):
    def __init__(self):
        super().__init__("analyze_filter_algorithm_node")
        # member variables
        self.map_data = None
        self.scan_data = None
        self.px = None
        self.py = None
        self.yaw = None

        # node parameters
        self.debug = self.declare_parameter("debug", True).value
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_front_laser_base_link").value
        self.marker_duration = self.declare_parameter("marker_duration", 5).value
        self.distance_threshold = self.declare_parameter("distance_threshold", 1.9).value

        # node modification
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ros objects
        self.subscriber_map = self.create_subscription(
            OccupancyGrid, "/map", self.subscriber_map_callback, qos_profile
        )
        self.subscriber_scan = self.create_subscription(
            LaserScan, "/scan", self.subscriber_scan_callback, 10
        )
        self.timer_transformation = self.create_timer(
            1.0, self.timer_transformation_callback
        )
        self.timer_analyse = self.create_timer(
            2.0, self.analyse_laser_data
        )
        self.publisher_marker = self.create_publisher(
            Marker, "/visualization_marker", 10
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
                f"Could not transform from '{self.f_ref}' to '{self.t_ref}'", throttle_duration_sec=5
            )

    def subscriber_map_callback(self, message):
        self.map_data = message
        self.get_logger().info(f"Received map data: {message.info.height}x{message.info.width}")

    def subscriber_scan_callback(self, message):
        self.scan_data = message

    def analyse_laser_data(self):
        if not (self.map_data and self.scan_data and self.px and self.py and self.yaw):
            self.get_logger().warn("Awaiting map data, scan data and pose information.")
            return

        points = {"free": [], "occupied": [], "other": []}
        colors = {"free": (0.0, 0.66, 0.0), "occupied": (1.0, 0.0, 0.0), "other": (1.0, 1.0, 1.0)}

        for i, distance in enumerate(self.scan_data.ranges):
            # check if distance is within the threshold value
            if distance <= self.distance_threshold:
                # convert polar to (x, y) coordinates
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                x = distance * math.cos(-angle)
                y = distance * math.sin(-angle)

                # convert (x, y) point to map coordinates frame
                px = round(self.px + (math.cos(self.yaw) * x - math.sin(self.yaw) * y), 2)
                py = round(self.py + (math.sin(self.yaw) * x + math.cos(self.yaw) * y), 2)

                # determine point type and categorize accordingly
                point_type = self.get_point_type(px, py)
                points[point_type].append(Point(x=px, y=py, z=1.0))

        # publish scan markers
        for ptype, point_list in points.items():
            self.get_logger().info(f"Publishing {ptype} marker: {len(point_list)}")
            self.publish_points_marker(f"points_{ptype}", point_list, colors[ptype])

        # calculate points of circle
        points_boundary, num_points = [], 100
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            cx = self.px + self.distance_threshold * math.cos(angle)
            cy = self.py + self.distance_threshold * math.sin(angle)
            points_boundary.append(Point(x=cx, y=cy, z=0.0))

        # publish scan boundary
        self.publish_line_marker("points_boundary", points_boundary, (1.0, 0.0, 1.0))

        if not self.debug:
            self.destroy_timer(self.timer_analyse)

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

    def publish_points_marker(self, name, points, cmap):
        # generate base marker
        marker = self.create_basic_marker(name, Marker.POINTS)

        # visual properties
        marker.scale = Vector3(x=0.01, y=0.01, z=0.01)
        marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

        # position and orientation
        marker.points = points
        marker.pose.orientation = Quaternion(w=1.0)

        # publish marker
        self.publisher_marker.publish(marker)

    def publish_line_marker(self, name, points, cmap):
        # generate base marker
        marker = self.create_basic_marker(name, Marker.LINE_STRIP)

        # visual properties
        marker.scale = Vector3(x=0.02, y=0.0, z=0.0)
        marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

        # position and orientation
        marker.points = points
        marker.pose.orientation = Quaternion(w=1.0)

        # publish marker
        self.publisher_marker.publish(marker)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = AnalyzeFilterAlgorithmNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
