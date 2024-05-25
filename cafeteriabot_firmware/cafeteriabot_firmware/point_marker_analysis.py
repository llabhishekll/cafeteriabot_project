#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class PointMarkerAnalysisNode(Node):
    def __init__(self):
        super().__init__("point_marker_analysis_node")
        # member variables
        self.map_data = None

        # node parameters
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.marker_duration = self.declare_parameter("marker_duration", 5).value

        # node modification
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ros objects
        self.subscriber_map = self.create_subscription(
            OccupancyGrid, "/map", self.subscriber_map_callback, qos_profile
        )
        self.subscriber_click = self.create_subscription(
            PointStamped, "/clicked_point", self.subscriber_click_callback, 10
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

    def subscriber_map_callback(self, message):
        self.map_data = message
        self.get_logger().info(f"Received map data: {message.info.height}x{message.info.width}")

    def subscriber_click_callback(self, message):
        # fetch (x, y) of clicked point
        px = round(message.point.x, 2)
        py = round(message.point.y, 2)
        self.get_logger().info(f"Received point coordinates: ({px}, {py})")

        # check clicked point location
        self.check_point_location(px, py)

    def check_point_location(self, x, y):
        # convert map coordinates to grid coordinates
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # determines whether a point is 'free', 'occupied', or 'other'
        if 0 <= mx < self.map_data.info.width and 0 <= my < self.map_data.info.height:
            # calculate the index and value in the flat array
            index = mx + my * self.map_data.info.width
            occupancy_value = self.map_data.data[index]

            # compare value to find status of the point
            if occupancy_value == -1:
                self.get_logger().info("Point is in unknown area.")
            elif occupancy_value == 0:
                self.get_logger().info("Point is in free area.")
                self.publish_sphere_marker("points_clicked", (x, y), (0.0, 1.0, 0.0))
            else:
                self.get_logger().info("Point is in occupied area.")
                self.publish_sphere_marker("points_clicked", (x, y), (1.0, 0.0, 0.0))
        else:
            self.get_logger().info("Point is outside the map bounds.")

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

    def publish_sphere_marker(self, name, position, cmap):
        # generate base marker
        marker = self.create_basic_marker(name, Marker.SPHERE)

        # visual properties
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

        # position and orientation
        marker.pose.position = Point(x=position[0], y=position[1], z=0.0)
        marker.pose.orientation = Quaternion(w=1.0)

        # publish marker to topic
        self.publisher_marker.publish(marker)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PointMarkerAnalysisNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
