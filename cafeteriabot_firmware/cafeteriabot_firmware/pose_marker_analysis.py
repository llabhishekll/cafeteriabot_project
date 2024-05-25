#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener, Buffer

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Vector3
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class PoseMarkerAnalysisNode(Node):
    def __init__(self):
        super().__init__("pose_marker_analysis_node")
        # member variables

        # node parameters
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_base_footprint").value
        self.marker_duration = self.declare_parameter("marker_duration", 5).value

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ros objects
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscriber_amcl_callback, 10
        )
        self.subscriber_odom = self.create_subscription(
            Odometry, "/odom", self.subscriber_odom_callback, 10
        )
        self.timer_transformation = self.create_timer(
            1.0, self.timer_transformation_callback
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

            # extract position and orientation
            translation = transform.transform.translation
            orientation = transform.transform.rotation

            # convert to correct message form
            position = Point(x=translation.x, y=translation.y, z=translation.z)

            # publish tf marker
            self.publish_arrow_marker("arrow_transform", (position, orientation), (1.0, 0.0, 0.0))
        except Exception as e:
            self.get_logger().warn(f"Could not transform from '{self.f_ref}' to '{self.t_ref}'")

    def subscriber_amcl_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        # publish amcl marker
        self.publish_arrow_marker("arrow_amcl", (position, orientation), (0.0, 0.0, 1.0))

    def subscriber_odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        # publish odom marker
        self.publish_arrow_marker("arrow_odom", (position, orientation), (0.0, 1.0, 0.0))

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

    def publish_arrow_marker(self, name, pose, cmap):
        # generate base marker
        marker = self.create_basic_marker(name, Marker.ARROW)

        # visual properties
        marker.scale = Vector3(x=0.1, y=0.05, z=0.01)
        marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

        # position and orientation
        marker.pose.position = pose[0]
        marker.pose.orientation = pose[1]

        # publish marker
        self.publisher_marker.publish(marker)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PoseMarkerAnalysisNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
