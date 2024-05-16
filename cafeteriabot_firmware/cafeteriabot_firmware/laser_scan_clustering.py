#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener, Buffer

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header


class LaserScanClusteringNode(Node):
    def __init__(self):
        super().__init__("laser_scan_clustering_node")
        # member variables
        self.scan_data = None
        self.px = None
        self.py = None
        self.yaw = None

        # node parameters
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_front_laser_base_link").value
        self.publish_rate = self.declare_parameter("publish_rate", 1.0).value
        self.distance_threshold = self.declare_parameter("distance_threshold", 0.1).value
        self.min_points = self.declare_parameter("min_points", 10).value
        self.max_points = self.declare_parameter("max_points", 50).value
        self.stale_duration = self.declare_parameter("stale_duration", 1).value

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # callback groups
        self.callback_g1 = MutuallyExclusiveCallbackGroup()
        self.callback_g2 = MutuallyExclusiveCallbackGroup()

        # ros objects
        self.subscriber_scan = self.create_subscription(
            LaserScan, "/scan_filtered", self.subscriber_scan_callback, 10,
        )
        self.publisher_centroid = self.create_publisher(
            PoseArray, "/centroid", 10
        )
        self.timer_cluster = self.create_timer(
            self.publish_rate, self.timer_cluster_callback, callback_group=self.callback_g2,
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

    def subscriber_scan_callback(self, message):
        # update the last scan timestamp to the current time
        self.last_update_time = self.get_clock().now()

        # update laser scan data
        self.scan_data = message

    def timer_cluster_callback(self):
        if not (self.scan_data and self.px and self.py and self.yaw):
            self.get_logger().warn("Awaiting scan data and pose information.", skip_first=True, throttle_duration_sec=1)
            return

        # calculate the elapsed time since the last update
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.last_update_time

        # check if the elapsed time has exceeded the stale duration threshold
        if elapsed_time.nanoseconds * 1e-9 > self.stale_duration:
            # reset scan data
            self.scan_data = None

            # no new scan within the stale duration
            self.get_logger().warn(f"No new scan received for {self.stale_duration} seconds.")
            return

        centroids, current_cluster, centroids_meta = [], [], []

        for i, distance in enumerate(self.scan_data.ranges):
            # skip invalid (non-positive or infinity) distances
            if distance <= 0 or distance == float("inf"):
                continue

            # convert polar to (x, y) coordinates
            angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
            x = distance * math.cos(-angle)
            y = distance * math.sin(-angle)

            # convert (x, y) point to map coordinates frame
            px = round(self.px + (math.cos(self.yaw) * x - math.sin(self.yaw) * y), 2)
            py = round(self.py + (math.sin(self.yaw) * x + math.cos(self.yaw) * y), 2)

            # find the cluster of the (px, py) point
            if not current_cluster:
                current_cluster.append((px, py))
            else:
                last_point = current_cluster[-1]
                if self.euclidean_distance((px, py), last_point) <= self.distance_threshold:
                    current_cluster.append((px, py))
                else:
                    if self.is_valid_cluster_size(current_cluster):
                        # calculate cluster centroids
                        cx, cy = self.find_centroid(current_cluster)
                        centroids.append(Point(x=cx, y=cy, z=1.0))
                    # start fresh cluster
                    centroids_meta.append(len(current_cluster))
                    current_cluster = [(px, py)]

        if current_cluster and self.is_valid_cluster_size(current_cluster):
            cx, cy = self.find_centroid(current_cluster)
            centroids.append(Point(x=cx, y=cy, z=1.0))
            centroids_meta.append(len(current_cluster))

        # publish cluster points
        self.publish_centroids_points(centroids)
        self.get_logger().info(
            f"Centroids: {len(centroids)}/{len(centroids_meta)} Details: {centroids_meta}", throttle_duration_sec=1
        )

    def euclidean_distance(self, point1, point2):
        # calculate the euclidean distance between two points
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def is_valid_cluster_size(self, cluster):
        # check if a cluster meets size constraints
        return self.min_points <= len(cluster) <= self.max_points

    def find_centroid(self, cluster):
        # finds the centroid of a cluster
        xs, ys = zip(*cluster)
        return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

    def publish_centroids_points(self, centroids):
        # message header definition
        message = PoseArray()
        message.header = Header(frame_id=self.f_ref)
        message.header.stamp = self.get_clock().now().to_msg()

        # create pose array
        for point in centroids:
            pose = Pose()
            pose.position = point
            pose.orientation = Quaternion(w=1.0)
            message.poses.append(pose)

        # publish message
        self.publisher_centroid.publish(message)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = LaserScanClusteringNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
