#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener, Buffer

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion, Vector3
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class ClusterAlgorithmAnalysisNode(Node):
    def __init__(self):
        super().__init__("cluster_algorithm_analysis_node")
        # member variables
        self.scan_data = None
        self.px = None
        self.py = None
        self.yaw = None

        # node parameters
        self.debug = self.declare_parameter("debug", True).value
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.t_ref = self.declare_parameter("t_ref", "robot_front_laser_base_link").value
        self.distance_threshold = self.declare_parameter("distance_threshold", 0.1).value
        self.min_points = self.declare_parameter("min_points", 10).value
        self.max_points = self.declare_parameter("max_points", 50).value
        self.marker_duration = self.declare_parameter("marker_duration", 5).value

        # transformation objects
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ros objects
        self.subscriber_scan = self.create_subscription(
            LaserScan, "/scan_filtered", self.subscriber_scan_callback, 10
        )
        self.timer_analyse = self.create_timer(
            2.0, self.analyse_cluster_data
        )
        self.publisher_marker = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        self.timer_transformation = self.create_timer(
            1.0, self.timer_transformation_callback
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

    def subscriber_scan_callback(self, message):
        self.scan_data = message

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
                throttle_duration_sec=5,
            )

    def analyse_cluster_data(self):
        if not (self.scan_data and self.px and self.py and self.yaw):
            self.get_logger().warn("Awaiting scan data and pose information.")
            return

        points = []
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
            points.append([px, py])

        # find cluster
        self.cluster_by_distance(points)
        self.cluster_by_neighbour(points)

        if not self.debug:
            self.destroy_timer(self.timer_analyse)

    def cluster_by_distance(self, points):
        centroids, current_cluster, centroids_meta = [], [], []
        for px, py in points:
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

        # publish centroids marker
        self.get_logger().info(
            f"Centroids by distance  : {len(centroids)}/{len(centroids_meta)} Details: {centroids_meta}",
            throttle_duration_sec=1,
        )
        self.publish_points_marker("centroids_distance", centroids, (1.0, 0.0, 1.0))

    def cluster_by_neighbour(self, points):
        clusters = []
        # find the cluster of the (px, py) point
        for point in points:
            added = False
            for group in clusters:
                if any(self.euclidean_distance(point, member) <= self.distance_threshold for member in group):
                    group.append(point)
                    added = True
                    break
            if not added:
                clusters.append([point])

        centroids, centroids_meta = [], []
        for group in clusters:
            # filter cluster based on size constraints
            if self.is_valid_cluster_size(group):
                # calculate centroids for filtered cluster
                cx, cy = self.find_centroid(group)
                centroids.append(Point(x=cx, y=cy, z=1.0))
            centroids_meta.append(len(group))

        # publish centroids marker
        self.get_logger().info(
            f"Centroids by neighbour : {len(centroids)}/{len(centroids_meta)} Details: {centroids_meta}",
            throttle_duration_sec=1,
        )
        self.publish_points_marker("centroids_neighbour", centroids, (0.0, 1.0, 1.0))

    def is_valid_cluster_size(self, cluster):
        # check if a cluster meets size constraints
        return self.min_points <= len(cluster) <= self.max_points

    def euclidean_distance(self, point1, point2):
        # calculate the euclidean distance between two points
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def find_centroid(self, cluster):
        # finds the centroid of a cluster
        xs, ys = zip(*cluster)
        return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

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
        marker.scale = Vector3(x=0.05, y=0.05, z=0.01)
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
    node = ClusterAlgorithmAnalysisNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
