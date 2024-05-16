#!/usr/bin/env python3
import math
import itertools
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseArray, Quaternion, Vector3
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# custom interface
from cafeteriabot_interface.msg import TableGeometry
from cafeteriabot_interface.srv import TableAvailable


class LaserScanDetectionNode(Node):
    def __init__(self):
        super().__init__("laser_scan_detection_node")
        # member variables
        self.centroids = None
        self.centroids_distance = {}
        self.centroids_angle = {}
        self.center = None
        self.footprint = None
        self.midpoints = None
        self.last_update_time = ROSClock().now()
        self.last_find_time = ROSClock().now()

        # node parameters
        self.f_ref = self.declare_parameter("f_ref", "map").value
        self.publish_rate = self.declare_parameter("publish_rate", 2.0).value
        self.min_distance = self.declare_parameter("min_distance", 0.5).value
        self.max_distance = self.declare_parameter("max_distance", 1.0).value
        self.angle_tolerance = self.declare_parameter("angle_tolerance", 5).value
        self.marker_duration = self.declare_parameter("marker_duration", 1).value
        self.stale_duration = self.declare_parameter("stale_duration", 5).value

        # callback groups
        self.callback_g1 = MutuallyExclusiveCallbackGroup()

        # ros objects
        self.subscriber_centroid = self.create_subscription(
            PoseArray, "/centroid", self.subscriber_centroid_callback, 10
        )
        self.publisher_table = self.create_publisher(
            TableGeometry, "/table", 10
        )
        self.publisher_marker = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        self.server_table = self.create_service(
            TableAvailable, "/is_available", self.service_availability_callback,
        )
        self.timer_detection = self.create_timer(
            self.publish_rate, self.timer_detection_callback, callback_group=self.callback_g1,
        )
        self.timer_publisher = self.create_timer(
            self.publish_rate, self.timer_publisher_callback, callback_group=self.callback_g1,
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

    def subscriber_centroid_callback(self, message):
        # update the last_centroid_update timestamp to the current time
        self.last_update_time = self.get_clock().now()

        # extract cluster centroids from the PoseArray message
        self.centroids = [(pose.position.x, pose.position.y) for pose in message.poses]

    def service_availability_callback(self, request, response):
        self.get_logger().warn("Service /is_available received new request.")

        # requested duration to check for table availability
        check_duration = request.duration

        # iterate over the specified duration, checking table availability
        for i in range(check_duration):
            if self.center:
                response.success = True
                response.message = f"{self.center}"
                return response
            else:
                # wait before checking again
                self.get_logger().info(f"/is_available is waiting for table {i}.")
                time.sleep(1)

        # if no table was found within the duration
        response.success = False
        response.message = "No table found within the requested duration."
        return response

    def timer_publisher_callback(self):
        # calculate the elapsed time since the last update
        current_time = self.get_clock().now()
        elapsed_update_time = current_time - self.last_update_time

        # check if the elapsed time has exceeded the stale duration threshold
        if elapsed_update_time.nanoseconds * 1e-9 > self.stale_duration:
            # reset centroids, other data
            self.centroids = None
            self.center = None
            self.footprint = None
            self.midpoints = None

            # no new centroids within the stale duration
            self.get_logger().warn(f"No new centroids received for {self.stale_duration} seconds.")

        # calculate the elapsed time since the last valid find
        elapsed_find_time = current_time - self.last_find_time

        # check if the elapsed time has exceeded the stale duration threshold
        if elapsed_find_time.nanoseconds * 1e-9 > self.stale_duration:
            # reset valid center, footprint, and midpoints
            self.center = None
            self.footprint = None
            self.midpoints = None

            # no new centroids within the stale duration
            self.get_logger().warn(f"No new valid shape found for {self.stale_duration} seconds.")

        if self.center and self.footprint:
            # publish marker
            x, y = self.center[0], self.center[1]
            self.publish_sphere_marker("table_center", (x, y), (1.0, 0.0, 0.0))

            footprint = [Point(x=x, y=y, z=1.0) for x, y in self.footprint]
            footprint.append(Point(x=self.footprint[0][0], y=self.footprint[0][1], z=1.0))
            self.publish_line_marker("table_footprint", footprint, (1.0, 0.0, 0.0))

            midpoints = [Point(x=x, y=y, z=1.0) for x, y in self.midpoints]
            self.publish_points_marker("table_midpoints", midpoints, (0.0, 0.0, 1.0))

            # publish table to topic
            self.publish_table()
        else:
            # delete all published marker
            self.publish_delete_all_marker()

    def timer_detection_callback(self):
        # check if there are enough centroids to form a polygon (at least 3)
        if not self.centroids or len(self.centroids) < 3:
            self.get_logger().warn(
                f"Insufficient centroids data expected at least 3, found {len(self.centroids) if self.centroids else 0}.",
                skip_first=True,
                throttle_duration_sec=1,
            )
            return

        self.centroids_distance = {}
        self.centroids_angle = {}

        # check for quadrilateral first, then a triangle
        start_time = self.get_clock().now()
        for shape in ["quadrilateral", "triangle"]:
            polygon_points = self.find_valid_polygon_points(shape, self.centroids)
            if polygon_points:
                # if a valid polygon is found, calculate its centroid
                sorted_points, midpoints, centroid = self.calculate_polygon_footprint(shape, polygon_points)

                # update last_find_time to current time
                self.last_find_time = self.get_clock().now()

                # update the found valid points and their centroid
                self.center = centroid
                self.footprint = sorted_points
                self.midpoints = midpoints

                # log information and return
                execution_time = round((self.get_clock().now() - start_time).nanoseconds / 1e9, 5)
                self.get_logger().info(f"Centroid found for {shape}: {centroid} in {execution_time}.")
                return

        self.get_logger().warn("No valid shape found for table")

    def find_valid_polygon_points(self, shape, points):
        # needed parametric value for shape
        criteria = {"quadrilateral": (4, 4, 4), "triangle": (3, 2, 1)}
        num_sides, valid_distance_count, valid_angle_count = criteria[shape]

        # iterate over all combinations of points that could form the specified shape
        for polygon_points in itertools.combinations(points, num_sides):
            complement = [point for point in points if point not in polygon_points]
            # validate the polygon based on exteriority
            if self.is_valid_by_exteriority(polygon_points, complement):
                # validate the polygon based on distances criteria
                if self.is_valid_by_distance(polygon_points, valid_distance_count):
                    # validate the polygon based on angle criteria
                    if self.is_valid_by_angle(polygon_points, valid_angle_count):
                        # return the first valid combinations
                        return polygon_points
        return []

    def is_valid_by_exteriority(self, polygon_points, complement):
        # iterate over complement and perform ray-casting algorithm
        for point in complement:
            x, y = point
            intersections = 0
            vertices_count = len(polygon_points)
            # iterate over each edge of the polygon
            for i in range(vertices_count):
                vertex1 = polygon_points[i]
                vertex2 = polygon_points[(i + 1) % vertices_count]
                min_y = min(vertex1[1], vertex2[1])
                max_y = max(vertex1[1], vertex2[1])
                if min_y < y <= max_y:
                    if vertex2[1] != vertex1[1]:
                        slope = (vertex2[0] - vertex1[0]) / (vertex2[1] - vertex1[1])
                    else:
                        slope = 0
                    # check to find intersections happened
                    intersection_x = (y - vertex1[1]) * slope + vertex1[0]
                    if x < intersection_x:
                        intersections += 1
            if intersections % 2 != 0:
                return False
        return True

    def is_valid_by_distance(self, points, valid_distance_count, epsilon=1e-7):
        valid_sides = 0
        points_meta = []
        for pair in itertools.combinations(points, 2):
            # ensure uniqueness and consistency in distance keys
            pair = tuple(sorted(pair, key=lambda x: (x[0], x[1])))

            # check if the combination of points has already been calculated
            if pair not in self.centroids_distance:
                self.centroids_distance[pair] = self.calculate_distance(*pair)

            # check if the distance is within a max and min tolerance
            distance = self.centroids_distance[pair]
            if self.min_distance + epsilon < distance < self.max_distance + epsilon:
                valid_sides += 1

            # if the number of valid angles meets or exceeds the required count
            if valid_sides >= valid_distance_count:
                return True

            points_meta.append((pair, distance))

        self.get_logger().debug(f"Points rejected by distance: {points_meta}")
        return False

    def is_valid_by_angle(self, points, valid_angle_count, epsilon=1e-7):
        valid_angles = 0
        points_meta = []
        for triple in itertools.combinations(points, 3):
            for triple_pnc in itertools.permutations(triple):
                # check if the combination of points has already been calculated
                if triple_pnc not in self.centroids_angle:
                    self.centroids_angle[triple_pnc] = self.calculate_angle(*triple_pnc)

                # check if the angle is close to 90 degrees, within a tolerance
                angle = self.centroids_angle[triple_pnc]
                if abs(angle - 90) < self.angle_tolerance + epsilon:
                    valid_angles += 1

                # if the number of valid angles meets or exceeds the required count
                if valid_angles >= valid_angle_count:
                    return True

                points_meta.append((triple_pnc, angle))

        self.get_logger().debug(f"Points rejected by angle: {points_meta}")
        return False

    def calculate_polygon_footprint(self, shape, polygon_points):
        # calculate centroid based on shape
        num_points = len(polygon_points)
        if shape == "triangle" and num_points == 3:
            point1, point2, point3 = polygon_points

            # identify the hypotenuse based on distances
            distances = [
                ((point1, point2), self.calculate_distance(point1, point2)),
                ((point1, point3), self.calculate_distance(point1, point3)),
                ((point2, point3), self.calculate_distance(point2, point3)),
            ]
            hypotenuse_points, _ = max(distances, key=lambda x: x[1])

            # calculate the centroid by finding the mid-point of points of hypotenuse
            cx = round((hypotenuse_points[0][0] + hypotenuse_points[1][0]) / 2, 2)
            cy = round((hypotenuse_points[0][1] + hypotenuse_points[1][1]) / 2, 2)

            # sort points by angle of each point from centroid
            angles = [math.atan2(point[1] - cy, point[0] - cx) for point in polygon_points]
            sorted_points = [point for _, point in sorted(zip(angles, polygon_points))]

            # calculate midpoints of all edges except hypotenuse
            midpoints = []
            for i in range(num_points):
                point1 = sorted_points[i]
                point2 = sorted_points[(i + 1) % num_points]
                if not ((point1, point2) == hypotenuse_points or (point2, point1) == hypotenuse_points):
                    midpoint_x = round((point1[0] + point2[0]) / 2, 2)
                    midpoint_y = round((point1[1] + point2[1]) / 2, 2)
                    midpoints.append((midpoint_x, midpoint_y))

        elif shape == "quadrilateral" and num_points == 4:
            # calculate the centroid by averaging the x and y coordinates of all points
            cx = round(sum(point[0] for point in polygon_points) / num_points, 2)
            cy = round(sum(point[1] for point in polygon_points) / num_points, 2)

            # sort points by angle of each point from centroid
            angles = [math.atan2(point[1] - cy, point[0] - cx) for point in polygon_points]
            sorted_points = [point for _, point in sorted(zip(angles, polygon_points))]

            # calculate midpoints of all edges
            midpoints = []
            for i in range(num_points):
                point1 = sorted_points[i]
                point2 = sorted_points[(i + 1) % num_points]
                midpoint_x = round((point1[0] + point2[0]) / 2, 2)
                midpoint_y = round((point1[1] + point2[1]) / 2, 2)
                midpoints.append((midpoint_x, midpoint_y))
        else:
            self.get_logger().error(f"Reached unknown state for centroid calculation {shape} {num_points}")

        # return polygon centroid and midpoints
        return sorted_points, midpoints, (cx, cy)

    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def calculate_angle(self, point1, point2, point3):
        # calculate squared distances between points
        a2 = self.calculate_distance(point1, point2) ** 2
        b2 = self.calculate_distance(point2, point3) ** 2
        c2 = self.calculate_distance(point3, point1) ** 2

        # avoid division by zero
        denominator = 2 * math.sqrt(a2) * math.sqrt(b2)
        if denominator == 0:
            return 0

        # clamps the value to the valid range of acos to avoid ValueError
        cos_angle = (a2 + b2 - c2) / denominator
        cos_angle = math.acos(min(1, max(cos_angle, -1)))

        # return angle in degree
        return math.degrees(cos_angle)

    def publish_table(self):
        # publishing to the topic /table
        message = TableGeometry()

        # define header
        message.stamp = self.get_clock().now().to_msg()

        # define table table structure
        message.center = Point(x=self.center[0], y=self.center[1], z=1.0)
        message.footprint = [Point(x=x, y=y, z=1.0) for x, y in self.footprint]
        message.midpoints = [Point(x=x, y=y, z=1.0) for x, y in self.midpoints]

        # publish message to topic
        self.publisher_table.publish(message)

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
        marker.scale = Vector3(x=0.05, y=0.05, z=0.01)
        marker.color = ColorRGBA(a=1.0, r=cmap[0], g=cmap[1], b=cmap[2])

        # position and orientation
        marker.pose.position = Point(x=position[0], y=position[1], z=0.0)
        marker.pose.orientation = Quaternion(w=1.0)

        # publish marker to topic
        self.publisher_marker.publish(marker)

    def publish_points_marker(self, name, points, cmap):
        # generate base marker
        marker = self.create_basic_marker(name, Marker.POINTS)

        # visual properties
        marker.scale = Vector3(x=0.025, y=0.025, z=0.01)
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

    def publish_delete_all_marker(self):
        # marker header definition
        marker = Marker()
        marker.header.frame_id = self.f_ref
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL

        # publish marker
        self.publisher_marker.publish(marker)


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = LaserScanDetectionNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
