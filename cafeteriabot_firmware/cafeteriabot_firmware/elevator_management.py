#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Point32, Polygon
from rcl_interfaces.msg import SetParametersResult


class ElevatorManagementNode(Node):
    def __init__(self):
        super().__init__("elevator_management_node")
        # member variables
        self.previous_action = "unknown"

        # node parameters
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.robot_radius = self.declare_parameter("robot_radius", 0.22).value
        self.table_radius = self.declare_parameter("table_radius", 0.35).value

        # ros objects
        self.server_elevator = self.create_service(
            SetBool, "/elevator_control", self.elevator_control_callback
        )
        self.publisher_gfootprint = self.create_publisher(
            Polygon, "/global_costmap/footprint", 10
        )
        self.publisher_lfootprint = self.create_publisher(
            Polygon, "/local_costmap/footprint", 10
        )
        self.publisher_elevator_up = self.create_publisher(
            String, "/elevator_up", 10
        )
        self.publisher_elevator_down = self.create_publisher(
            String, "/elevator_down", 10
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

    def elevator_control_callback(self, request, response):
        # determine action based on the request
        action = "up" if request.data else "down"

        # elevator up
        if action == "up":
            self.publisher_elevator_up.publish(String())
            # calculate footprint for cart
            footprint = self.polygon_message_rectangle(radius=self.table_radius)
            # response message
            message = f"Elevator moved up, earlier {self.previous_action}"
        # elevator down
        else:
            self.publisher_elevator_down.publish(String())
            # calculate footprint for robot
            footprint = self.polygon_message_rectangle(radius=self.robot_radius)
            # response message
            message = f"Elevator moved down, earlier {self.previous_action}"

        # service logger
        self.get_logger().info(f"{message}")

        # update robot footprint
        self.publisher_gfootprint.publish(footprint)
        self.publisher_lfootprint.publish(footprint)

        # maintain history
        self.previous_action = action

        # sleep for few seconds
        time.sleep(2)

        # send response
        response.success = True
        response.message = message
        return response

    def polygon_message_rectangle(self, radius):
        # initialize message
        polygon = Polygon()

        # define the corners of the rectangle (centered at origin)
        points = [
            Point32(x=radius, y=radius, z=0.0),  # top right
            Point32(x=radius, y=-radius, z=0.0),  # bottom right
            Point32(x=-radius, y=-radius, z=0.0),  # bottom left
            Point32(x=-radius, y=radius, z=0.0),  # top left
        ]
        polygon.points = points
        return polygon


def main(args=None):
    # initialize ros, executor and node
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ElevatorManagementNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
