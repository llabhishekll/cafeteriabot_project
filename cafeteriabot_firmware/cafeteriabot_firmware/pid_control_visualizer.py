#!/usr/bin/env python3
import collections
import threading
import warnings

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from rcl_interfaces.msg import SetParametersResult

warnings.filterwarnings("ignore")

# External dependency
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Custom interface
from cafeteriabot_interface.msg import PIDValue


class PIDControlVisualizerNode(Node):
    def __init__(self):
        super().__init__("pid_control_visualizer_node")

        # member variables
        self.start_time = self.get_clock().now()
        self.time_percentile = [None, None, None, None]  # 25%, 50%, 75%, 100%

        # node parameters
        self.queue_limit = self.declare_parameter("queue_limit", 10000).value

        # initialize deque with fixed size for data storage
        self.time_data = collections.deque(maxlen=self.queue_limit)
        self.setpoint_data = collections.deque(maxlen=self.queue_limit)
        self.variable_data = collections.deque(maxlen=self.queue_limit)
        self.proportional_data = collections.deque(maxlen=self.queue_limit)
        self.integral_data = collections.deque(maxlen=self.queue_limit)
        self.differential_data = collections.deque(maxlen=self.queue_limit)
        self.output_data = collections.deque(maxlen=self.queue_limit)

        # ros objects
        self.subscriber_pid = self.create_subscription(
            PIDValue, "/pid_value", self.subscriber_pid_callback, 10
        )
        self.server_reset = self.create_service(
            Empty, "/reset_pid_values", self.server_reset_callback
        )

        # start the animation in a separate thread
        self.thread_animation = threading.Thread(target=self.thread_animation_callback)
        self.thread_animation.start()

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

    def subscriber_pid_callback(self, message):
        # calculate the current time since node started
        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.time_data.append(current_time)

        # append received values to deques
        self.setpoint_data.append(message.setpoint)
        self.variable_data.append(message.variable)
        self.proportional_data.append(message.proportional)
        self.integral_data.append(message.integral)
        self.differential_data.append(message.differential)
        self.output_data.append(message.output)

        # log received message details
        self.get_logger().info(
            f"Time: {current_time:.2f}s "
            f"(SP): {message.setpoint:.2f}, (PV): {message.variable:.2f} | Output: {message.output:.2f}, "
            f"(P): {message.proportional:.2f}, (I): {message.integral:.2f}, (D): {message.differential:.2f}"
        )

        # check for times to reach specific percentages of setpoint
        if self.time_percentile[0] is None and message.variable >= 0.25 * message.setpoint:
            self.time_percentile[0] = current_time
        if self.time_percentile[1] is None and message.variable >= 0.50 * message.setpoint:
            self.time_percentile[1] = current_time
        if self.time_percentile[2] is None and message.variable >= 0.75 * message.setpoint:
            self.time_percentile[2] = current_time
        if self.time_percentile[3] is None and message.variable >= 1.00 * message.setpoint:
            self.time_percentile[3] = current_time

    def server_reset_callback(self, request, response):
        # clear value deques
        self.proportional_data.clear()
        self.integral_data.clear()
        self.differential_data.clear()
        self.output_data.clear()
        self.variable_data.clear()
        self.setpoint_data.clear()
        self.time_data.clear()
        self.time_percentile = [None, None, None, None]

        # log information and return
        self.get_logger().info("PID values and time percentiles have been reset.")
        return response

    def thread_animation_callback(self):
        # create subplot
        fig, (ax1, ax2) = plt.subplots(2, 1)

        def animate(i):
            # clear previous plots
            ax1.clear()
            ax2.clear()

            # plot PV and SP data
            ax1.plot(self.time_data, self.variable_data, label="Process Variable (PV)")
            ax1.plot(self.time_data, self.setpoint_data, label="Setpoint (SP)", linestyle="--")

            # plot time percentiles as vertical lines
            for time_point in self.time_percentile:
                if time_point is not None:
                    ax1.axvline(x=time_point, color="grey", linestyle=":", alpha=0.5)

            ax1.legend()
            ax1.set_ylabel("Value")
            ax1.set_title("Process Variable and Setpoint Over Time")

            # plot PID and output data
            ax2.plot(self.time_data, self.proportional_data, label="P")
            ax2.plot(self.time_data, self.integral_data, label="I")
            ax2.plot(self.time_data, self.differential_data, label="D")
            ax2.plot(self.time_data, self.output_data, label="Controller Output")

            # plot time percentiles as vertical lines
            for time_point in self.time_percentile:
                if time_point is not None:
                    ax2.axvline(x=time_point, color="grey", linestyle=":", alpha=0.5)

            ax2.legend()
            ax2.set_ylabel("Output / Gain")
            ax2.set_title("PID and Controller Output Over Time")

            # configure axes
            for ax in (ax1, ax2):
                ax.set_xlabel("Time (s)")
                ax.relim()
                ax.autoscale_view()

        # set up the animation
        animation_object = animation.FuncAnimation(fig, animate, interval=1000)

        # show plot
        plt.tight_layout()
        plt.show()


def main(args=None):
    # initialize ROS
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PIDControlVisualizerNode()

    # add node to executor and spin
    executor.add_node(node)
    executor.spin()

    # shutdown ROS
    rclpy.shutdown()


if __name__ == "__main__":
    main()
