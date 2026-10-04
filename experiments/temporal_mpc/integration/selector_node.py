#!/usr/bin/env python3
"""Experimental ROS selector supervisor. M3 plugin health producer is required.

It cannot enable MPC without fresh health and never publishes cmd_vel. Neither
this node nor the example BT/profile is launched by any formal project launch.
"""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from temporal_mpc.selection import Selection


class Selector(Node):
    def __init__(self):
        super().__init__("temporal_controller_selector")
        self.policy = Selection()
        self.published = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(String, "controller_selector", qos)
        self.create_subscription(String, "temporal_mpc/request_controller", self.request, 10)
        self.create_subscription(String, "temporal_mpc/health", self.health, 1)
        self.create_timer(.02, self.publish_selection)
        self.publish_selection()

    def request(self, msg):
        try:
            self.policy.request(msg.data, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(str(error))
        self.publish_selection()

    def health(self, msg):
        try:
            data = json.loads(msg.data)
            # Additional deployment checks belong to the plugin producer.
            self.policy.health(time.monotonic(), data["ready"], data["fallback_requested"])
        except (ValueError, TypeError, KeyError):
            self.policy.health(time.monotonic(), False, True)
        self.publish_selection()

    def publish_selection(self):
        selected = self.policy.tick(time.monotonic())
        if selected != self.published:
            self.output.publish(String(data=selected))
            self.get_logger().info(f"{selected}: {self.policy.reason}")
            self.published = selected


def main():
    rclpy.init()
    node = Selector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
