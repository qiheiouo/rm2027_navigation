#!/usr/bin/env python3
"""Same fixed step sequence as probe.py, sent through Nav2 velocity_smoother."""
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist

from probe import Probe


def main():
    rclpy.init()
    probe = Probe(Path(sys.argv[1]))
    # The direct probe owns /cmd_vel. Replace that publisher before the first
    # command; the independently launched smoother consumes /cmd_vel_nav.
    probe.destroy_publisher(probe.publisher)
    probe.publisher = probe.create_publisher(Twist, "/cmd_vel_nav", 10)
    probe.create_subscription(
        Twist, "/cmd_vel",
        lambda msg: probe.write(
            kind="smoothed", callback_sim_s=probe.stamp(),
            steady_ns=time.monotonic_ns(),
            velocity=[msg.linear.x, msg.linear.y, msg.angular.z]), 50)
    deadline = time.monotonic() + 90
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.02)
            if probe.tick():
                return 0
        probe.write(kind="timeout", steady_ns=time.monotonic_ns())
        return 1
    finally:
        probe.close()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
