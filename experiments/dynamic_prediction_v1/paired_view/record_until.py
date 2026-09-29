#!/usr/bin/env python3
"""Record scans and odometry until a fixed simulation time, without a goal."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, output):
        super().__init__("paired_view_observer")
        self.scans = (output / "scans.jsonl").open("x", buffering=1)
        self.odometry = (output / "odometry.jsonl").open("x", buffering=1)
        self.targets = (output / "moving_target.jsonl").open("x", buffering=1)
        self.scan_count = 0
        self.odom_count = 0
        self.target_count = 0
        self.clock_count = 0
        self.clock_t = None
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odometry, 10)
        self.create_subscription(Float64, "/simulation/moving_obstacle/target",
                                 self.on_target, 10)

    def on_clock(self, msg):
        self.clock_count += 1
        self.clock_t = seconds(msg.clock)

    def on_scan(self, msg):
        self.scan_count += 1
        self.scans.write(json.dumps({"t": seconds(msg.header.stamp),
            "frame": msg.header.frame_id, "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment, "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": [value if math.isfinite(value) else
                       ("inf" if value > 0 else "invalid") for value in msg.ranges]}) + "\n")

    def on_odometry(self, msg):
        self.odom_count += 1
        self.odometry.write(json.dumps({"t": seconds(msg.header.stamp),
            "frame": msg.header.frame_id, "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_z": msg.pose.pose.orientation.z,
            "yaw_w": msg.pose.pose.orientation.w}) + "\n")

    def on_target(self, msg):
        self.target_count += 1
        self.targets.write(json.dumps({"t": self.clock_t,
                                       "target": msg.data}) + "\n")

    def close(self):
        for stream in (self.scans, self.odometry, self.targets):
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stop-sim-s", type=float, default=44.0)
    parser.add_argument("--wall-timeout-s", type=float, default=240.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Recorder(args.output)
    deadline = time.monotonic() + args.wall_timeout_s
    try:
        while rclpy.ok() and (node.clock_t is None or node.clock_t < args.stop_sim_s):
            if time.monotonic() >= deadline:
                raise TimeoutError("fixed observation wall timeout")
            rclpy.spin_once(node, timeout_sec=0.05)
        summary = {"schema": "paired_view_observation/v1",
                   "stop_sim_s": args.stop_sim_s,
                   "actual_stop_sim_s": node.clock_t,
                   "clock_count": node.clock_count,
                   "scan_count": node.scan_count,
                   "odom_count": node.odom_count,
                   "target_count": node.target_count}
        (args.output / "observation_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary))
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
