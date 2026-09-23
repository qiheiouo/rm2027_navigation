#!/usr/bin/env python3
"""Simulation-only final-command filter for the known moving joint sweep."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from guard_core import certificate, fixture_sweep


class Guard(Node):
    def __init__(self, profile, output):
        super().__init__("tdt_fixture_sweep_guard")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        params = yaml.safe_load(profile.read_text())
        cfg = params["local_costmap"]["local_costmap"]["ros__parameters"]
        self.body = yaml.safe_load(cfg["footprint"])
        assert len(self.body) == 8 and cfg["footprint_padding"] == 0.03
        self.sweep, source = fixture_sweep()
        self.stream = output.open("x", buffering=1)
        self.stream.write(json.dumps({"kind": "configuration", "fixture": source,
                                      "body": self.body, "padding_m": 0.03,
                                      "input_topic": "/cmd_vel",
                                      "output_topic": "/cmd_vel_guarded"}) + "\n")
        self.odom = None
        self.last_input_sim = None
        self.last_output_zero = True
        self.publisher = self.create_publisher(Twist, "/cmd_vel_guarded", 20)
        self.create_subscription(Odometry, "/simulation/ground_truth/odom", self.odom_cb, 50)
        self.create_subscription(Twist, "/cmd_vel", self.command_cb, 20)
        self.create_timer(0.02, self.watchdog)

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        v = msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            self.odom = None
            return
        self.odom = ((p.x, p.y, yaw), (v.linear.x, v.linear.y, v.angular.z), stamp)

    @staticmethod
    def vector(msg):
        return (msg.linear.x, msg.linear.y, msg.angular.z)

    def emit(self, vector):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vector
        self.publisher.publish(msg)
        self.last_output_zero = vector == (0.0, 0.0, 0.0)

    def command_cb(self, msg):
        now = self.sim_time()
        proposed = self.vector(msg)
        self.last_input_sim = now
        record = {"kind": "decision", "sim_s": now,
                  "steady_ns": time.monotonic_ns(), "proposed": proposed}
        if self.odom is None or now <= 0:
            record["reason"] = "missing_odom"
        else:
            pose, speed, stamp = self.odom
            record.update(pose=pose, odom_speed=speed, odom_stamp_s=stamp,
                          odom_age_s=now - stamp)
            if now - stamp < -0.02 or now - stamp > 0.1:
                record["reason"] = "stale_odom"
            else:
                try:
                    record["certificate"] = certificate(pose, speed, proposed,
                                                        self.body, self.sweep)
                    record["reason"] = ("admit" if record["certificate"]["safe_under_model"]
                                        else "reject_sweep")
                except ValueError as error:
                    record["reason"] = "invalid_input"
                    record["error"] = str(error)
        output = proposed if record["reason"] == "admit" else (0.0, 0.0, 0.0)
        self.emit(output)
        record["emitted"] = output
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")

    def watchdog(self):
        now = self.sim_time()
        if self.last_input_sim is not None and now - self.last_input_sim > 0.12 and not self.last_output_zero:
            self.emit((0.0, 0.0, 0.0))
            self.stream.write(json.dumps({"kind": "watchdog_zero", "sim_s": now}) + "\n")


def main():
    rclpy.init()
    node = Guard(Path(sys.argv[1]), Path(sys.argv[2]))
    try:
        rclpy.spin(node)
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
