#!/usr/bin/env python3
"""Isolated direct-command stop probe for the Phase 1.5 Gazebo chassis."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


SEQUENCE = (
    ("settle", 1.0, (0.0, 0.0, 0.0)),
    ("lateral_drive", 2.0, (0.0, 0.4, 0.0)),
    ("lateral_stop", 2.0, (0.0, 0.0, 0.0)),
    ("longitudinal_drive", 2.0, (-0.4, 0.0, 0.0)),
    ("longitudinal_stop", 2.0, (0.0, 0.0, 0.0)),
    ("yaw_drive", 2.0, (0.0, 0.0, 0.5)),
    ("yaw_stop", 2.0, (0.0, 0.0, 0.0)),
)


class Probe(Node):
    def __init__(self, path):
        super().__init__("tdt_direct_stop_probe")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.rows = path.open("x", buffering=1)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/simulation/ground_truth/odom", self.odom, 50)
        self.create_subscription(Twist, "/simulation/chassis/cmd_vel", self.relay, 50)
        self.latest_odom = None
        self.latest_relay = None
        self.phase = 0
        self.phase_start = None
        self.last_publish_sim = -math.inf

    def close(self):
        self.rows.close()

    def write(self, **row):
        self.rows.write(json.dumps(row, allow_nan=False) + "\n")

    def stamp(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.twist
        self.latest_odom = t
        self.write(kind="odom", stamp_s=t, callback_sim_s=self.stamp(),
                   steady_ns=time.monotonic_ns(), velocity=[v.linear.x, v.linear.y, v.angular.z])

    def relay(self, msg):
        self.latest_relay = self.stamp()
        self.write(kind="relay", callback_sim_s=self.latest_relay,
                   steady_ns=time.monotonic_ns(), velocity=[msg.linear.x, msg.linear.y, msg.angular.z])

    def tick(self):
        t = self.stamp()
        if not math.isfinite(t) or t <= 0 or self.latest_odom is None or t - self.latest_odom > 0.1:
            return False
        if self.phase_start is None:
            self.phase_start = t
            self.write(kind="phase", name=SEQUENCE[0][0], callback_sim_s=t)
        if t - self.phase_start >= SEQUENCE[self.phase][1]:
            self.phase += 1
            if self.phase == len(SEQUENCE):
                self.write(kind="complete", callback_sim_s=t)
                return True
            self.phase_start = t
            self.write(kind="phase", name=SEQUENCE[self.phase][0], callback_sim_s=t)
        if t - self.last_publish_sim >= 0.04:
            cmd = SEQUENCE[self.phase][2]
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd
            self.publisher.publish(msg)
            self.write(kind="sent", name=SEQUENCE[self.phase][0], callback_sim_s=t,
                       steady_ns=time.monotonic_ns(), velocity=cmd)
            self.last_publish_sim = t
        return False


def main():
    rclpy.init()
    probe = Probe(Path(sys.argv[1]))
    deadline = time.monotonic() + 80
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
