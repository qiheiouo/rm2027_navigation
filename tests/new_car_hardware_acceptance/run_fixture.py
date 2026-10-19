#!/usr/bin/env python3
"""No-hardware fixture for the read-only acceptance probe."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


class Fixture(Node):
    def __init__(self, regress):
        super().__init__("new_car_acceptance_fixture")
        self.regress = regress
        self.sequence = 0
        self.imu = self.create_publisher(Imu, "/fixture/imu", 10)
        self.odom = self.create_publisher(Odometry, "/fixture/odom", 10)
        self.tf = self.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf_static = self.create_publisher(TFMessage, "/tf_static", static_qos)

    def publish(self):
        stamp = self.get_clock().now()
        if self.regress and self.sequence == 70:
            stamp = rclpy.time.Time(nanoseconds=stamp.nanoseconds - 1_000_000_000)
        stamp_message = stamp.to_msg()
        imu = Imu()
        imu.header.stamp = stamp_message
        imu.header.frame_id = "sensor"
        imu.orientation.w = 1.0
        self.imu.publish(imu)
        if self.sequence % 2 == 0:
            odom = Odometry()
            odom.header.stamp = stamp_message
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self.odom.publish(odom)
        dynamic = TransformStamped()
        dynamic.header.stamp = stamp_message
        dynamic.header.frame_id = "odom"
        dynamic.child_frame_id = "base_link"
        dynamic.transform.rotation.w = 1.0
        self.tf.publish(TFMessage(transforms=[dynamic]))
        static = TransformStamped()
        static.header.stamp = stamp_message
        static.header.frame_id = "base_link"
        static.child_frame_id = "sensor"
        static.transform.rotation.w = 1.0
        self.tf_static.publish(TFMessage(transforms=[static]))
        self.sequence += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--regress", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output or Path(tempfile.mkdtemp(prefix="rm2027_acceptance_fixture_"))
    environment = os.environ.copy()
    python_path = str(root / "src" / "rm_system_monitor")
    environment["PYTHONPATH"] = python_path + os.pathsep + environment.get("PYTHONPATH", "")
    command = [
        sys.executable,
        "-m",
        "rm_system_monitor.hardware_acceptance",
        "--mode",
        "observe",
        "--plan",
        str(root / "tests" / "new_car_hardware_acceptance" / "fixture_plan.yaml"),
        "--workspace",
        str(root),
        "--output",
        str(output),
        "--steps",
        "1",
        "--duration",
        "2.0",
    ]
    rclpy.init()
    fixture = Fixture(args.regress)
    process = subprocess.Popen(command, cwd=root, env=environment)
    try:
        while process.poll() is None:
            fixture.publish()
            rclpy.spin_once(fixture, timeout_sec=0.005)
            time.sleep(0.015)
        result = json.loads((output / "report.json").read_text())
        assert result["safety"] == {"publishers_created": 0, "motion_commands_sent": 0}
        if args.regress:
            assert process.returncode == 2
            assert result["decision"] == "STOP"
            assert any(
                item["kind"] == "topic" and item["metrics"]["timestamp_regressions"] > 0
                for item in result["checks"]
            )
        else:
            assert process.returncode == 0
            assert result["decision"] == "GO"
            assert result["automatic_gate_passed"]
        print(json.dumps({"output": str(output), "decision": result["decision"]}))
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=3)
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
