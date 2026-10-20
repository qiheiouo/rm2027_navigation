"""Verify invalid IMU initialization publishes no odometry, then recovers.

Complete production bundle executable, real ROS messages and DDS. Synthetic
software input only: no physical sensor, canonical TF, Nav2 or serial output.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp(value, seconds):
    nanoseconds = round(seconds * 1e9)
    value.sec = nanoseconds // 1_000_000_000
    value.nanosec = nanoseconds % 1_000_000_000


def cloud(seconds):
    message = CustomMsg()
    stamp(message.header.stamp, seconds)
    message.header.frame_id = "livox_frame"
    message.timebase = round(seconds * 1e9)
    for index in range(900):
        point = CustomPoint()
        point.x = 5.0
        point.y = -3.0 + (index % 30) * 0.2
        point.z = -3.0 + (index // 30) * 0.2
        point.offset_time = int(index * 99_900_000 / 899)
        point.reflectivity = 50
        point.line = index % 4
        point.tag = 0
        message.points.append(point)
    message.point_num = len(message.points)
    return message


class Probe(Node):
    def __init__(self):
        super().__init__("aud059_invalid_init_probe")
        self.samples = []
        self.lidar = self.create_publisher(CustomMsg, "/aud059/lidar", 10)
        self.imu = self.create_publisher(Imu, "/aud059/imu", 100)
        self.subscription = self.create_subscription(
            Odometry, "/aud059/odom", self.samples.append, 10
        )

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic()))
            )


def publish_frame(node, seconds, invalid_mode=None):
    node.lidar.publish(cloud(seconds))
    node.spin_for(0.035)
    for index in range(22):
        message = Imu()
        stamp(message.header.stamp, seconds + index * 0.005)
        message.header.frame_id = "livox_imu"
        if invalid_mode == "zero_acc":
            pass
        elif invalid_mode == "nan_acc":
            message.linear_acceleration.z = math.nan
        elif invalid_mode == "inf_gyro":
            message.linear_acceleration.z = 9.81
            message.angular_velocity.x = math.inf
        else:
            message.linear_acceleration.z = 9.81
        node.imu.publish(message)
        node.spin_for(0.003)
    node.spin_for(0.22)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Probe()
    with (output / "stdout.log").open("w") as stdout, (
        output / "stderr.log"
    ).open("w") as stderr:
        child = subprocess.Popen(
            [
                args.binary,
                "--ros-args",
                "--params-file",
                "/workspace/src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml",
                "-p",
                "common.lid_topic:=/aud059/lidar",
                "-p",
                "common.imu_topic:=/aud059/imu",
                "-r",
                "/Odometry:=/aud059/odom",
                "-r",
                "/tf:=/aud059/backend_tf",
            ],
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 5.0
            while (
                time.monotonic() < deadline
                and child.poll() is None
                and (
                    node.lidar.get_subscription_count() < 1
                    or node.imu.get_subscription_count() < 1
                )
            ):
                node.spin_for(0.03)
            assert child.poll() is None
            assert node.lidar.get_subscription_count() >= 1
            assert node.imu.get_subscription_count() >= 1
            node.spin_for(0.5)

            invalid_modes = ["zero_acc", "nan_acc", "inf_gyro"]
            for index, mode in enumerate(invalid_modes):
                publish_frame(node, 1000.1 + index * 0.11, mode)
            node.spin_for(1.0)
            invalid_odom_count = len(node.samples)
            assert child.poll() is None
            assert invalid_odom_count == 0

            for index in range(8):
                publish_frame(node, 1000.43 + index * 0.11)
            deadline = time.monotonic() + 10.0
            while (
                time.monotonic() < deadline
                and child.poll() is None
                and len(node.samples) < 5
            ):
                node.spin_for(0.05)
            assert child.poll() is None
            assert len(node.samples) >= 5
            for sample in node.samples:
                pose = sample.pose.pose
                values = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
                assert all(math.isfinite(value) for value in values)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                    raise
            node.destroy_node()
            rclpy.shutdown()

    assert child.returncode == 0
    initializations = (output / "stdout.log").read_text().count("IMU Initial Done")
    assert initializations == 1
    result = {
        "scope": __doc__,
        "invalid_modes": invalid_modes,
        "invalid_odom_count": invalid_odom_count,
        "recovery_odom_count": len(node.samples),
        "initializations": initializations,
        "final_exit": child.returncode,
    }
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
