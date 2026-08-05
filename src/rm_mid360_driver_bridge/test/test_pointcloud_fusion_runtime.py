import math
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(node, x):
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "base_link"
    message.height = 1
    message.width = 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12
    message.data = list(struct.pack("<fff", x, 1.0, 0.5))
    message.is_dense = True
    return message


def read_x(message):
    storage = bytes(message.data)
    return sorted(
        struct.unpack_from("<f", storage, index * message.point_step)[0]
        for index in range(message.width * message.height)
    )


class FusionProbe(Node):
    def __init__(self):
        super().__init__("pointcloud_fusion_runtime_test")
        self.left = self.create_publisher(
            PointCloud2, "/livox/left/pointcloud", qos_profile_sensor_data
        )
        self.right = self.create_publisher(
            PointCloud2, "/livox/right/pointcloud", qos_profile_sensor_data
        )
        self.outputs = []
        self.create_subscription(
            PointCloud2,
            "/points/obstacles_fused",
            self.outputs.append,
            qos_profile_sensor_data,
        )


def spin_until(probe, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if predicate():
            return True
    return False


def matching_output(probe, expected):
    for output in reversed(probe.outputs):
        values = read_x(output)
        if output.header.frame_id != "base_link" or len(values) != len(expected):
            continue
        if all(
            math.isclose(actual, wanted, abs_tol=1.0e-5)
            for actual, wanted in zip(values, sorted(expected))
        ):
            return True
    return False


def publish_until(probe, publishers_and_x, expected, timeout=2.0):
    probe.outputs.clear()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for publisher, x in publishers_and_x:
            publisher.publish(make_cloud(probe, x))
        rclpy.spin_once(probe, timeout_sec=0.05)
        if matching_output(probe, expected):
            return
        time.sleep(0.02)
    observed = [read_x(message) for message in probe.outputs]
    raise AssertionError(f"expected fused x={expected}, observed={observed}")


@pytest.fixture(scope="module")
def fusion_launch():
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "rm_mid360_driver_bridge",
            "dual_pointcloud_obstacle_fusion.launch.py",
            "enable_fusion:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    yield process
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)


def test_left_right_dual_stale_and_target_frame(fusion_launch):
    rclpy.init()
    probe = FusionProbe()
    try:
        ready = spin_until(
            probe,
            lambda: probe.left.get_subscription_count() > 0
            and probe.right.get_subscription_count() > 0,
        )
        if not ready:
            fusion_launch.send_signal(signal.SIGINT)
            output, _ = fusion_launch.communicate(timeout=5.0)
            pytest.fail(f"fusion launch did not subscribe to both inputs:\n{output}")

        publish_until(probe, [(probe.left, 1.0)], [1.0])

        time.sleep(0.30)
        publish_until(probe, [(probe.right, 2.0)], [2.0])

        publish_until(
            probe,
            [(probe.right, 2.0), (probe.left, 1.5)],
            [1.5, 2.0],
        )

        time.sleep(0.30)
        publish_until(probe, [(probe.left, 3.0)], [3.0])
    finally:
        probe.destroy_node()
        rclpy.shutdown()
