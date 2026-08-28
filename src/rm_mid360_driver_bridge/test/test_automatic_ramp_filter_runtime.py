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


INPUT_TOPIC = "/test/automatic_ramp/input"
OUTPUT_TOPIC = "/test/automatic_ramp/output"


def make_points(slope_deg=None, obstacle=False):
    points = []
    if slope_deg is None:
        for x_index in range(16):
            for y_index in range(-6, 7):
                points.append((0.10 * x_index, 0.10 * y_index, 0.0))
        return points

    gradient = math.tan(math.radians(slope_deg))
    for x_index in range(16):
        for y_index in range(-6, 7):
            x = 0.10 * x_index
            y = 0.10 * y_index
            noise = 0.002 * ((x_index + y_index + 30) % 3 - 1)
            points.append((x, y, gradient * x + noise))
    for x_index in range(-5, 0):
        for y_index in range(-6, 7):
            points.append((0.10 * x_index, 0.10 * y_index, 0.0))
    if obstacle:
        points.append((0.80, 0.0, gradient * 0.80 + 0.15))
    return points


def make_cloud(node, points):
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "base_link"
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = list(b"".join(struct.pack("<fff", *point) for point in points))
    message.is_dense = True
    return message


def read_points(message):
    storage = bytes(message.data)
    return [
        struct.unpack_from("<fff", storage, index * message.point_step)
        for index in range(message.width * message.height)
    ]


class RampProbe(Node):
    def __init__(self):
        super().__init__("automatic_ramp_filter_runtime_test")
        self.publisher = self.create_publisher(
            PointCloud2, INPUT_TOPIC, qos_profile_sensor_data
        )
        self.outputs = []
        self.create_subscription(
            PointCloud2,
            OUTPUT_TOPIC,
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


def publish_once(probe, points):
    probe.outputs.clear()
    probe.publisher.publish(make_cloud(probe, points))
    assert spin_until(probe, lambda: bool(probe.outputs)), "filter did not publish"
    return probe.outputs[-1]


@pytest.fixture(scope="module")
def automatic_filter_process():
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "rm_mid360_driver_bridge",
            "automatic_ramp_filter_node",
            "--ros-args",
            "-p",
            f"input_topic:={INPUT_TOPIC}",
            "-p",
            f"output_topic:={OUTPUT_TOPIC}",
            "-p",
            "detection_frame:=base_link",
            "-p",
            "base_frame:=base_link",
            "-p",
            "tracking.confirmation_frames:=3",
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


def test_flat_passthrough_then_multiframe_ramp_filter(automatic_filter_process):
    rclpy.init()
    probe = RampProbe()
    try:
        ready = spin_until(probe, lambda: probe.publisher.get_subscription_count() > 0)
        if not ready:
            automatic_filter_process.send_signal(signal.SIGINT)
            output, _ = automatic_filter_process.communicate(timeout=5.0)
            pytest.fail(f"automatic ramp filter did not subscribe:\n{output}")

        flat = make_points()
        flat_output = publish_once(probe, flat)
        assert flat_output.width == len(flat)

        ramp = make_points(11.0, obstacle=True)
        first = publish_once(probe, ramp)
        second = publish_once(probe, ramp)
        third = publish_once(probe, ramp)
        assert first.width == len(ramp)
        assert second.width == len(ramp)
        assert third.width < len(ramp)

        obstacle_z = math.tan(math.radians(11.0)) * 0.80 + 0.15
        assert any(
            math.isclose(x, 0.80, abs_tol=1.0e-4)
            and math.isclose(y, 0.0, abs_tol=1.0e-4)
            and math.isclose(z, obstacle_z, abs_tol=1.0e-4)
            for x, y, z in read_points(third)
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
