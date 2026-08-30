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
SECONDARY_INPUT_TOPIC = "/test/automatic_ramp/secondary_input"
SECONDARY_OUTPUT_TOPIC = "/test/automatic_ramp/secondary_output"


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
        self.secondary_publisher = self.create_publisher(
            PointCloud2, SECONDARY_INPUT_TOPIC, qos_profile_sensor_data
        )
        self.outputs = []
        self.secondary_outputs = []
        self.create_subscription(
            PointCloud2,
            OUTPUT_TOPIC,
            self.outputs.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            SECONDARY_OUTPUT_TOPIC,
            self.secondary_outputs.append,
            qos_profile_sensor_data,
        )


def spin_until(probe, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if predicate():
            return True
    return False


def publish_once(probe, points, secondary=False):
    publisher = probe.secondary_publisher if secondary else probe.publisher
    outputs = probe.secondary_outputs if secondary else probe.outputs
    outputs.clear()
    publisher.publish(make_cloud(probe, points))
    assert spin_until(probe, lambda: bool(outputs)), "filter did not publish"
    return outputs[-1]


def publish_until_connected(probe, points, timeout=3.0):
    probe.outputs.clear()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.publisher.publish(make_cloud(probe, points))
        rclpy.spin_once(probe, timeout_sec=0.10)
        if probe.outputs:
            return probe.outputs[-1]
        time.sleep(0.05)
    raise AssertionError("filter output publisher did not match the test subscription")


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
            f"secondary_input_topic:={SECONDARY_INPUT_TOPIC}",
            "-p",
            f"secondary_output_topic:={SECONDARY_OUTPUT_TOPIC}",
            "-p",
            "detection_frame:=base_link",
            "-p",
            "base_frame:=base_link",
            "-p",
            "detection.update_period_sec:=0.0",
            "-p",
            "detection.accumulation_window_sec:=0.0",
            "-p",
            "tracking.confirmation_frames:=3",
            "-p",
            "tracking.pending_max_missed_frames:=1",
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


def test_shared_tracker_bridges_gaps_and_filters_both_streams(
    automatic_filter_process,
):
    rclpy.init()
    probe = RampProbe()
    try:
        ready = spin_until(
            probe,
            lambda: (
                probe.publisher.get_subscription_count() > 0
                and probe.secondary_publisher.get_subscription_count() > 0
            ),
        )
        if not ready:
            automatic_filter_process.send_signal(signal.SIGINT)
            output, _ = automatic_filter_process.communicate(timeout=5.0)
            pytest.fail(f"automatic ramp filter did not subscribe:\n{output}")

        flat = make_points()
        # The input subscription can be visible slightly before the filter's
        # output publisher matches this process after a fresh DDS/container
        # startup. Repeating flat input is safe because it cannot advance a
        # ramp candidate; later ramp frames remain exactly one publish each.
        flat_output = publish_until_connected(probe, flat)
        assert flat_output.width == len(flat)

        ramp = make_points(11.0, obstacle=True)
        first = publish_once(probe, ramp)
        gap_one = publish_once(probe, flat)
        second = publish_once(probe, ramp)
        gap_two = publish_once(probe, flat)
        third = publish_once(probe, ramp)
        assert first.width == len(ramp)
        assert gap_one.width == len(flat)
        assert second.width == len(ramp)
        assert gap_two.width == len(flat)
        assert third.width < len(ramp)

        secondary = publish_once(probe, ramp, secondary=True)
        assert secondary.width < len(ramp)

        obstacle_z = math.tan(math.radians(11.0)) * 0.80 + 0.15
        for message in (third, secondary):
            assert any(
                math.isclose(x, 0.80, abs_tol=1.0e-4)
                and math.isclose(y, 0.0, abs_tol=1.0e-4)
                and math.isclose(z, obstacle_z, abs_tol=1.0e-4)
                for x, y, z in read_points(message)
            )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
