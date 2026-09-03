import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


INPUT_TOPIC = "/test/livox/custom_native"
CUSTOM_TOPIC = "/test/livox/custom_canonical"
POINTCLOUD_TOPIC = "/test/livox/from_custom"


def make_custom():
    message = CustomMsg()
    message.header.stamp.sec = 100
    message.header.stamp.nanosec = 1000
    message.header.frame_id = "livox_frame"
    message.timebase = 100_000_001_000
    message.lidar_id = 7
    message.points = [
        CustomPoint(
            offset_time=0,
            x=1.0,
            y=2.0,
            z=3.0,
            reflectivity=12,
            tag=0x10,
            line=1,
        ),
        CustomPoint(
            offset_time=2000,
            x=-1.0,
            y=-2.0,
            z=-3.0,
            reflectivity=255,
            tag=0x20,
            line=2,
        ),
    ]
    message.point_num = len(message.points)
    return message


class AdapterProbe(Node):
    def __init__(self):
        super().__init__("livox_custom_adapter_test_probe")
        self.publisher = self.create_publisher(
            CustomMsg, INPUT_TOPIC, qos_profile_sensor_data
        )
        self.custom = None
        self.pointcloud = None
        self.create_subscription(
            CustomMsg,
            CUSTOM_TOPIC,
            lambda message: setattr(self, "custom", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            lambda message: setattr(self, "pointcloud", message),
            qos_profile_sensor_data,
        )


@pytest.fixture
def adapter_process():
    process = subprocess.Popen(
        [
            "ros2", "run", "rm_mid360_driver_bridge", "livox_custom_adapter_node",
            "--ros-args",
            "-p", f"input_topic:={INPUT_TOPIC}",
            "-p", f"custom_output_topic:={CUSTOM_TOPIC}",
            "-p", f"pointcloud_output_topic:={POINTCLOUD_TOPIC}",
            "-p", "output_frame_id:=mid360_left_frame",
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
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5.0)


def test_preserves_native_custom_timing_and_builds_pointcloud(adapter_process):
    rclpy.init()
    probe = AdapterProbe()
    source = make_custom()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            probe.publisher.publish(source)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if probe.custom is not None and probe.pointcloud is not None:
                break

        if probe.custom is None or probe.pointcloud is None:
            adapter_process.send_signal(signal.SIGINT)
            output, _ = adapter_process.communicate(timeout=5.0)
            pytest.fail(f"adapter produced no complete output:\n{output}")

        assert probe.custom.header.frame_id == "mid360_left_frame"
        assert probe.custom.timebase == source.timebase
        assert probe.custom.point_num == source.point_num
        assert [point.offset_time for point in probe.custom.points] == [0, 2000]

        cloud = probe.pointcloud
        assert cloud.header.frame_id == "mid360_left_frame"
        assert cloud.header.stamp == source.header.stamp
        assert cloud.width == 2
        assert cloud.point_step == 26
        assert [field.name for field in cloud.fields] == [
            "x", "y", "z", "intensity", "tag", "line", "timestamp"
        ]
        first = struct.unpack_from("<ffffBBd", cloud.data, 0)
        second = struct.unpack_from("<ffffBBd", cloud.data, 26)
        assert first == pytest.approx((1.0, 2.0, 3.0, 12.0, 0x10, 1, source.timebase))
        assert second == pytest.approx(
            (-1.0, -2.0, -3.0, 255.0, 0x20, 2, source.timebase + 2000)
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
