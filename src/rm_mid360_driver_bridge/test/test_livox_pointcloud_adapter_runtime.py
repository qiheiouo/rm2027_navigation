import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


INPUT_TOPIC = "/test/livox/native"
POINTCLOUD_TOPIC = "/test/livox/pointcloud"
CUSTOM_TOPIC = "/test/livox/custom"


def make_cloud(node):
    message = PointCloud2()
    message.header.stamp.sec = 100
    message.header.stamp.nanosec = 1000
    message.header.frame_id = "livox_frame"
    message.height = 1
    message.width = 2
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
        PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 32
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = bytearray(message.row_step)
    base_time = 100_000_001_000
    struct.pack_into("<ffffBBd", message.data, 0, 1.0, 2.0, 3.0, 12.4, 0x10, 1, base_time)
    struct.pack_into(
        "<ffffBBd", message.data, 32, -1.0, -2.0, -3.0, 255.9, 0x20, 2,
        base_time + 2000,
    )
    return message


class AdapterProbe(Node):
    def __init__(self):
        super().__init__("livox_pointcloud_adapter_test_probe")
        self.publisher = self.create_publisher(
            PointCloud2, INPUT_TOPIC, qos_profile_sensor_data
        )
        self.pointcloud = None
        self.custom = None
        self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            lambda message: setattr(self, "pointcloud", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CustomMsg,
            CUSTOM_TOPIC,
            lambda message: setattr(self, "custom", message),
            qos_profile_sensor_data,
        )


@pytest.fixture
def adapter_process():
    process = subprocess.Popen(
        [
            "ros2", "run", "rm_mid360_driver_bridge", "livox_pointcloud_adapter_node",
            "--ros-args",
            "-p", f"input_topic:={INPUT_TOPIC}",
            "-p", f"pointcloud_output_topic:={POINTCLOUD_TOPIC}",
            "-p", f"custom_output_topic:={CUSTOM_TOPIC}",
            "-p", "output_frame_id:=mid360_left_frame",
            "-p", "publish_custom:=true",
            "-p", "lidar_id:=7",
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


def test_preserves_pointcloud_and_reconstructs_custom_timing(adapter_process):
    rclpy.init()
    probe = AdapterProbe()
    source = make_cloud(probe)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            probe.publisher.publish(source)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if probe.pointcloud is not None and probe.custom is not None:
                break

        if probe.pointcloud is None or probe.custom is None:
            adapter_process.send_signal(signal.SIGINT)
            output, _ = adapter_process.communicate(timeout=5.0)
            pytest.fail(f"adapter produced no complete output:\n{output}")

        assert probe.pointcloud.header.frame_id == "mid360_left_frame"
        assert bytes(probe.pointcloud.data) == bytes(source.data)
        assert probe.custom.header.frame_id == "mid360_left_frame"
        assert probe.custom.timebase == 100_000_001_000
        assert probe.custom.point_num == 2
        assert probe.custom.lidar_id == 7
        assert [point.offset_time for point in probe.custom.points] == [0, 2000]
        assert [point.reflectivity for point in probe.custom.points] == [12, 255]
        assert [point.tag for point in probe.custom.points] == [0x10, 0x20]
        assert [point.line for point in probe.custom.points] == [1, 2]
    finally:
        probe.destroy_node()
        rclpy.shutdown()
