from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class PointCloudSamplerNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_pointcloud_sampler")
        input_topic = self.declare_parameter("input_topic", "/points/obstacles").value
        output_topic = self.declare_parameter(
            "output_topic", "/mapping/sensor_cloud"
        ).value
        output_rate = float(self.declare_parameter("output_rate", 5.0).value)
        status_topic = self.declare_parameter(
            "recording_status_topic", "/mapping/recording"
        ).value
        if output_rate <= 0.0:
            raise ValueError("output_rate must be positive")

        self._lock = threading.Lock()
        self._latest: PointCloud2 | None = None
        self._received_sequence = 0
        self._published_sequence = 0
        self._enabled = False
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, status_topic, self._status_callback, status_qos)
        self.create_timer(1.0 / output_rate, self._publish_latest)
        self.get_logger().info(
            "mapping cloud sampler: %s -> %s at <= %.2f Hz"
            % (input_topic, output_topic, output_rate)
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest = msg
            self._received_sequence += 1

    def _status_callback(self, msg: Bool) -> None:
        with self._lock:
            self._enabled = msg.data
            if self._enabled:
                self._published_sequence = self._received_sequence

    def _publish_latest(self) -> None:
        with self._lock:
            msg = self._latest
            if (
                not self._enabled
                or msg is None
                or self._received_sequence == self._published_sequence
            ):
                return
            self._published_sequence = self._received_sequence
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PointCloudSamplerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
