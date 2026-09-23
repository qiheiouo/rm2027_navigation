#!/usr/bin/env python3
"""Publish the SDF-bounded sweep to isolated Nav2 obstacle layers."""
import struct

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField

from planner_sweep import sweep_points


class FixtureSweepCloud(Node):
    def __init__(self):
        super().__init__("tdt_fixture_sweep_cloud")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.publisher = self.create_publisher(PointCloud2, "/fixture_sweep/points", 10)
        self.points, source = sweep_points()
        self.payload = b"".join(struct.pack("<fff", *point) for point in self.points)
        self.count = 0
        print(f"fixture_sweep_cloud_source={source}", flush=True)
        self.create_timer(0.1, self.publish_cloud)

    def publish_cloud(self):
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return
        msg = PointCloud2()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.height = 1
        msg.width = len(self.points)
        msg.fields = [PointField(name=axis, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, axis in enumerate(("x", "y", "z"))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.width * msg.point_step
        msg.data = self.payload
        msg.is_dense = True
        self.publisher.publish(msg)
        self.count += 1
        if self.count == 1 or self.count % 50 == 0:
            print(f"cloud_publish_count={self.count} sim_s={now.nanoseconds * 1e-9:.3f}", flush=True)


def main():
    rclpy.init()
    node = FixtureSweepCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
