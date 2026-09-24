#!/usr/bin/env python3
"""Experiment-only static fixture map publisher and read-only tracker recorder."""
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from probe import static_map


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StaticFixtureMap(Node):
    def __init__(self):
        super().__init__('prediction_v1_fixture_static_map')
        occupancy, _ = static_map()
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.width = occupancy.width
        msg.info.height = occupancy.height
        msg.info.resolution = occupancy.resolution
        msg.info.origin.position.x = occupancy.origin_x
        msg.info.origin.position.y = occupancy.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.data
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, '/prediction_v1/static_map', qos)
        self.message = msg
        self.timer = self.create_timer(2.0, self.publish_map)
        self.publish_map()

    def publish_map(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


class PredictionRecorder(Node):
    def __init__(self, directory):
        super().__init__('prediction_v1_shadow_recorder')
        self.predictions = (directory / 'predictions.jsonl').open('x', buffering=1)
        self.diagnostics = (directory / 'diagnostics.jsonl').open('x', buffering=1)
        self.create_subscription(DynamicObstaclePredictionArray,
            '/perception/dynamic_obstacles_shadow/predictions', self.on_prediction, 10)
        self.create_subscription(DiagnosticArray,
            '/perception/dynamic_obstacles_shadow/diagnostics', self.on_diagnostic, 10)

    def on_prediction(self, msg):
        tracks = []
        for track in msg.tracks:
            tracks.append({'id': track.track_id, 'state': track.state,
                'xy': [track.position.x, track.position.y],
                'vxy': [track.velocity.x, track.velocity.y],
                'size_xy': [track.size.x, track.size.y],
                'last_observation_t': seconds(track.last_observation_stamp),
                'future_xy': [[point.x, point.y] for point in track.prediction]})
        row = {'source_t': seconds(msg.header.stamp),
            'processing_t': seconds(msg.processing_stamp),
            'receive_ros_t': seconds(self.get_clock().now().to_msg()),
            'receive_wall_ns': time.time_ns(), 'frame': msg.header.frame_id,
            'schema': msg.schema, 'complete': msg.complete,
            'total_track_count': msg.total_track_count,
            'prediction_dt': msg.prediction_dt,
            'prediction_steps': msg.prediction_steps, 'tracks': tracks}
        self.predictions.write(json.dumps(row, allow_nan=False) + '\n')

    def on_diagnostic(self, msg):
        self.diagnostics.write(json.dumps({'source_t': seconds(msg.header.stamp),
            'statuses': [{'name': s.name, 'level': int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level), 'message': s.message,
                'values': {v.key: v.value for v in s.values}} for s in msg.status]},
            allow_nan=False) + '\n')

    def destroy_node(self):
        self.predictions.close()
        self.diagnostics.close()
        super().destroy_node()


def main():
    mode = sys.argv[1]
    rclpy.init(args=None)
    if mode == 'map':
        node = StaticFixtureMap()
    elif mode == 'record':
        node = PredictionRecorder(Path(sys.argv[2]))
    else:
        raise SystemExit('usage: live_nodes.py map | record DIRECTORY')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
