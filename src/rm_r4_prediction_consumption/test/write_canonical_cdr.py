"""Finite canonical-producer wire fixture, isolated from navigation/hardware."""
from pathlib import Path
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rm_r4_interfaces.msg import ObservedPredictionEnvelope
from sensor_msgs.msg import LaserScan
from rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node import DynamicObstacleTrackerNode


def main():
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    topic = '/r4_cdr_check/run_'+uuid.uuid4().hex
    args = ['--ros-args']
    for item in ['observed_members.enabled:=true', 'observed_members.topic:='+topic,
                 'prediction.anchor_mode:=last_observation_cv',
                 'prediction.velocity_decay_tau:=0.0', 'prediction.max_speed:=0.0',
                 'tracker.min_hits_to_confirm:=1', 'tracker.min_displacement_to_confirm:=0.0']:
        args += ['-p', item]
    rclpy.init(args=args)
    tracker = probe = executor = None
    try:
        tracker = DynamicObstacleTrackerNode()
        probe = Node('r4_cdr_probe', use_global_arguments=False)
        messages = []
        probe.create_subscription(ObservedPredictionEnvelope, topic, messages.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(tracker)
        executor.add_node(probe)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*(80*80)
        tracker._on_map(grid)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'r4_cdr_scan'
        tf.transform.rotation.w = 1.
        tracker._tf_buffer.set_transform_static(tf, 'r4_cdr_check')
        source = tracker.get_clock().now().nanoseconds-200_000_000
        for index, (delta, offset, empty) in enumerate([
                (0, 0., False), (50_000_000, .1, False),
                (100_000_000, 0., True), (25_000_000, 0., False)]):
            scan = LaserScan()
            scan.header.frame_id = 'r4_cdr_scan'
            scan.header.stamp = Time(nanoseconds=source+delta).to_msg()
            scan.angle_min, scan.angle_increment = -.1, .04
            scan.range_min, scan.range_max = .05, 3.
            scan.ranges = [float('inf')]*6 if empty else [float('nan'), 1.+offset,
                1.02+offset, float('inf'), 1.03+offset, 4.]
            tracker._on_scan(scan)
            deadline = time.monotonic()+3.
            while len(messages) < index+1 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert len(messages) == index+1 and messages[-1].complete
        for name, item in zip(['observed', 'coasting', 'reset'], messages[1:]):
            assert item.producer_generation == (1 if name == 'reset' else 0)
            (output/(name+'.cdr')).write_bytes(serialize_message(item))
        print('Three actual canonical-producer envelopes serialized; no command output.')
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if tracker is not None:
            tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
