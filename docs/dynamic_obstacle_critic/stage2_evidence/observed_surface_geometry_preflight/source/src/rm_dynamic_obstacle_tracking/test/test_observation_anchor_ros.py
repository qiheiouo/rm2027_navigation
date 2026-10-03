"""Real ROS DDS scan/map/TF inputs and prediction/marker outputs, no hardware.

Requires the installed interface and Humble ROS runtime; skipped on pure host.
Synthetic scan expectations stay in this test and are never sent as geometry.
"""
import array
import gzip
import json
import math
import struct
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('rm_competition_interfaces.msg')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from diagnostic_msgs.msg import DiagnosticArray
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array
from rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node import DynamicObstacleTrackerNode


def ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


@pytest.mark.parametrize('mode', ['filtered', 'last_observation_cv'])
def test_dds_scan_source_epoch_prediction_markers_and_output_budget(mode, tmp_path):
    args = ['--ros-args', '-p', 'use_sim_time:=true', '-p',
            'tracker.min_hits_to_confirm:=1', '-p', 'prediction.steps:=30',
            '-p', 'prediction.velocity_decay_tau:=0.0', '-p', 'prediction.max_speed:=0.0',
            '-p', 'prediction.max_tracks:=1', '-p', 'prediction.anchor_mode:='+mode]
    rclpy.init(args=args)
    tracker = DynamicObstacleTrackerNode()
    source = Node('observation_anchor_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(tracker); executor.add_node(source)
    received, markers, diagnostics = [], [], []
    wire_scans = []
    readback = {key: parameter.value for key, parameter in tracker.get_parameters_by_prefix('').items()}
    assert readback['prediction.anchor_mode'] == mode
    assert readback['prediction.max_speed'] == readback['prediction.velocity_decay_tau'] == 0.
    assert readback['prediction.steps'] == 30 and readback['prediction.max_tracks'] == 1
    subscriptions = [
        source.create_subscription(Array, tracker._predictions_topic, received.append, 10),
        source.create_subscription(MarkerArray, tracker._markers_topic, markers.append, 10),
        source.create_subscription(DiagnosticArray, tracker._diagnostics_topic, diagnostics.append, 10)]
    clock_pub = source.create_publisher(Clock, '/clock', 10)
    scan_pub = source.create_publisher(LaserScan, tracker._scan_topic, qos_profile_sensor_data)
    map_pub = source.create_publisher(OccupancyGrid, tracker._map_topic,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster = StaticTransformBroadcaster(source)

    def until(predicate):
        deadline = time.monotonic()+3.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), 'DDS fixture timed out'

    def clock_at(stamp_ns):
        message = Clock(clock=Time(nanoseconds=stamp_ns).to_msg())
        until(lambda: clock_pub.get_subscription_count() >= 1)
        clock_pub.publish(message)
        until(lambda: tracker.get_clock().now().nanoseconds == stamp_ns)

    def scan_at(stamp_ns, ranges):
        clock_at(stamp_ns)
        scan = LaserScan()
        scan.header.frame_id = 'observation_fixture_laser'
        scan.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        scan.angle_min = -.04; scan.angle_increment = .04
        scan.angle_max = scan.angle_min+(len(ranges)-1)*scan.angle_increment
        scan.range_min = .1; scan.range_max = 10.
        scan.ranges = ranges
        before = len(received)
        scan_pub.publish(scan)
        wire_scans.append(serialize_message(scan))
        until(lambda: len(received) == before+1 and
              any(ns(m.markers[0].header.stamp) == stamp_ns for m in markers))
        # LaserScan angles are float32 on the wire, even though Python fields
        # hold pre-serialization doubles. Expected geometry uses received bits.
        return deserialize_message(serialize_message(scan), LaserScan), received[-1]

    try:
        until(lambda: scan_pub.get_subscription_count() == 1 and
                      map_pub.get_subscription_count() == 1)
        epoch = 1700000000*10**9+123456789  # deliberately not float-round-trippable
        clock_at(epoch)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'observation_fixture_laser'
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        until(lambda: tracker._tf_buffer.can_transform('map', transform.child_frame_id, Time()))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'; grid.info.resolution = .1
        grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.; grid.data = [0]*10000
        map_pub.publish(grid)
        until(lambda: tracker._map is not None)
        first_scan, first = scan_at(epoch, [2., 2., 2.])
        second_scan, second = scan_at(epoch+100000000, [2.3, 2.3, 2.3])
        expected = sum(float(r)*math.cos(second_scan.angle_min+i*second_scan.angle_increment)
                       for i, r in enumerate(second_scan.ranges))/3.
        assert len(first.tracks) == len(second.tracks) == 1
        assert second.header.stamp == second_scan.header.stamp
        assert second.header.frame_id == 'map' and second.authority == Array.AUTHORITY_SHADOW_ONLY
        assert second.complete and second.total_track_count == 1
        assert second.prediction_steps == 30 and second.prediction_dt == .1
        if mode == 'last_observation_cv':
            assert second.schema == Array.SCHEMA_OBSERVATION_ANCHOR
            assert abs(second.tracks[0].position.x-expected) < 1e-14
            assert ns(second.tracks[0].last_observation_stamp) == epoch+100000000
        else:
            assert second.schema == Array.SCHEMA
            assert abs(second.tracks[0].position.x-expected) > 1e-4
        assert serialize_message(deserialize_message(serialize_message(second), Array)) == serialize_message(second)
        marker_message = next(m for m in markers if ns(m.markers[0].header.stamp) == epoch+100000000)
        box = next(m for m in marker_message.markers if m.ns == 'track_box')
        assert (box.pose.position.x, box.pose.position.y) == (
            second.tracks[0].position.x, second.tracks[0].position.y)
        _, coast = scan_at(epoch+500000000, [math.inf]*3)
        assert coast.tracks[0].state == coast.tracks[0].STATE_COASTING
        if mode == 'last_observation_cv':
            assert coast.tracks[0].last_observation_stamp == second_scan.header.stamp
            elapsed = 400000000/1e9  # integer source difference before float conversion
            assert coast.tracks[0].position.x == second.tracks[0].position.x + coast.tracks[0].velocity.x*elapsed
        for k, point in enumerate(coast.tracks[0].prediction):
            future = (k+1)*.1
            assert point.x == coast.tracks[0].position.x + coast.tracks[0].velocity.x*future
        # Two separated clusters, one omitted by output budget; no clearance claim.
        _, overflow = scan_at(epoch+600000000, [2., 2., 2.]+[math.inf]*20+[3., 3., 3.])
        assert not overflow.complete and overflow.total_track_count >= 2
        assert len(overflow.tracks) == 1
        # Timestamp cache includes omitted live tracks and releases expired IDs.
        if mode == 'last_observation_cv':
            assert len(tracker._observation_stamp_ns) == overflow.total_track_count
        _, expired = scan_at(epoch+1500000000, [math.inf]*3)
        assert expired.complete and expired.total_track_count == 0 and not expired.tracks
        assert not tracker._observation_stamp_ns
        until(lambda: any(any(k.key == 'prediction_schema' and k.value == second.schema
            for k in status.values) for d in diagnostics for status in d.status))
        def fields(value):
            if hasattr(value, 'get_fields_and_field_types'):
                return {key: fields(getattr(value, key)) for key in value.get_fields_and_field_types()}
            if isinstance(value, (list, tuple, array.array)):
                return [fields(item) for item in value]
            if isinstance(value, float) and not math.isfinite(value):
                return str(value)
            return value
        report = dict(schema=1, mode=mode, status='PASS', actual_parameters=readback,
                      arguments=args, received_predictions=[fields(message) for message in received],
                      scope='Actual isolated DDS scan/map/TF publication test; no physical geometry or control acceptance.')
        (tmp_path/'observations.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        for label, records in [('scans', wire_scans),
                               ('predictions', [serialize_message(message) for message in received])]:
            raw = b''.join(struct.pack('<I', len(record))+record for record in records)
            (tmp_path/(label+'.cdr_records.gz')).write_bytes(gzip.compress(raw, mtime=0))
    finally:
        executor.remove_node(tracker); executor.remove_node(source)
        tracker.destroy_node(); source.destroy_node(); executor.shutdown(); rclpy.shutdown()


@pytest.mark.parametrize('parameters', [
    ['prediction.anchor_mode:=unknown'],
    ['prediction.anchor_mode:=last_observation_cv'],
    ['prediction.anchor_mode:=last_observation_cv', 'prediction.velocity_decay_tau:=0.0']])
def test_invalid_ros_anchor_configuration_refuses_start(parameters):
    args = ['--ros-args']
    for parameter in parameters:
        args += ['-p', parameter]
    rclpy.init(args=args)
    try:
        with pytest.raises(ValueError):
            DynamicObstacleTrackerNode()
    finally:
        rclpy.shutdown()
