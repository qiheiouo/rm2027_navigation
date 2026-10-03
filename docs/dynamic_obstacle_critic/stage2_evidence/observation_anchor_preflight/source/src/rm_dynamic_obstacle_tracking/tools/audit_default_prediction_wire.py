#!/usr/bin/env python3
"""Compare actual v1 serialized publications to a frozen original wrapper.

No mocks of message layouts: both use generated Humble message classes and
the actual ROS serializer. A fixed clock and collecting publisher make the
comparison deterministic; actual DDS integration has its own test.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import struct
import sys
from types import SimpleNamespace

from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array
from rm_dynamic_obstacle_tracking.core import MultiObjectTracker, Detection, Point2D
from rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node import DynamicObstacleTrackerNode

REFERENCE_SHA = 'eb9510b385fe7f1662f0ac5cc4c595e3ff23b868e947231d265632b9be7309c9'


def canonical_cdr(message, payload):
    """Verify every semantic byte; zero only structural CDR alignment padding.

    Humble's serializer may leave unspecified padding bytes uninitialized.
    This explicit parser refuses extra bytes, different fields or encoding.
    """
    if payload[:4] != b'\x00\x01\x00\x00':
        raise ValueError('pinned little-endian CDR encapsulation required')
    position, padding = 4, []
    canonical = bytearray(payload)
    def number(fmt, value):
        nonlocal position
        size = struct.calcsize(fmt)
        aligned = position+(-(position-4) % size)
        padding.extend(range(position, aligned))
        canonical[position:aligned] = b'\x00'*(aligned-position)
        expected = struct.pack('<'+fmt, value)
        if payload[aligned:aligned+size] != expected:
            raise ValueError(f'CDR semantic value mismatch at {aligned}')
        position = aligned+size
    def stamp(value):
        number('i', value.sec); number('I', value.nanosec)
    def string(value):
        nonlocal position
        encoded = value.encode()+b'\x00'
        number('I', len(encoded))
        if payload[position:position+len(encoded)] != encoded:
            raise ValueError('CDR string mismatch')
        position += len(encoded)
    def vector(value):
        for axis in ('x', 'y', 'z'): number('d', getattr(value, axis))
    stamp(message.header.stamp); string(message.header.frame_id)
    string(message.schema); string(message.authority); stamp(message.processing_stamp)
    number('d', message.prediction_dt); number('I', message.prediction_steps)
    number('?', message.complete); number('I', message.total_track_count)
    number('I', len(message.tracks))
    for track in message.tracks:
        number('Q', track.track_id); number('B', track.state)
        vector(track.position); vector(track.velocity); vector(track.size)
        stamp(track.last_observation_stamp); number('I', track.observation_count)
        number('I', track.miss_count); number('I', len(track.prediction))
        for point in track.prediction: vector(point)
    if position != len(payload): raise ValueError('CDR trailing bytes')
    return bytes(canonical), padding


def run(reference_path, trace_directory):
    if hashlib.sha256(reference_path.read_bytes()).hexdigest() != REFERENCE_SHA:
        raise ValueError('frozen 41e96399 wrapper identity mismatch')
    name = 'rm_dynamic_obstacle_tracking.frozen_node_reference'
    spec = importlib.util.spec_from_file_location(name, reference_path)
    reference = importlib.util.module_from_spec(spec); sys.modules[name] = reference
    spec.loader.exec_module(reference)
    publications = []
    clock = SimpleNamespace(now=lambda: Time(nanoseconds=1700000000323456789))
    wrapper = SimpleNamespace(_map_frame='map', _prediction_dt=.1, _prediction_steps=30,
        _prediction_max_tracks=2, _prediction_schema=Array.SCHEMA, _anchor_mode='filtered',
        _observation_stamp_ns={}, get_clock=lambda: clock,
        _predictions_pub=SimpleNamespace(publish=publications.append))
    tracker = MultiObjectTracker(prediction_steps=30, min_hits_to_confirm=2)
    rng = random.Random(20261003); digest = hashlib.sha256(); truncations = coasting = 0
    raw_traces = [bytearray(), bytearray()]; raw_equal = 0; padding_count = 0
    first_padding_difference = None
    for frame in range(200):
        stamp_ns = 1700000000123456789+frame*100000000
        detections = [Detection(Point2D(index*1.5+frame*.01, rng.uniform(-.03,.03)), .3, .2, 8)
                      for index in range(4) if rng.random() > .25]
        scan = LaserScan(); scan.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        scan.header.frame_id = 'fixture_laser'
        tracks = list(tracker.update(detections, stamp_ns/1e9).tracks)
        DynamicObstacleTrackerNode._publish_predictions(wrapper, scan, tracks)
        reference.DynamicObstacleTrackerNode._publish_predictions(wrapper, scan, tracks)
        actual, baseline = publications[-2:]
        actual_bytes, baseline_bytes = serialize_message(actual), serialize_message(baseline)
        assert actual == baseline, f'v1 message fields differ at frame {frame}'
        actual_cdr, actual_padding = canonical_cdr(actual, actual_bytes)
        baseline_cdr, baseline_padding = canonical_cdr(baseline, baseline_bytes)
        assert actual_padding == baseline_padding and actual_cdr == baseline_cdr
        if actual_bytes == baseline_bytes:
            raw_equal += 1
        elif first_padding_difference is None:
            first_padding_difference = dict(frame=frame, differing_offsets=[i for i, (a,b)
                in enumerate(zip(actual_bytes, baseline_bytes)) if a != b],
                structural_padding_offsets=actual_padding)
        for trace, payload in zip(raw_traces, (actual_bytes, baseline_bytes)):
            trace.extend(struct.pack('<I', len(payload))); trace.extend(payload)
        padding_count += len(actual_padding)
        digest.update(actual_cdr)
        truncations += not actual.complete
        coasting += sum(track.state == track.STATE_COASTING for track in actual.tracks)
    trace_directory.mkdir(exist_ok=False)
    for label, trace in zip(('actual', 'baseline'), raw_traces):
        (trace_directory/(label+'.cdr_records.gz')).write_bytes(gzip.compress(bytes(trace), mtime=0))
    return dict(schema=1, status='PASS', reference_commit='41e96399',
        reference_wrapper_sha256=REFERENCE_SHA, canonical_wire_trace_sha256=digest.hexdigest(),
        raw_byte_equal_frames=raw_equal, raw_padding_different_frames=200-raw_equal,
        structural_padding_bytes_examined=padding_count,
        first_padding_difference=first_padding_difference,
        frames=200, truncated_frames=truncations, coasting_tracks=coasting, seed=20261003,
        serialized_fields=list(Array.get_fields_and_field_types()),
        scope='Every semantic CDR byte and field is verified. Only explicit structural alignment padding is canonicalized; raw-byte identity is not guaranteed by this serializer. DDS scan integration is tested separately.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('reference_node', type=Path)
    parser.add_argument('new_report', type=Path); args = parser.parse_args()
    if args.new_report.exists(): raise FileExistsError(args.new_report)
    report = run(args.reference_node, args.new_report.with_suffix('.raw'))
    args.new_report.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report, sort_keys=True))
