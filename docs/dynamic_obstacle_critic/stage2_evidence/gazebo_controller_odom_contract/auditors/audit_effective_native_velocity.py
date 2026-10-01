#!/usr/bin/env python3
"""Public route and exact native speed value consistency, with bounded epochs.

Matching a value to available odometry is not a direct capture of the native
subscriber's source stamp. Repeated values retain all matching source stamps.
"""
import argparse
import bisect
import json
from pathlib import Path
import struct
import yaml

from native_snapshot_io import read_snapshot, measured_prefix_matches
from trial_io import rows


def double_bits(value):
    return struct.pack('<d', value)


def analyze(root):
    controller = yaml.safe_load((root / 'profile.yaml').read_text())['controller_server']['ros__parameters']
    guard = yaml.safe_load((root / 'installed_inputs/guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
    # Reuse the frozen measurement freshness limit, without fitting it to data.
    age_limit = guard['odom_timeout']
    thresholds = [controller[name] for name in
                  ['min_x_velocity_threshold', 'min_y_velocity_threshold', 'min_theta_velocity_threshold']]
    odometry = sorted((row for row in rows(root, 'observations.jsonl')
                       if row['kind'] == 'canonical_odom'), key=lambda row: row['stamp'])
    stamps = [row['stamp'] for row in odometry]
    readback_path = root / 'controller_odom_readback.json'
    readback = json.loads(readback_path.read_text()) if readback_path.exists() else None
    route = (readback is not None and readback['verdict'] == 'ROUTE PASS'
             and readback['configured_odom_topic'] == '/odometry/lio'
             and controller.get('odom_topic') == '/odometry/lio')
    records = []
    for path in sorted((root / 'native_cycles').glob('cycle_*.json'),
                       key=lambda path: int(path.stem.split('_')[1])):
        meta, blocks = read_snapshot(path)
        pose_stamp = meta['pose_stamp_sec'] + meta['pose_stamp_nanosec'] * 1e-9
        capture = meta['capture_stamp']
        speed = [meta['speed'][index] for index in [0, 1, 5]]
        available = odometry[bisect.bisect_left(stamps, capture - age_limit):
                             bisect.bisect_right(stamps, capture)]
        matches = []
        for row in available:
            if row['frame'] != 'odom' or row['child_frame'] != 'base_link':
                continue
            value = [v if abs(v) > threshold else 0.
                     for v, threshold in zip(row['velocity'], thresholds)]
            if all(double_bits(v) == double_bits(n) for v, n in zip(value, speed)):
                matches.append(row['stamp'])
        source_index = bisect.bisect_right(stamps, pose_stamp) - 1
        source = odometry[source_index] if source_index >= 0 else None
        source_fresh = source is not None and 0 <= pose_stamp - source['stamp'] <= age_limit
        moving = source_fresh and any(abs(v) > threshold
                                     for v, threshold in zip(source['velocity'], thresholds))
        frames = meta['pose_frame'] == 'odom' and meta['evaluation_frame'] == 'odom' and meta['base_frame'] == 'base_link'
        records.append({
            'ordinal': meta['ordinal'], 'source_pose_stamp': pose_stamp,
            'capture_stamp': capture, 'native_speed': meta['speed'],
            'matching_source_stamps': matches,
            'latest_matching_value_age': capture - matches[-1] if matches else None,
            'actual_consumer_source_stamp': None,
            'source_pose_canonical_motion': bool(moving),
            'native_input_nonzero': any(value != 0 for value in speed),
            'native_planar': all(meta['speed'][index] == 0 for index in [2, 3, 4]),
            'actual_measured_prefix_bit_exact': measured_prefix_matches(meta, blocks),
            'source_pose_odom_available': source_fresh,
            'frames_match': frames})
    moving = [row for row in records if row['source_pose_canonical_motion']]
    gate = (route and bool(moving) and bool(records)
            and all(row['matching_source_stamps'] and row['native_planar']
                    and row['actual_measured_prefix_bit_exact'] and row['source_pose_odom_available']
                    and row['frames_match'] for row in records)
            and all(row['native_input_nonzero'] for row in moving))
    ages = [row['latest_matching_value_age'] for row in records if row['matching_source_stamps']]
    return {
        'verdict': 'EFFECTIVE VELOCITY VALUE CONSISTENCY PASS' if gate else 'FAILED',
        'scope': 'configured public route, bounded canonical value provenance, actual native measured prefix; no direct consumer source stamp',
        'route_verified': bool(route), 'captures': len(records),
        'canonical_motion_cycles': len(moving),
        'canonical_motion_cycles_with_nonzero_native': sum(row['native_input_nonzero'] for row in moving),
        'all_native_nonzero_cycles': sum(row['native_input_nonzero'] for row in records),
        'cycles_with_exact_thresholded_canonical_value': sum(bool(row['matching_source_stamps']) for row in records),
        'age_limit_from_frozen_guard': age_limit,
        'max_latest_matching_value_age': max(ages) if ages else None,
        'thresholds': thresholds, 'records': records,
        'limits': 'Installed controller uses nav_2d_utils::OdomSubscriber latest planar twist, with per-component strict thresholding. Exact values in the fixed freshness window establish consistency with recorded physical measurements, not unique source-message identity or callback freshness. Repeated values are ambiguous. Subscriber source stamp is unavailable in the capture. Pose and velocity epochs can differ; full-horizon geometry remains conditional and physical safety/task gates remain separate.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = analyze(args.trial)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'records'}))


if __name__ == '__main__':
    main()
