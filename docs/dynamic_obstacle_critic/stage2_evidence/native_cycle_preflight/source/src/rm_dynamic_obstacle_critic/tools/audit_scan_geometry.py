#!/usr/bin/env python3
"""Offline source-time range labels for the frozen Phase1.5 planar fixture.

Uses canonical odometry and fixed SDF extrinsics for projection, Gazebo truth
only for independent actor-box labels. Never a runtime input or TF publisher.
"""
import argparse
import bisect
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET
import yaml
from analyze_trial import compose, load_truth_rows
from trial_io import rows


def interpolate(times, poses, stamp, max_gap):
    i = bisect.bisect_left(times, stamp)
    if i < len(times) and times[i] == stamp:
        return poses[i]
    if i == 0 or i == len(times) or times[i] - times[i - 1] > max_gap:
        return None
    u = (stamp - times[i - 1]) / (times[i] - times[i - 1])
    a, b = poses[i - 1], poses[i]
    return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]),
            a[2] + u * math.remainder(b[2] - a[2], 2 * math.pi))


def ray_box(sensor, angle, box_pose, dimensions):
    dx, dy = sensor[0] - box_pose[0], sensor[1] - box_pose[1]
    c, s = math.cos(box_pose[2]), math.sin(box_pose[2])
    origin = (c * dx + s * dy, -s * dx + c * dy)
    direction = (math.cos(angle - box_pose[2]), math.sin(angle - box_pose[2]))
    enter, leave = 0., math.inf
    for p, v, length in zip(origin, direction, dimensions):
        half = length / 2
        if abs(v) < 1e-12:
            if abs(p) > half:
                return None
            continue
        a, b = (-half - p) / v, (half - p) / v
        enter = max(enter, min(a, b)); leave = min(leave, max(a, b))
    return enter if leave >= enter and leave >= 0 else None


def fixture(scene):
    world = ET.parse(scene / 'phase1_omni.sdf').getroot().find('world')
    robot = world.find("model[@name='rm_sentry_2027']")
    lidar = robot.find("link[@name='sim_lidar_link']")
    sensor_pose = [float(x) for x in lidar.findtext('pose').split()]
    if lidar.find('pose').attrib.get('relative_to') != 'base_link' or any(abs(x) > 1e-12 for x in sensor_pose[3:5]):
        raise ValueError('audit supports only a fixed planar lidar relative to base_link')
    joint = robot.find("joint[@name='base_link_to_sim_lidar_link']")
    if joint.attrib['type'] != 'fixed':
        raise ValueError('audit requires fixed simulation lidar')
    center = world.find("model[@name='center_block']")
    center_pose = [float(x) for x in center.findtext('pose').split()]
    size = [float(x) for x in center.findtext('link/collision/geometry/box/size').split()]
    moving = ET.parse(scene / 'moving_obstacle.sdf').getroot().find('model')
    link = moving.find("link[@name='obstacle_link']")
    moving_size = [float(x) for x in link.findtext('collision/geometry/box/size').split()]
    if any(abs(x) > 1e-12 for x in center_pose[3:5]) or link.find('collision/pose') is not None:
        raise ValueError('unsupported box collision reference')
    return {'sensor_xy_yaw': [sensor_pose[0], sensor_pose[1], sensor_pose[5]],
            'sensor_height': sensor_pose[2], 'static_pose': [center_pose[0], center_pose[1], center_pose[5]],
            'static_dimensions': size[:2], 'actor_dimensions': moving_size[:2],
            'noise_stddev': float(lidar.findtext('sensor/lidar/noise/stddev'))}


def stats(values):
    if not values:
        return {'count': 0}
    return {'count': len(values), 'mean': statistics.mean(values),
            'stddev': statistics.pstdev(values), 'median': statistics.median(values),
            'min': min(values), 'max': max(values)}


def first_box(sensor, angle, actor, geometry, minimum, maximum):
    candidates = []
    for name, box, dims in [('static', geometry['static_pose'], geometry['static_dimensions']),
                             ('actor', actor, geometry['actor_dimensions'])]:
        expected = ray_box(sensor, angle, box, dims)
        if expected is not None and minimum <= expected <= maximum:
            candidates.append((expected, name))
    return min(candidates) if candidates else None


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    if execution['execution'] != 'PASS':
        return {'scope': 'infrastructure_before_goal', 'range_labels': {}}
    scene = root / 'scene_inputs'; geometry = fixture(scene)
    records = list(rows(root, 'observations.jsonl'))
    odom = sorted((r for r in records if r['kind'] == 'canonical_odom'), key=lambda r: r['stamp'])
    times, poses = [], []
    for r in odom:
        x, y, qx, qy, qz, qw = r['pose']
        if r['frame'] != 'odom' or r['child_frame'] != 'base_link' or not all(math.isfinite(v) for v in r['pose']):
            raise ValueError('noncanonical odometry')
        if abs(qx) > 1e-3 or abs(qy) > 1e-3 or abs(qz * qz + qw * qw - 1) > 1e-3:
            raise ValueError('nonplanar odometry')
        if times and r['stamp'] <= times[-1]:
            continue
        times.append(r['stamp']); poses.append((x, y, math.atan2(2 * qw * qz, 1 - 2 * qz * qz)))
    truth = load_truth_rows(root, execution)
    truth_times = [r['t'] for r in truth]; actors = [r['obstacle'] for r in truth]
    true_robots = [r['robot'] for r in truth]
    residuals = defaultdict(list); interior = defaultdict(list)
    cells = defaultdict(set); missing = Counter(); accepted = 0
    delays = []; pose_errors = []; mismatches = 0; finite_no_box = 0
    map_cfg = json.loads((root / 'policy.json').read_text())
    # Label projection cells only for this fixed axis-aligned world. These are
    # scan endpoint bins, never claimed to be the costmap's consumed marking.
    profile = yaml.safe_load((root / 'profile.yaml').read_text())
    resolution = profile['local_costmap']['local_costmap']['ros__parameters']['resolution']
    for scan in records:
        if scan['kind'] != 'scan' or not execution['start_sim'] <= scan['stamp'] <= execution['last_sim']:
            continue
        if scan['frame'] != 'sim_lidar_link' or scan['time_increment'] != 0:
            missing['unsupported_scan_frame_or_motion_timing'] += 1; continue
        sensor_base = interpolate(times, poses, scan['stamp'], .04)
        actor = interpolate(truth_times, actors, scan['stamp'], .04)
        true_robot = interpolate(truth_times, true_robots, scan['stamp'], .04)
        if sensor_base is None or actor is None or true_robot is None:
            missing['missing_source_time_pose_bracket'] += 1; continue
        error = math.hypot(sensor_base[0] - true_robot[0], sensor_base[1] - true_robot[1])
        angular = abs(math.remainder(sensor_base[2] - true_robot[2], 2 * math.pi))
        if error > 1e-4 or angular > 1e-4:
            missing['world_odom_identity_mismatch'] += 1; continue
        pose_errors.append(error); accepted += 1; delays.append(scan['receive_sim'] - scan['stamp'])
        sensor = compose(sensor_base, geometry['sensor_xy_yaw'])
        for index, measured in enumerate(scan['ranges']):
            angle = sensor[2] + scan['angle_min'] + index * scan['angle_increment']
            hit = first_box(sensor, angle, actor, geometry, scan['range_min'], scan['range_max'])
            valid = isinstance(measured, (int, float)) and scan['range_min'] <= measured <= scan['range_max']
            if hit is None:
                finite_no_box += valid; continue
            expected, name = hit
            if not valid:
                mismatches += 1; continue
            residuals[name].append(measured - expected)
            # A geometry-only subset avoids labelling silhouettes/occlusion
            # boundaries as pure range noise. Keep all-ray failures above too.
            neighbors = [first_box(sensor, angle + sign * scan['angle_increment'],
                                   actor, geometry, scan['range_min'], scan['range_max'])
                         for sign in (-1, 1)]
            if all(n is not None and n[1] == name for n in neighbors):
                interior[name].append(measured - expected)
            if name == 'static':
                point = (sensor[0] + measured * math.cos(angle), sensor[1] + measured * math.sin(angle))
                # Distinct world endpoint bins per source scan. Pose/time and
                # noise are explicit; no arbitrary latest orientation lookup.
                cells[scan['stamp']].add((math.floor(point[0] / resolution), math.floor(point[1] / resolution)))
    return {'scope': 'offline source-time fixed planar fixture ray/box range labels; no exact costmap scan-consumer attribution',
            'fixture': geometry, 'scene_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(scene.iterdir())},
            'policy_raw_threshold': map_cfg['raw_costmap_threshold'], 'accepted_source_time_scans': accepted,
            'skipped_scans': dict(missing), 'receipt_delay_s': stats(delays), 'world_odom_xy_error_m': stats(pose_errors),
            'range_labels': {k: {**stats(v), 'abs_residual_gt_3sigma': sum(abs(x) > 3 * geometry['noise_stddev'] for x in v)}
                             for k, v in residuals.items()},
            'angular_interior_range_labels': {k: {**stats(v), 'abs_residual_gt_3sigma': sum(abs(x) > 3 * geometry['noise_stddev'] for x in v)}
                                             for k, v in interior.items()},
            'missing_finite_returns_on_expected_box_rays': mismatches,
            'finite_returns_without_expected_box': finite_no_box,
            'static_endpoint_bin_count_per_scan': stats([len(v) for v in cells.values()]),
            'limits': 'Source-time canonical odometry interpolated with <=0.04s brackets; fixed SDF extrinsics, instantaneous scan only. Box labels use independent physical truth. Angular-interior subset requires the same nearest box at both neighboring ray angles, without selecting on measured residual. All-ray mismatches remain reported. Endpoint bins are not exact costmap marking identities. No formal Gaussian bound, full sensor accuracy or hardware acceptance.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); args = parser.parse_args()
    result = analyze(args.trial)
    (args.trial / 'scan_geometry_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
