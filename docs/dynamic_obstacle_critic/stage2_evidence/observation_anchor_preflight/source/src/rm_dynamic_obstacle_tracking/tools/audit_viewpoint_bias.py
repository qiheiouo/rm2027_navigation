#!/usr/bin/env python3
"""Offline stationary-box ray labels: isolate viewpoint bias in visible-centroid tracking.

Truth is used only to construct/label this synthetic audit, never as runtime input.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import statistics
import sys
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm_dynamic_obstacle_tracking.core import Point2D, TrackState, MultiObjectTracker, cluster_points


def rotate(point, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def scan(sensor, dimensions, yaw, resolution):
    origin = rotate(sensor, -yaw)
    points = []
    # Full angular scan, exact world transform, no range noise or static-map
    # contamination. This isolates the visible-centroid measurement model.
    count = math.ceil(2 * math.pi / resolution)
    for i in range(count):
        angle = math.atan2(-sensor[1], -sensor[0]) - math.pi + i * resolution
        direction = rotate((math.cos(angle), math.sin(angle)), -yaw)
        enter, leave = 0., math.inf
        for p, v, half in zip(origin, direction, (d / 2 for d in dimensions)):
            if abs(v) < 1e-12:
                if abs(p) > half:
                    leave = -1.; break
                continue
            a, b = (-half - p) / v, (half - p) / v
            enter = max(enter, min(a, b)); leave = min(leave, max(a, b))
        if leave >= enter and .1 <= enter <= 8.:
            points.append(Point2D(sensor[0] + enter * math.cos(angle), sensor[1] + enter * math.sin(angle)))
    return points


def run(cfg, dimensions, yaw, viewpoint, stream):
    settings = cfg['tracker']; prediction = cfg['prediction']
    tracker = MultiObjectTracker(**settings, prediction_steps=prediction['steps'],
        prediction_dt=prediction['dt'], velocity_decay_tau=prediction['velocity_decay_tau'],
        max_prediction_speed=prediction['max_speed'])
    corners = [rotate(p, yaw) for p in [(-dimensions[0]/2, -dimensions[1]/2),
        (dimensions[0]/2, -dimensions[1]/2), (dimensions[0]/2, dimensions[1]/2), (-dimensions[0]/2, dimensions[1]/2)]]
    errors, speeds, extents = [], [], []; confirmed = 0; covered = 0
    for i in range(121):
        # The physical object is stationary. Only the sensor viewpoint changes.
        angle = 0. if viewpoint == 'fixed' else math.radians(-80. + 160. * i / 120.)
        sensor = (-3. * math.cos(angle), -3. * math.sin(angle))
        points = scan(sensor, dimensions, yaw, math.radians(.5))
        detections = cluster_points(points, cfg['cluster_tolerance'], cfg['cluster_min_points'], cfg['cluster_max_extent'])
        update = tracker.update(detections, 10. + i * .05)
        if len(update.tracks) != 1:
            raise RuntimeError(f'Expected one isolated track, got {len(update.tracks)}')
        track = update.tracks[0]; detection = detections[0]
        error = math.hypot(track.position.x, track.position.y)
        speed = math.hypot(track.velocity.x, track.velocity.y)
        radius = max(.36, .5 * math.hypot(track.size_x, track.size_y))
        full_coverage = all(math.hypot(x-track.position.x, y-track.position.y) <= radius for x, y in corners)
        errors.append(error); speeds.append(speed); extents.append((track.size_x, track.size_y))
        confirmed += track.state == TrackState.CONFIRMED; covered += full_coverage
        stream.write(json.dumps({'dimensions': dimensions, 'yaw': yaw, 'viewpoint': viewpoint,
            'stamp': track.timestamp, 'sensor': sensor, 'true_center': [0., 0.], 'true_velocity': [0., 0.],
            'point_count': len(points), 'visible_centroid': [detection.centroid.x, detection.centroid.y],
            'filtered_centroid': [track.position.x, track.position.y], 'estimated_velocity': [track.velocity.x, track.velocity.y],
            'visible_extent': [track.size_x, track.size_y], 'state': track.state.value,
            'legacy_radius': radius, 'full_true_box_covered': full_coverage}) + '\n')
    return {'dimensions': dimensions, 'yaw': yaw, 'viewpoint': viewpoint, 'frames': len(errors),
        'center_error_median': statistics.median(errors), 'center_error_max': max(errors),
        'estimated_stationary_speed_max': max(speeds), 'confirmed_frames': confirmed,
        'full_box_covered_by_legacy_disk': covered,
        'visible_extent_x_range': [min(x for x, y in extents), max(x for x, y in extents)],
        'visible_extent_y_range': [min(y for x, y in extents), max(y for x, y in extents)]}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('output', type=Path)
    parser.add_argument('--config', required=True, type=Path); args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.config, args.output / 'tracker.yaml')
    policy = {'scope': 'offline geometric measurement-model diagnostic; not a Gazebo or deployment acceptance',
        'objects': [[[.4, .6], 0.], [[.8, .4], .3], [[.45, .55], 0.]],
        'viewpoints': ['fixed', 'arc'], 'stationary_object_center': [0., 0.],
        'dt': .05, 'frames': 121, 'sensor_radius': 3., 'scan_resolution_deg': .5,
        'legacy_critic_minimum_radius': .36, 'noise': 'none; exact world transform'}
    (args.output / 'policy.json').write_text(json.dumps(policy, indent=2) + '\n')
    sources = [Path(__file__), Path(__file__).resolve().parents[1] / 'rm_dynamic_obstacle_tracking/core.py']
    (args.output / 'source_identity.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}, indent=2) + '\n')
    cfg = yaml.safe_load(args.config.read_text())['dynamic_obstacle_tracker_shadow']['ros__parameters']
    with (args.output / 'samples.jsonl').open('w') as stream:
        result = [run(cfg, dims, yaw, view, stream) for dims, yaw in policy['objects'] for view in policy['viewpoints']]
    summary = {'scope': policy['scope'], 'cases': result,
        'limits': 'Ideal opaque convex boxes and exact TF. No occlusion, clutter, missed detection or range noise. Radius uses stage-one .36m configuration. Results isolate measurement bias, not full tracker accuracy.'}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
