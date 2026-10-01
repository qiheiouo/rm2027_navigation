#!/usr/bin/env python3
"""Fixed synthetic matrix and source-time replay for conditional visible-box fits.

All clusters are fitted before truth labels are consulted. This tool has no ROS,
TF publication or runtime consumer role. Truth only generates/labels experiments.
"""
import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import statistics
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm_dynamic_obstacle_tracking.core import (
    Detection, MultiObjectTracker, Point2D, TrackState, cluster_point_indices,
    cluster_points, dynamic_candidates, OccupancyMap, filter_detections_near_static,
)
from rm_dynamic_obstacle_tracking.visible_box_geometry import BoxFitConfig, fit_visible_box


def fit_config(config):
    settings = dict(config['fit'])
    settings['orthogonality_tolerance'] = math.radians(settings.pop('orthogonality_tolerance_deg'))
    return BoxFitConfig(**settings)


def ray_box(sensor, angle, pose, dimensions):
    """Independent slab intersection for synthetic generation/labels only."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = sensor.x - pose[0], sensor.y - pose[1]
    origin = (c * dx + s * dy, -s * dx + c * dy)
    direction = (math.cos(angle - pose[2]), math.sin(angle - pose[2]))
    enter, leave = 0., math.inf
    for p, v, length in zip(origin, direction, dimensions):
        if abs(v) < 1e-12:
            if abs(p) > length / 2:
                return None
        else:
            a, b = (-length / 2 - p) / v, (length / 2 - p) / v
            enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
    return enter if leave >= enter and enter > 0 else None


def vertices(pose, dimensions):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [(pose[0] + c * x - s * y, pose[1] + s * x + c * y)
            for x, y in ((-dimensions[0] / 2, -dimensions[1] / 2),
                         (dimensions[0] / 2, -dimensions[1] / 2),
                         (dimensions[0] / 2, dimensions[1] / 2),
                         (-dimensions[0] / 2, dimensions[1] / 2))]


def generate_scan(sensor, objects, policy, mode, rng):
    angle_min = math.atan2(-sensor.y, -sensor.x) - math.pi
    increment = math.radians(policy['angle_increment_deg'])
    ranges, labels = [], []
    for i in range(policy['beam_count']):
        angle = angle_min + i * increment
        hits = [(hit, name) for name, pose, dims in objects
                if (hit := ray_box(sensor, angle, pose, dims)) is not None
                and policy['range_min'] <= hit <= policy['range_max']]
        hit, label = min(hits) if hits else (policy['finite_background'], 'background')
        if label != 'background' and mode == 'bounded_noise':
            hit += rng.uniform(-policy['bounded_noise'], policy['bounded_noise'])
        if label != 'background' and mode == 'gaussian_noise':
            hit += rng.gauss(0., policy['gaussian_stddev'])
        if label == 'background' and mode == 'unknown_background':
            hit = math.inf
        ranges.append(hit); labels.append(label)
    target = [i for i, label in enumerate(labels) if label == 'target']
    if target and mode == 'internal_dropout':
        ranges[target[len(target) // 2]] = math.nan
    if target and mode == 'boundary_dropout':
        ranges[target[0]] = math.nan
        if target[0] > 0:
            ranges[target[0] - 1] = math.nan
    return angle_min, increment, ranges, labels


def clusters_and_fits(sensor, angle_min, increment, ranges, minimum, maximum, cfg, fit_cfg,
                      occupancy=None):
    indexed = [(i, Point2D(sensor.x + r * math.cos(angle_min + i * increment),
                          sensor.y + r * math.sin(angle_min + i * increment)))
               for i, r in enumerate(ranges) if math.isfinite(r) and minimum <= r <= maximum]
    if occupancy is not None:
        # Preserve exact endpoint objects and their original indices through the
        # same map subtraction used by the runtime tracker. Never select by truth.
        candidates = dynamic_candidates([p for _, p in indexed], occupancy,
            cfg['static_distance_threshold'], cfg['require_known_free'])
        candidate_ids = {id(p) for p in candidates}
        indexed = [(i, p) for i, p in indexed if id(p) in candidate_ids]
    points = [p for _, p in indexed]
    results = []
    for members in cluster_point_indices(points, cfg['cluster_tolerance'],
                                          cfg['cluster_min_points'], cfg['cluster_max_extent']):
        selected = [points[i] for i in members]
        indices = [indexed[i][0] for i in members]
        xs, ys = [p.x for p in selected], [p.y for p in selected]
        detection = Detection(Point2D(sum(xs) / len(xs), sum(ys) / len(ys)),
            max(max(xs) - min(xs), cfg['cluster_tolerance']),
            max(max(ys) - min(ys), cfg['cluster_tolerance']), len(selected))
        if occupancy is not None and not filter_detections_near_static(
                [detection], occupancy, cfg['detection_static_distance_threshold']):
            results.append({'indices': sorted(indices), 'visible_detection': asdict(detection),
                            'fit': {'reason': 'legacy_centroid_static_filter', 'box': None}})
            continue
        fit = fit_visible_box(selected, indices, ranges=ranges, sensor=sensor,
            angle_min=angle_min, angle_increment=increment, range_min=minimum,
            range_max=maximum, config=fit_cfg)
        results.append({'indices': sorted(indices), 'visible_detection': asdict(detection),
                        'fit': asdict(fit)})
    return results


def disk_covers(box, pose, dims):
    return all(math.hypot(x - box['center']['x'], y - box['center']['y'])
               <= box['enclosing_radius'] for x, y in vertices(pose, dims))


def stats(values):
    return ({'count': len(values), 'median': statistics.median(values), 'max': max(values),
             'min': min(values)} if values else {'count': 0})


def json_ready(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'nan' if math.isnan(value) else ('inf' if value > 0 else '-inf')
    if isinstance(value, dict):
        return {k: json_ready(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_ready(v) for v in value]
    return value


def write_sample(stream, sample):
    stream.write(json.dumps(json_ready(sample), allow_nan=False, separators=(',', ':')) + '\n')


def synthetic(config, tracker_cfg, stream):
    policy, fit_cfg = config['synthetic'], fit_config(config)
    rng = random.Random(policy['seed'])
    groups = defaultdict(lambda: {'scans': 0, 'clusters': 0, 'target_clusters': 0,
        'accepted_target': 0, 'covered_target': 0, 'covered_contributors': 0,
        'mixed_accepted': 0, 'center_errors': [], 'refusals': Counter()})
    case_id = 0
    for dimensions in policy['dimensions']:
        for yaw in policy['yaws']:
            for distance in policy['sensor_ranges']:
                for bearing in policy['sensor_bearings_deg']:
                    angle = math.radians(bearing)
                    sensor = Point2D(-distance * math.cos(angle), -distance * math.sin(angle))
                    for mode in policy['modes']:
                        objects = [('target', (0., 0., yaw), dimensions)]
                        if mode == 'foreground_occlusion':
                            objects.append(('occluder', (sensor.x * .45, sensor.y * .45, -.2), (.12, .30)))
                        if mode == 'merged_clutter':
                            objects.append(('clutter', (0., .42, -.3), (.28, .32)))
                        a, inc, ranges, labels = generate_scan(sensor, objects, policy, mode, rng)
                        fits = clusters_and_fits(sensor, a, inc, ranges, policy['range_min'],
                            policy['range_max'], tracker_cfg, fit_cfg)
                        group = groups[mode]; group['scans'] += 1; group['clusters'] += len(fits)
                        # Truth is consulted only AFTER every cluster was fitted.
                        for item in fits:
                            members = Counter(labels[i] for i in item['indices'])
                            item['truth_member_labels'] = dict(members)
                            if not members['target']:
                                continue
                            group['target_clusters'] += 1
                            box = item['fit']['box']
                            if box is None:
                                group['refusals'][item['fit']['reason']] += 1; continue
                            group['accepted_target'] += 1
                            error = math.hypot(box['center']['x'], box['center']['y'])
                            covered = disk_covers(box, (0., 0., yaw), dimensions)
                            contributors = [(pose, dims) for name, pose, dims in objects if members[name]]
                            complete = all(disk_covers(box, p, d) for p, d in contributors)
                            group['center_errors'].append(error)
                            group['covered_target'] += covered; group['covered_contributors'] += complete
                            group['mixed_accepted'] += len(contributors) > 1
                            item['truth_label'] = {'center_error': error, 'target_box_covered': covered,
                                                  'contributor_boxes_covered': complete}
                        write_sample(stream, {'kind': 'matrix', 'case_id': case_id, 'mode': mode,
                            'objects': objects, 'sensor': asdict(sensor), 'angle_min': a,
                            'angle_increment': inc, 'ranges': ranges, 'clusters': fits})
                        case_id += 1
    return {name: {**{k: v for k, v in group.items() if k != 'center_errors'},
                   'center_error_m': stats(group['center_errors'])} for name, group in groups.items()}


def tracker(cfg):
    prediction = cfg['prediction']
    return MultiObjectTracker(**cfg['tracker'], prediction_steps=prediction['steps'],
        prediction_dt=prediction['dt'], velocity_decay_tau=prediction['velocity_decay_tau'],
        max_prediction_speed=prediction['max_speed'])


def arc(config, cfg, stream):
    policy, fit_cfg = config['synthetic'], fit_config(config)
    results = []
    for dims, yaw in zip(policy['dimensions'], policy['yaws']):
        for mode in ('ideal', 'gaussian_noise'):
            rng = random.Random(policy['seed'])
            old, fitted = tracker(cfg), tracker(cfg)
            speeds, old_speeds, errors, old_errors = [], [], [], []
            confirmed = old_confirmed = fits_count = missing_tracks = 0
            for i in range(policy['arc_frames']):
                angle = math.radians(-80. + 160. * i / (policy['arc_frames'] - 1))
                sensor = Point2D(-3. * math.cos(angle), -3. * math.sin(angle))
                a, inc, ranges, _ = generate_scan(sensor, [('target', (0., 0., yaw), dims)], policy, mode, rng)
                allfits = clusters_and_fits(sensor, a, inc, ranges, policy['range_min'],
                    policy['range_max'], cfg, fit_cfg)
                legacy = [Detection(Point2D(**f['visible_detection']['centroid']),
                    f['visible_detection']['size_x'], f['visible_detection']['size_y'],
                    f['visible_detection']['point_count']) for f in allfits]
                boxes = [f['fit']['box'] for f in allfits if f['fit']['box'] is not None]
                detections = [Detection(Point2D(**b['center']),
                    max(c['x'] for c in b['corners']) - min(c['x'] for c in b['corners']),
                    max(c['y'] for c in b['corners']) - min(c['y'] for c in b['corners']),
                    0) for b in boxes]
                stamp = 10. + i * policy['arc_dt']
                old_update, update = old.update(legacy, stamp), fitted.update(detections, stamp)
                if len(old_update.tracks) != 1:
                    raise ValueError('isolated arc legacy tracker did not retain exactly one track')
                track = old_update.tracks[0]
                old_speeds.append(math.hypot(track.velocity.x, track.velocity.y))
                old_errors.append(math.hypot(track.position.x, track.position.y))
                old_confirmed += track.state == TrackState.CONFIRMED
                fits_count += bool(boxes)
                missing_tracks += not update.tracks
                for track in update.tracks:
                    speeds.append(math.hypot(track.velocity.x, track.velocity.y))
                    errors.append(math.hypot(track.position.x, track.position.y))
                    confirmed += track.state == TrackState.CONFIRMED
                write_sample(stream, {'kind': 'arc', 'mode': mode, 'dimensions': dims, 'yaw': yaw,
                    'stamp': stamp, 'sensor': asdict(sensor), 'clusters': allfits,
                    'legacy_tracks': [asdict(t) for t in old_update.tracks],
                    'fitted_tracks': [asdict(t) for t in update.tracks]})
            results.append({'dimensions': dims, 'yaw': yaw, 'mode': mode,
                'frames': policy['arc_frames'], 'fitted_frames': fits_count,
                'no_fitted_track_frames': missing_tracks, 'legacy_confirmed': old_confirmed,
                'fitted_confirmed': confirmed, 'legacy_stationary_speed_mps': stats(old_speeds),
                'fitted_stationary_speed_mps': stats(speeds), 'legacy_center_error_m': stats(old_errors),
                'fitted_center_error_m': stats(errors)})
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--tracker-config', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.config.read_text())
    cfg = yaml.safe_load(args.tracker_config.read_text())['dynamic_obstacle_tracker_shadow']['ros__parameters']
    shutil.copyfile(args.config, args.output / 'geometry.yaml')
    shutil.copyfile(args.tracker_config, args.output / 'tracker.yaml')
    sources = [Path(__file__), Path(__file__).resolve().parents[1] / 'rm_dynamic_obstacle_tracking/core.py',
               Path(__file__).resolve().parents[1] / 'rm_dynamic_obstacle_tracking/visible_box_geometry.py']
    (args.output / 'policy.json').write_text(json.dumps({'scope': 'offline conditional observation model',
        'policy': config, 'all_clusters_fitted_before_truth_labels': True,
        'support_is_certified': False}, indent=2) + '\n')
    (args.output / 'source_identity.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sources}, indent=2) + '\n')
    with (args.output / 'samples.jsonl').open('w') as stream:
        summary = {'matrix': synthetic(config, cfg, stream), 'stationary_arc': arc(config, cfg, stream),
            'limits': 'Conditional rectangle/nearest-return model only. Finite background is synthetic known geometry. Missing returns, clutter and line-angle errors can invalidate full support. Radial allowance is empirical, never a certified bound. Accepted fits do not imply task, raw203, CV horizon or physical safety acceptance.'}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
