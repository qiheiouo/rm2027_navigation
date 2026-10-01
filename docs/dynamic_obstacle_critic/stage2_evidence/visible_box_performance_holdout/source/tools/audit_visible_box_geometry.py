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
    dynamic_candidates, OccupancyMap, filter_detections_near_static,
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


def visible_faces(sensor, pose, dims):
    dx, dy = sensor.x - pose[0], sensor.y - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return int(abs(c * dx + s * dy) > dims[0] / 2) + int(abs(-s * dx + c * dy) > dims[1] / 2)


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
        'mixed_accepted': 0, 'accepted_single_face_truth': 0,
        'center_errors': [], 'refusals': Counter()})
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
                            face_count = visible_faces(sensor, (0., 0., yaw), dimensions)
                            group['accepted_single_face_truth'] += face_count == 1
                            error = math.hypot(box['center']['x'], box['center']['y'])
                            covered = disk_covers(box, (0., 0., yaw), dimensions)
                            contributors = [(pose, dims) for name, pose, dims in objects if members[name]]
                            complete = all(disk_covers(box, p, d) for p, d in contributors)
                            group['center_errors'].append(error)
                            group['covered_target'] += covered; group['covered_contributors'] += complete
                            group['mixed_accepted'] += len(contributors) > 1
                            item['truth_label'] = {'center_error': error, 'target_box_covered': covered,
                                'contributor_boxes_covered': complete, 'visible_faces': face_count}
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
            raw_covered = 0
            filtered_coverage = Counter()
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
                    f['visible_detection']['point_count']) for f in allfits
                    if (b := f['fit']['box']) is not None]
                stamp = 10. + i * policy['arc_dt']
                old_update, update = old.update(legacy, stamp), fitted.update(detections, stamp)
                if len(old_update.tracks) != 1:
                    raise ValueError('isolated arc legacy tracker did not retain exactly one track')
                track = old_update.tracks[0]
                old_speeds.append(math.hypot(track.velocity.x, track.velocity.y))
                old_errors.append(math.hypot(track.position.x, track.position.y))
                old_confirmed += track.state == TrackState.CONFIRMED
                fits_count += bool(boxes)
                raw_covered += bool(boxes) and all(disk_covers(b, (0., 0., yaw), dims) for b in boxes)
                missing_tracks += not update.tracks
                for track in update.tracks:
                    speeds.append(math.hypot(track.velocity.x, track.velocity.y))
                    errors.append(math.hypot(track.position.x, track.position.y))
                    confirmed += track.state == TrackState.CONFIRMED
                    # This hypothetical public-size circle is only an audit
                    # proxy. No fitted radius/validity has entered runtime state.
                    radius = max(.36, .5 * math.hypot(track.size_x, track.size_y)) + fit_cfg.range_error
                    for horizon in (0, 1, 2, 3):
                        cx = track.position.x + horizon * track.velocity.x
                        cy = track.position.y + horizon * track.velocity.y
                        filtered_coverage[str(horizon)] += all(math.hypot(x - cx, y - cy) <= radius
                            for x, y in vertices((0., 0., yaw), dims))
                write_sample(stream, {'kind': 'arc', 'mode': mode, 'dimensions': dims, 'yaw': yaw,
                    'stamp': stamp, 'sensor': asdict(sensor), 'clusters': allfits,
                    'legacy_tracks': [asdict(t) for t in old_update.tracks],
                    'fitted_tracks': [asdict(t) for t in update.tracks]})
            results.append({'dimensions': dims, 'yaw': yaw, 'mode': mode,
                'frames': policy['arc_frames'], 'fitted_frames': fits_count,
                'no_fitted_track_frames': missing_tracks, 'legacy_confirmed': old_confirmed,
                'fitted_confirmed': confirmed, 'legacy_stationary_speed_mps': stats(old_speeds),
                'fitted_stationary_speed_mps': stats(speeds), 'legacy_center_error_m': stats(old_errors),
                'fitted_center_error_m': stats(errors), 'raw_fit_full_box_covered': raw_covered,
                'filtered_proxy_full_box_covered_0_1_2_3s': dict(filtered_coverage),
                'filtered_proxy_track_samples': len(speeds)})
    return results


def load_static_map(path, occupied_threshold):
    """Read the frozen P5 trinary map; preserve image-to-map row inversion."""
    cfg = yaml.safe_load(path.read_text())
    if cfg['mode'] != 'trinary' or cfg['negate'] not in (0, 1):
        raise ValueError('only trinary P5 maps are supported')
    with (path.parent / cfg['image']).open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('expected a binary P5 map')

        def tokens():
            line = stream.readline()
            while line.startswith(b'#'):
                line = stream.readline()
            return line.split()

        width, height = map(int, tokens())
        if list(map(int, tokens())) != [255]:
            raise ValueError('expected 8-bit map')
        pixels = stream.read()
    if len(pixels) != width * height:
        raise ValueError('invalid P5 data length')
    data = []
    for y in range(height - 1, -1, -1):
        for x in range(width):
            pixel = pixels[y * width + x] / 255
            probability = pixel if cfg['negate'] else 1 - pixel
            data.append(100 if probability > cfg['occupied_thresh'] else
                        0 if probability < cfg['free_thresh'] else -1)
    return OccupancyMap(width, height, cfg['resolution'], *cfg['origin'], data,
                        occupied_threshold=occupied_threshold)


def verify_trial(trial):
    manifest = json.loads((trial / 'manifest.json').read_text())
    for name, expected in manifest['files'].items():
        actual = hashlib.sha256((trial / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f'frozen trial manifest mismatch: {name}')
    return {'manifest_sha256': hashlib.sha256((trial / 'manifest.json').read_bytes()).hexdigest(),
            'checked_files': len(manifest['files']), 'source_checkpoint': manifest['source_checkpoint']}


def replay(config, cfg, trial, stream):
    identity = verify_trial(trial)
    frozen_cfg = yaml.safe_load((trial / 'installed_inputs/tracker_cv.yaml').read_text())[
        'dynamic_obstacle_tracker_shadow']['ros__parameters']
    if cfg != frozen_cfg:
        raise ValueError('replay must use exactly the frozen runtime tracker configuration')
    # These frozen utilities only decode source-time input and independent labels.
    # They never enter the tracker package runtime or the geometric fitter.
    sys.path.insert(0, str(trial / 'auditors'))
    from audit_scan_geometry import interpolate, fixture
    from analyze_trial import compose, load_truth_rows
    from trial_io import rows
    utility_files = [trial / 'auditors' / name for name in
                     ('audit_scan_geometry.py', 'analyze_trial.py', 'trial_io.py')]
    execution = json.loads((trial / 'execution.json').read_text())
    if execution['execution'] != 'PASS':
        raise ValueError('replay requires a completed trial capture')
    geometry = fixture(trial / 'scene_inputs')
    times, poses, scans = [], [], []
    odom = []
    for row in rows(trial, 'observations.jsonl'):
        if row['kind'] == 'canonical_odom':
            odom.append(row)
        elif row['kind'] == 'scan' and execution['start_sim'] <= row['stamp'] <= execution['last_sim']:
            scans.append(row)
    for row in sorted(odom, key=lambda r: r['stamp']):
        x, y, qx, qy, qz, qw = row['pose']
        if (row['frame'] != 'odom' or row['child_frame'] != 'base_link'
                or not all(math.isfinite(v) for v in row['pose'])
                or abs(qx) > 1e-3 or abs(qy) > 1e-3
                or abs(qz * qz + qw * qw - 1) > 1e-3):
            raise ValueError('noncanonical or nonplanar odometry')
        if times and row['stamp'] <= times[-1]:
            continue
        times.append(row['stamp']); poses.append((x, y, math.atan2(2 * qw * qz, 1 - 2 * qz * qz)))
    truth = load_truth_rows(trial, execution)
    truth_times = [row['t'] for row in truth]
    actors, robots = [r['obstacle'] for r in truth], [r['robot'] for r in truth]
    occupancy = load_static_map(trial / 'installed_inputs/course_static.yaml', cfg['occupied_threshold'])
    accepted_scans, skips, missing_labels = 0, Counter(), Counter()
    groups = {name: {'clusters': 0, 'accepted': 0, 'actor_clusters': 0,
        'accepted_actor': 0, 'covered_actor': 0, 'center_errors': [],
        'visible_center_errors': [], 'refusals': Counter(), 'actor_refusals': Counter(),
        'accepted_actor_single_face': 0} for name in ('strict_finite_boundary', 'assumed_no_return_clear')}
    base = asdict(fit_config(config))
    for scan in scans:
        source = {'kind': 'replay', 'stamp': scan['stamp'], 'receive_sim': scan['receive_sim']}
        if scan['frame'] != 'sim_lidar_link' or scan['time_increment'] != 0:
            reason = 'unsupported_frame_or_timing'
        elif not -cfg['max_future_scan_sec'] <= scan['receive_sim'] - scan['stamp'] <= cfg['max_scan_age_sec']:
            reason = 'source_age_gate'
        else:
            reason = None
        robot = interpolate(times, poses, scan['stamp'], .04) if reason is None else None
        if robot is None and reason is None:
            reason = 'missing_canonical_source_pose_bracket'
        if reason:
            skips[reason] += 1; write_sample(stream, {**source, 'skipped': reason}); continue
        sensor_pose = compose(robot, geometry['sensor_xy_yaw'])
        sensor = Point2D(sensor_pose[0], sensor_pose[1])
        angle_min = sensor_pose[2] + scan['angle_min']
        ranges = [float(r) for r in scan['ranges']]
        accepted_scans += 1
        fitted_modes = {}
        # Fit ALL input clusters, for BOTH declared policies, before consulting
        # actor truth or deciding whether world/odom alignment permits labels.
        for name in groups:
            settings = {**base, 'assume_no_return_clear': name == 'assumed_no_return_clear'}
            fitted_modes[name] = clusters_and_fits(sensor, angle_min, scan['angle_increment'],
                ranges, scan['range_min'], scan['range_max'], cfg, BoxFitConfig(**settings), occupancy)
        actor = interpolate(truth_times, actors, scan['stamp'], .04)
        true_robot = interpolate(truth_times, robots, scan['stamp'], .04)
        label_missing = ('missing_truth_source_bracket' if actor is None or true_robot is None else
            'world_odom_identity_mismatch' if math.hypot(robot[0] - true_robot[0], robot[1] - true_robot[1]) > 1e-4
            or abs(math.remainder(robot[2] - true_robot[2], 2 * math.pi)) > 1e-4 else None)
        if label_missing:
            missing_labels[label_missing] += 1
        for name, fits in fitted_modes.items():
            group = groups[name]
            for item in fits:
                group['clusters'] += 1
                box = item['fit']['box']
                group['accepted'] += box is not None
                if box is None:
                    group['refusals'][item['fit']['reason']] += 1
                if label_missing:
                    continue
                labels = Counter()
                for i in item['indices']:
                    angle = angle_min + i * scan['angle_increment']
                    possible = [(r, label) for label, pose, dims in
                        [('actor', actor, geometry['actor_dimensions']),
                         ('static', geometry['static_pose'], geometry['static_dimensions'])]
                        if (r := ray_box(sensor, angle, pose, dims)) is not None
                        and scan['range_min'] <= r <= scan['range_max']]
                    # Geometry-only ray labels; do not select on residual/fit error.
                    labels[min(possible)[1] if possible else 'no_expected_box'] += 1
                item['truth_member_labels'] = dict(labels)
                if not labels['actor']:
                    continue
                group['actor_clusters'] += 1
                visible = item['visible_detection']['centroid']
                group['visible_center_errors'].append(math.hypot(visible['x'] - actor[0], visible['y'] - actor[1]))
                if box is None:
                    group['actor_refusals'][item['fit']['reason']] += 1; continue
                error = math.hypot(box['center']['x'] - actor[0], box['center']['y'] - actor[1])
                covered = disk_covers(box, actor, geometry['actor_dimensions'])
                face_count = visible_faces(sensor, actor, geometry['actor_dimensions'])
                group['accepted_actor'] += 1; group['covered_actor'] += covered
                group['accepted_actor_single_face'] += face_count == 1
                group['center_errors'].append(error)
                item['truth_label'] = {'actor_center_error': error, 'actor_box_covered': covered,
                                       'visible_faces': face_count}
        write_sample(stream, {**source, 'sensor_xy_yaw': sensor_pose, 'label_missing': label_missing,
                              'truth_actor': actor, 'modes': fitted_modes})
    return {'trial_identity': identity, 'scans_in_window': len(scans),
        'canonical_source_time_scans': accepted_scans, 'skips': dict(skips),
        'missing_truth_labels': dict(missing_labels),
        'frozen_utilities': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in utility_files},
        'modes': {name: {**{k: v for k, v in g.items() if k not in ('center_errors', 'visible_center_errors')},
                        'fitted_center_error_m': stats(g['center_errors']),
                        'visible_center_error_m': stats(g['visible_center_errors'])}
                  for name, g in groups.items()},
        'limits': 'All canonical source-time clusters fitted before truth labels. Odom brackets <=0.04s, fixed SDF lidar extrinsic, instantaneous scans only. Truth alignment checked only for offline labels. No exact historical TF/DDS consumer reconstruction. +inf-as-clear is an explicitly unsafe assumption under dropout; its contrast results are not deployment eligible.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--tracker-config', required=True, type=Path)
    parser.add_argument('--trial', type=Path)
    parser.add_argument('--replay-only', action='store_true')
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
        'support_is_certified': False, 'replay_trial': str(args.trial) if args.trial else None,
        'replay_modes': ['strict_finite_boundary', 'assumed_no_return_clear'] if args.trial else []}, indent=2) + '\n')
    (args.output / 'source_identity.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sources}, indent=2) + '\n')
    if args.replay_only and args.trial is None:
        raise ValueError('--replay-only requires --trial')
    with (args.output / 'samples.jsonl').open('w') as stream:
        summary = {'matrix': {} if args.replay_only else synthetic(config, cfg, stream),
            'stationary_arc': [] if args.replay_only else arc(config, cfg, stream),
            'replay': replay(config, cfg, args.trial, stream) if args.trial else None,
            'limits': 'Conditional rectangle/nearest-return model only. Finite background is synthetic known geometry. Missing returns, clutter and line-angle errors can invalidate full support. Radial allowance is empirical, never a certified bound. Accepted fits do not imply task, raw203, CV horizon or physical safety acceptance.'}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
