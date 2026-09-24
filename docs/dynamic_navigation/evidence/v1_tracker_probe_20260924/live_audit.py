#!/usr/bin/env python3
"""Audit one shadow-only fixture trial; never infer controller prediction benefit."""
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRIAL = ROOT / 'build/tdt_p2b/runs/dynamic_prediction_tracker_live_v2/tdt_qp_1'
INPUTS = ROOT / 'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json'
sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport, obstacle_polygon, geometry_metrics
sys.path.insert(0, str(HERE))
from probe import interpolator
from coverage_audit import box_size, summarize
sys.path.insert(0, str(ROOT / 'experiments/tdt_planner'))


def read_jsonl(path):
    return [json.loads(line) for line in path.open()]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, p):
    values = sorted(values)
    return values[math.ceil(p * len(values)) - 1] if values else None


def summary(values):
    return {'count': len(values), 'median': statistics.median(values) if values else None,
            'p95': percentile(values, .95), 'max': max(values) if values else None}


def main():
    trial = TRIAL
    assert int((trial / 'docker_exit.txt').read_text()) == 1
    metadata = json.loads((trial / 'metadata.json').read_text())
    assert sha(trial / 'profile.yaml') == metadata['profile_sha256']
    predictions = read_jsonl(trial / 'predictions.jsonl')
    diagnostics = read_jsonl(trial / 'diagnostics.jsonl')
    events = read_jsonl(trial / 'observation/events.jsonl')
    start = next(r['t'] for r in events if r.get('event') == 'preflight')
    end = next(r['t'] for r in events if r.get('event') == 'navigation_result')
    all_poses = rows_from_transport(trial / 'gazebo_poses.jsonl')
    times = [r['t'] for r in all_poses]
    assert times[0] <= start and times[-1] >= end
    lo = max(0, bisect.bisect_right(times, start) - 1)
    hi = bisect.bisect_left(times, end)
    nav_poses = all_poses[lo:hi+1]
    at = interpolator(all_poses)
    polygon = json.loads(INPUTS.read_text())['body_polygon_m']
    box = obstacle_polygon()
    geometry = geometry_metrics(nav_poses, polygon, .03, box)
    sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
    from dynamic_metrics import placed, polygon_distance
    first_body_005 = next((r['t'] for r in nav_poses if polygon_distance(
        placed(polygon, r['robot']), placed(box, r['obstacle'])) < .05), None)
    censor = first_body_005 if first_body_005 is not None else end
    pre = [r for r in predictions if start <= r['source_t'] < censor]
    late = [r for r in pre if r['source_t'] >= censor-2]
    matched = []
    for row in late:
        actual = at(row['source_t'])
        if actual is None: continue
        tracks = [t for t in row['tracks'] if t['state'] == 2]
        tracks.sort(key=lambda t: math.dist(t['xy'], actual['obstacle'][:2]))
        if tracks and math.dist(tracks[0]['xy'], actual['obstacle'][:2]) < 1:
            track = tracks[0]
            matched.append({'t': row['source_t'], 'track_id': track['id'],
                'estimated_xy': track['xy'], 'estimated_visible_cluster_size_xy': track['size_xy'],
                'actual_obstacle_xy': actual['obstacle'][:2]})
    status = Counter(s['message'] for row in diagnostics for s in row['statuses']
                     if s['name'] == 'dynamic_obstacle_tracking_shadow')
    source_age = [r['receive_ros_t']-r['source_t'] for r in predictions]
    assert all(r['schema'] == 'rm_dynamic_obstacle_predictions/v1' and r['frame'] == 'map'
               and r['complete'] for r in predictions)
    assert all(r['source_t'] <= r['processing_t'] <= r['receive_ros_t'] + .2 for r in predictions)
    raw = json.loads((trial / 'observation/summary.json').read_text())
    checks = dict(raw['checks'])
    checks['body_clearance_at_least_005m'] = geometry['body_clearance_at_least_005m']
    checks['padded_footprint_no_contact'] = geometry['padded_footprint_no_contact']
    result = {'schema': 'rm_dynamic_prediction_v1_live_shadow_audit/v1',
        'trial': str(trial.relative_to(ROOT)),
        'source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in
            [trial / 'profile.yaml', trial / 'metadata.json', trial / 'predictions.jsonl',
             trial / 'diagnostics.jsonl', trial / 'gazebo_poses.jsonl',
             trial / 'observation/events.jsonl', trial / 'observation/summary.json', INPUTS]},
        'navigation_interval_s': [start, end],
        'first_sampled_body_clearance_below_005_s': first_body_005,
        'navigation': {'action_status': raw['action_status'], 'recoveries': raw['recoveries'],
            'xy_error_m': raw['final_xy_error_m'], 'yaw_error_rad': raw['final_yaw_error_rad'],
            'limited_dynamic_geometry_and_goal_pass': all(checks.values()),
            'checks': checks, 'geometry': geometry},
        'shadow_stream': {'prediction_messages': len(predictions),
            'diagnostic_messages': len(diagnostics), 'statuses': dict(status),
            'incomplete_messages': sum(not r['complete'] for r in predictions),
            'confirmed_messages_before_clearance_breach': sum(any(t['state']==2 for t in r['tracks']) for r in pre),
            'messages_before_clearance_breach': len(pre),
            'source_to_processing_s': summary([r['processing_t']-r['source_t'] for r in predictions]),
            'source_to_recorder_receive_s': summary(source_age),
            'prediction_source_gap_s': summary([b['source_t']-a['source_t'] for a,b in zip(predictions,predictions[1:])]),
            'late_prebreach_confirmed_near_box': len(matched),
            'late_prebreach_confirmed_ids': dict(Counter(r['track_id'] for r in matched)),
            'late_prebreach_visible_occupancy_coverage': summarize(matched, box_size())},
        'limits': ['Read-only tracker; no MPPI prediction consumer or controlled A/B.',
            'Map is experiment-only static geometry from SDF, not field map.',
            'Gazebo actual obstacle pose and dimensions are evaluation truth only.',
            'Recorder callback source age is not controller consumption age or DDS latency.',
            'Reference new-car polygon on surrogate simulation carrier, not final CAD or hardware.'],
        'accepted_for_deployment': False}
    with (HERE / 'live_audit_v2.json').open('x') as out:
        json.dump(result, out, indent=2, allow_nan=False); out.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','navigation')}, indent=2))
    print('navigation', json.dumps({k:v for k,v in result['navigation'].items() if k != 'geometry'}))


if __name__ == '__main__':
    main()
