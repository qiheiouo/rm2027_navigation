#!/usr/bin/env python3
"""Read-only audit of the frozen V1 phase-0 repeat failures."""
import bisect
import hashlib
import json
import math
import re
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SERIES = ROOT / 'build/tdt_p2b/runs/dynamic_prediction_mppi_multiphase_v2/phase_0'
sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_extent_shadow_20260924'))
sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_tracker_probe_20260924'))
from dynamic_metrics import rows_from_transport, obstacle_polygon, placed, polygon_distance
from envelope import predicted_box, fixture_target_acceleration
from probe import interpolator

SIZE = (.45, .55)
HORIZONS = (0., .2, .4, .6, .8, 1.)
BODY = json.loads((ROOT / 'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json').read_text())['body_polygon_m']
OBSTACLE = obstacle_polygon()


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.open()]


def miss_m(box, actual):
    x, y = actual
    return max(0., box.min_x - (x - SIZE[0] / 2),
               (x + SIZE[0] / 2) - box.max_x,
               box.min_y - (y - SIZE[1] / 2),
               (y + SIZE[1] / 2) - box.max_y)


def selected_track(message, truth):
    """Truth-assisted pairing is exclusively an offline audit operation."""
    tracks = [track for track in message['tracks'] if track['state'] == 2]
    if not tracks:
        return None
    track = min(tracks, key=lambda x: math.dist(x['xy'], truth['obstacle'][:2]))
    return track if math.dist(track['xy'], truth['obstacle'][:2]) < 1. else None


def coverage(predictions, at, start, end):
    result = {}
    for horizon in HORIZONS:
        checked = []
        for message in predictions:
            now = message['receive_ros_t']
            if message['schema'] != 'rm_dynamic_obstacle_predictions/v1' or message['frame'] != 'odom' or not message['complete']:
                continue
            if now < start or now + horizon >= end:
                continue
            truth = at(now + horizon)
            if truth is None:
                continue
            track = selected_track(message, truth)
            if track is None:
                continue
            age = now - message['source_t']
            if not 0 <= age <= .4:
                continue
            box = predicted_box(track['xy'], track['vxy'], track['size_xy'],
                                SIZE, age, horizon, fixture_target_acceleration())
            realized_robot = placed(BODY, truth['robot'])
            true_gap = polygon_distance(realized_robot, placed(OBSTACLE, truth['obstacle']))
            predicted_gap = polygon_distance(realized_robot, box.polygon())
            checked.append((now, miss_m(box, truth['obstacle'][:2]),
                            (box.max_y - box.min_y) / SIZE[1], true_gap, predicted_gap))
        assert checked
        misses = [(time, miss) for time, miss, *_ in checked if miss > 1e-9]
        result[str(horizon)] = {
            'checked': len(checked), 'not_fully_covered': len(misses),
            'maximum_uncovered_extent_m': max(row[1] for row in checked),
            'uncovered_when_realized_robot_within_05m': sum(row[1] > 1e-9 and row[3] < .5 for row in checked),
            'realized_robot_false_safe_samples': sum(row[1] > 1e-9 and row[3] < .05 and row[4] > .05 for row in checked),
            'median_predicted_to_true_y_width_ratio': statistics.median(x[2] for x in checked),
            'first_uncovered_receive_sim_s': misses[0][0] if misses else None,
        }
    return result


def mapped_logs(launch, predictions):
    wall = [message['receive_wall_ns'] * 1e-9 for message in predictions]
    mapped = []
    pattern = re.compile(r'\[([0-9]+\.[0-9]+)\].*predicted-collision trajectories (\d+)/(\d+)')
    for line in launch.splitlines():
        match = pattern.search(line)
        if not match:
            continue
        stamp = float(match[1])
        i = bisect.bisect_left(wall, stamp)
        i = min((j for j in (i-1, i) if 0 <= j < len(wall)),
                key=lambda j: abs(wall[j] - stamp))
        mapped.append({'approx_sim_s': predictions[i]['receive_ros_t'],
                       'collision_rollouts': int(match[2]), 'rollouts': int(match[3]),
                       'wall_match_error_s': abs(wall[i] - stamp)})
    return mapped


def event_details(event, at, predictions, receive_times):
    t = event['t']
    actual = at(t)
    assert actual is not None
    message = event['message']
    out = {'sim_s': t, 'robot_pose': actual['robot'],
           'actual_obstacle_pose': actual['obstacle'],
           'actual_body_to_box_gap_m': polygon_distance(
               placed(BODY, actual['robot']), placed(OBSTACLE, actual['obstacle']))}
    i = bisect.bisect_right(receive_times, t) - 1
    if i >= 0:
        prediction = predictions[i]
        track = selected_track(prediction, actual)
        out['latest_prediction_source_age_s'] = t - prediction['source_t']
        out['latest_prediction_confirmed_near_box'] = track is not None
        if track is not None:
            box = predicted_box(track['xy'], track['vxy'], track['size_xy'], SIZE,
                                t - prediction['source_t'], 0., fixture_target_acceleration())
            out['latest_prediction_box'] = [box.min_x, box.min_y, box.max_x, box.max_y]
            out['latest_prediction_full_box_miss_m'] = miss_m(box, actual['obstacle'][:2])
    if 'endpoint_witness_v1=' in message:
        witness = json.JSONDecoder().raw_decode(message.split('endpoint_witness_v1=', 1)[1])[0]
        pose_match = re.search(r'\[input=nav2_master start=\(([^,]+),([^\)]+)\).*?start_yaw=([^ ]+)', message)
        assert pose_match
        start_pose = tuple(map(float, pose_match.groups()))
        cell_x, cell_y = witness['start']['collision']['cell']
        resolution = witness['resolution']
        x = witness['origin'][0] + cell_x * resolution
        y = witness['origin'][1] + cell_y * resolution
        cell = [(x, y), (x+resolution, y), (x+resolution, y+resolution), (x, y+resolution)]
        out.update({'kind': 'start_blocked_conservative_circle', 'witness': witness,
                    'logged_planner_start_pose': start_pose,
                    'body_polygon_to_witness_cell_gap_m': polygon_distance(placed(BODY, start_pose), cell)})
    elif 'snapshot_admission_v1=' in message:
        out.update({'kind': 'unsafe_latest_path',
                    'snapshot_admission': json.JSONDecoder().raw_decode(
                        message.split('snapshot_admission_v1=', 1)[1])[0]})
    return out


def analyze(planner):
    path = SERIES / f'candidate_{planner}_1'
    source = [path / name for name in ('predictions.jsonl', 'gazebo_poses.jsonl',
                                      'observation/events.jsonl', 'observation/summary.json',
                                      'launch.log')]
    predictions = rows(source[0]); receive_times = [x['receive_ros_t'] for x in predictions]
    assert receive_times == sorted(receive_times)
    poses = rows_from_transport(source[1]); at = interpolator(poses)
    assert max(abs(row['obstacle'][2]) for row in poses) < 1e-8
    events = rows(source[2]); summary = json.loads(source[3].read_text())
    pre = [e for e in events if e.get('event') == 'preflight']
    nav = [e for e in events if e.get('event') == 'navigation_result']
    assert len(pre) == len(nav) == 1 and pre[0]['result']['status'] == 4 and nav[0]['status'] == 4
    assert nav[0]['recoveries'] == summary['recoveries']
    failures = [e for e in events if 'endpoint_witness_v1=' in e.get('message','') or
                'snapshot_admission_v1=' in e.get('message','')]
    details = [event_details(e, at, predictions, receive_times) for e in failures]
    logs = mapped_logs(source[4].read_text(errors='replace'), predictions)
    return {'planner': planner, 'navigation_sim_s': [pre[0]['t'], nav[0]['t']],
            'action_status': nav[0]['status'], 'recoveries': nav[0]['recoveries'],
            'failure_events': details, 'prediction_full_box_coverage': coverage(
                predictions, at, pre[0]['t'], nav[0]['t']),
            'critic_sampled_logs_approx_sim_time': logs,
            'source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in source}}


def main():
    old_manifest = ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/manifest.json'
    manifest = json.loads(old_manifest.read_text())['files_sha256']
    mismatches = [name for name, digest in manifest.items() if sha(ROOT / name) != digest]
    assert not mismatches, mismatches
    trials = [analyze(planner) for planner in ('tdt_qp', 'tdt_astar')]
    assert [t['recoveries'] for t in trials] == [5, 1]
    assert len(trials[0]['failure_events']) == 4 and len(trials[1]['failure_events']) == 1
    assert all(e['kind'] == 'start_blocked_conservative_circle' for e in trials[0]['failure_events'])
    assert trials[1]['failure_events'][0]['kind'] == 'unsafe_latest_path'
    result = {'schema': 'rm_dynamic_prediction_v1_failure_window/v1',
              'accepted_for_deployment': False, 'new_navigation_trials': 0,
              'old_manifest_entries_verified': len(manifest),
              'old_manifest_sha256': sha(old_manifest),
              'time_mapping_note': 'INFO logs mapped to nearest prediction receiver wall timestamp; approximate only, not a same-cycle score trace.',
              'coverage_note': 'Compare recorded prediction at receive_ros_t+h against linearly interpolated Gazebo box; truth assists track pairing only for audit. Finite samples do not certify future coverage.',
              'trials': trials}
    target = HERE / 'analysis.json'
    with target.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    for trial in trials:
        print(trial['planner'], trial['recoveries'],
              [(e['sim_s'], e['kind'], round(e['actual_body_to_box_gap_m'], 4))
               for e in trial['failure_events']])
        print('horizon1', trial['prediction_full_box_coverage']['1.0'])


if __name__ == '__main__':
    main()
