#!/usr/bin/env python3
"""Audit one Navfn transfer pair and its single pre-registered reproduction."""
import bisect
import hashlib
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / 'build/tdt_p2b'
PAIR = WORK / 'runs/dynamic_prediction_navfn_transfer_v1'
REPRO = WORK / 'runs/dynamic_prediction_navfn_repro_v1'
sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_tracker_probe_20260924'))
from dynamic_metrics import rows_from_transport, obstacle_polygon, placed, padded, polygon_distance
from probe import interpolator
sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_extent_shadow_20260924'))
from envelope import predicted_box, fixture_target_acceleration


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def preserved(path):
    entries = json.loads(path.read_text())['files_sha256' if path.parent.name == 'v1_mppi_ab_20260924' else 'new_files_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in entries.items())
    return {'manifest_sha256': sha(path), 'entries_verified': len(entries)}


def trial(path):
    audit = json.loads((path / 'audit.json').read_text())
    metadata = json.loads((path / 'metadata.json').read_text())
    assert metadata['profile_sha256'] == sha(path / 'profile.yaml')
    assert audit['evidence_valid'] and audit['action_status'] == 4 and audit['recoveries'] == 0
    assert audit['profile_sha256'] == metadata['profile_sha256']
    for name, digest in metadata['input_sha256'].items():
        assert sha(ROOT / name) == digest, name
    start, end = audit['navigation_interval_sim_s']
    poses = rows_from_transport(path / 'gazebo_poses.jsonl')
    at = interpolator(poses)
    first_crossing = next((row for row in poses if start <= row['t'] <= end and row['robot'][0] >= 4.9), None)
    assert first_crossing is not None
    return {
        'directory': str(path.relative_to(ROOT)), 'profile_sha256': metadata['profile_sha256'],
        'action': audit['action_status'], 'recoveries': audit['recoveries'],
        'navigation_duration_sim_s': audit['navigation_duration_sim_s'],
        'traveled_m': audit['traveled_m'], 'low_speed_time_s': audit['stall']['low_speed_time_s'],
        'body_sampled_overlap': audit['geometry']['body']['sampled_moving_overlap'],
        'body_min_sample_m': audit['geometry']['body']['moving_sample_min_m'],
        'body_interpolation_bound_m': audit['geometry']['body']['moving_interpolation_bound_m'],
        'padded_interpolation_bound_m': audit['geometry']['padded']['moving_interpolation_bound_m'],
        'limited_dynamic_gate_pass': audit['limited_dynamic_gate_pass'],
        'prediction_messages': audit['prediction_messages'],
        'critic_consumption_log_samples': len(audit['prediction_consumption_log_samples']),
        'obstacle_y_at_navigation_start_m': at(start)['obstacle'][1],
        'first_robot_x_ge_4p9': {'sim_s': first_crossing['t'],
                                'robot': first_crossing['robot'],
                                'obstacle': first_crossing['obstacle']},
        'body_min_witness': audit['geometry']['body']['moving_min_witness'],
        'source_sha256': {str((path / name).relative_to(ROOT)): sha(path / name) for name in (
            'metadata.json', 'profile.yaml', 'audit.json', 'gazebo_poses.jsonl',
            'observation/summary.json', 'observation/events.jsonl', 'launch.log',
            'observation/commands.jsonl') + (('predictions.jsonl',) if audit['prediction_messages'] else ())},
    }


def collision_window(path):
    poses = rows_from_transport(path / 'gazebo_poses.jsonl')
    at = interpolator(poses)
    predictions = [json.loads(line) for line in (path / 'predictions.jsonl').open()]
    times = [row['receive_ros_t'] for row in predictions]
    commands = [json.loads(line) for line in (path / 'observation/commands.jsonl').open()]
    command_times = [row['t'] for row in commands]
    body = json.loads((ROOT / 'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json').read_text())['body_polygon_m']
    robot_padded = padded(body, .03)
    obstacle = obstacle_polygon()
    first_body_breach = next(row['t'] for row in poses if polygon_distance(
        placed(body, row['robot']), placed(obstacle, row['obstacle'])) < .05)
    first_padded_overlap = next(row['t'] for row in poses if polygon_distance(
        placed(robot_padded, row['robot']), placed(obstacle, row['obstacle'])) == 0)
    first_body_overlap = next(row['t'] for row in poses if polygon_distance(
        placed(body, row['robot']), placed(obstacle, row['obstacle'])) == 0)
    samples = []
    for t in (32.5, 32.8, 32.9, 33., 33.02, 33.08):
        actual = at(t)
        row = predictions[bisect.bisect_right(times, t) - 1]
        track = min((tr for tr in row['tracks'] if tr['state'] == 2),
                    key=lambda tr: math.dist(tr['xy'], actual['obstacle'][:2]), default=None)
        assert track is not None and math.dist(track['xy'], actual['obstacle'][:2]) < 1.
        age = t - row['source_t']
        assert 0 <= age < .4
        box = predicted_box(track['xy'], track['vxy'], track['size_xy'],
                            (.45, .55), age, 0, fixture_target_acceleration())
        ox, oy = actual['obstacle'][:2]
        miss = max(0., box.min_x - (ox - .225), (ox + .225) - box.max_x,
                   box.min_y - (oy - .275), (oy + .275) - box.max_y)
        command = commands[bisect.bisect_right(command_times, t) - 1]
        samples.append({'sim_s': t, 'actual_body_gap_m': polygon_distance(
            placed(body, actual['robot']), placed(obstacle, actual['obstacle'])),
            'actual_padded_gap_m': polygon_distance(placed(robot_padded, actual['robot']),
                                                    placed(obstacle, actual['obstacle'])),
            'source_age_s': age, 'predicted_box_full_object_miss_m': miss,
            'padded_to_current_predicted_box_gap_m': polygon_distance(
                placed(robot_padded, actual['robot']), box.polygon()),
            'last_command_sim_s': command['t'],
            'last_command_planar_speed_m_s': math.hypot(command['vx'], command['vy'])})
    wall = [row['receive_wall_ns'] * 1e-9 for row in predictions]
    sampled_scores = []
    pattern = re.compile(r'\[([0-9]+\.[0-9]+)\].*predicted-collision trajectories (\d+)/(\d+)')
    for line in (path / 'launch.log').open(errors='replace'):
        match = pattern.search(line)
        if not match:
            continue
        stamp = float(match[1])
        i = bisect.bisect_left(wall, stamp)
        i = min((j for j in (i-1, i) if 0 <= j < len(wall)),
                key=lambda j: abs(wall[j] - stamp))
        t = predictions[i]['receive_ros_t']
        if 30 <= t <= 34:
            sampled_scores.append({'approx_sim_s': t, 'collision_rollouts': int(match[2]),
                                   'rollouts': int(match[3]),
                                   'wall_match_error_s': abs(wall[i] - stamp)})
    assert any(row['collision_rollouts'] == 300 for row in sampled_scores)
    return {'first_body_below_005m_sim_s': first_body_breach,
            'first_padded_overlap_sim_s': first_padded_overlap,
            'first_body_overlap_sim_s': first_body_overlap,
            'selected_samples': samples,
            'throttled_critic_logs_approx_sim': sampled_scores,
            'note': 'Prediction box evaluated from latest recorder message at sample time; critic logs mapped by nearest wall timestamp, not exact same-cycle MPPI trace.'}

def main():
    old = ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/manifest.json'
    prior = ROOT / 'docs/dynamic_navigation/evidence/v1_failure_window_20260924/manifest.json'
    frozen = {'old_v1_ab': preserved(old), 'prior_failure_window': preserved(prior)}
    plan = json.loads((PAIR / 'plan.json').read_text())
    repro_plan = json.loads((REPRO / 'plan.json').read_text())
    assert plan['phase_seconds'] == repro_plan['phase_seconds'] == 0
    assert repro_plan['reference_profile_sha256'] == sha(PAIR / 'candidate_navfn_1/profile.yaml')
    names = [('baseline', PAIR / 'baseline_navfn_1'),
             ('candidate_first', PAIR / 'candidate_navfn_1'),
             ('candidate_reproduction', REPRO / 'candidate_navfn_1')]
    trials = {name: trial(path) for name, path in names}
    assert trials['baseline']['body_interpolation_bound_m'] < .05
    assert trials['candidate_first']['body_sampled_overlap']
    assert not trials['candidate_reproduction']['body_sampled_overlap']
    assert trials['candidate_reproduction']['limited_dynamic_gate_pass']
    result = {
        'schema': 'rm_dynamic_prediction_navfn_transfer_review/v1',
        'accepted_for_deployment': False,
        'scope': 'One prespecified Navfn phase-0 baseline/candidate pair and one fixed-profile collision reproduction; not the T-DT matrix, not a pass-rate estimate.',
        'old_evidence_preservation': frozen,
        'plans_sha256': {str(path.relative_to(ROOT)): sha(path) for path in (PAIR / 'plan.json', REPRO / 'plan.json')},
        'trials': trials,
        'first_candidate_collision_window': collision_window(PAIR / 'candidate_navfn_1'),
    }
    with (HERE / 'review.json').open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    for name, row in trials.items():
        print(name, row['body_sampled_overlap'], round(row['body_interpolation_bound_m'], 4),
              round(row['traveled_m'], 2), row['recoveries'])


if __name__ == '__main__':
    main()
