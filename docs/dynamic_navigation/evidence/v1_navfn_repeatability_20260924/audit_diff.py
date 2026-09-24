#!/usr/bin/env python3
"""Read-only differential audit of the two frozen Navfn+V1 phase-0 trials."""
import bisect
import hashlib
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / 'build/tdt_p2b/runs'
FIRST = RUNS / 'dynamic_prediction_navfn_transfer_v1/candidate_navfn_1'
REPRO = RUNS / 'dynamic_prediction_navfn_repro_v1/candidate_navfn_1'
OLD = ROOT / 'docs/dynamic_navigation/evidence/v1_navfn_transfer_20260924/manifest.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_jsonl(path):
    return [json.loads(line) for line in path.open()]


def nearest(rows, t):
    return min(rows, key=lambda row: abs(row['t'] - t))


def last_prediction(rows, t):
    times = [row['receive_ros_t'] for row in rows]
    index = bisect.bisect_right(times, t) - 1
    assert index >= 0
    row = rows[index]
    confirmed = [track for track in row['tracks'] if track['state'] == 2]
    return {
        'source_t': row['source_t'], 'source_age_s': t - row['source_t'],
        'confirmed_tracks': len(confirmed),
        'track_xy': confirmed[0]['xy'] if len(confirmed) == 1 else None,
        'track_vxy': confirmed[0]['vxy'] if len(confirmed) == 1 else None,
        'visible_size_xy': confirmed[0]['size_xy'] if len(confirmed) == 1 else None,
    }


def score_samples(path, predictions):
    walls = [row['receive_wall_ns'] / 1e9 for row in predictions]
    pattern = re.compile(r'\[([0-9]+\.[0-9]+)\].*predicted-collision trajectories (\d+)/(\d+)')
    samples = []
    for line in (path / 'launch.log').open(errors='replace'):
        match = pattern.search(line)
        if match is None:
            continue
        stamp = float(match[1])
        index = bisect.bisect_left(walls, stamp)
        index = min((j for j in (index - 1, index) if 0 <= j < len(walls)),
                    key=lambda j: abs(walls[j] - stamp))
        t = predictions[index]['receive_ros_t']
        if 23 <= t <= 28.5:
            samples.append({
                'approx_sim_s': t, 'colliding_rollouts': int(match[2]),
                'total_rollouts': int(match[3]),
                'nearest_recorder_wall_error_s': abs(walls[index] - stamp),
            })
    return samples


def trial(path):
    audit = json.loads((path / 'audit.json').read_text())
    tracks = read_jsonl(path / 'predictions.jsonl')
    trajectory = read_jsonl(path / 'observation/trajectory.jsonl')
    commands = read_jsonl(path / 'observation/commands.jsonl')
    plans = read_jsonl(path / 'observation/plans.jsonl')
    assert audit['evidence_valid'] and audit['action_status'] == 4 and audit['recoveries'] == 0
    assert len(plans) >= 2 and len(tracks) > 500
    checkpoints = {}
    for t in (24.0, 24.5, 25.0, 26.0, 27.0):
        pose = nearest(trajectory, t)
        command = nearest(commands, t)
        checkpoints[str(t)] = {
            'pose_t': pose['t'], 'robot_xy': [pose['x'], pose['y']],
            'command_t': command['t'], 'command_vx_vy_wz': [command['vx'], command['vy'], command['wz']],
            'prediction': last_prediction(tracks, t),
        }
    return {
        'profile_sha256': digest(path / 'profile.yaml'),
        'navigation_start_sim_s': audit['navigation_interval_sim_s'][0],
        'body_sampled_overlap': audit['geometry']['body']['sampled_moving_overlap'],
        'body_interpolation_bound_m': audit['geometry']['body']['moving_interpolation_bound_m'],
        'initial_plan_xy_sha256': hashlib.sha256(json.dumps(plans[0]['xy']).encode()).hexdigest(),
        'initial_plan_points': len(plans[0]['xy']),
        'checkpoints': checkpoints,
        'throttled_critic_samples': score_samples(path, tracks),
        '_trajectory': trajectory,
    }


def main():
    frozen = json.loads(OLD.read_text())['files_sha256']
    for name, expected in frozen.items():
        assert digest(ROOT / name) == expected, name
    first = trial(FIRST)
    repro = trial(REPRO)
    assert first['profile_sha256'] == repro['profile_sha256']
    assert first['initial_plan_xy_sha256'] == repro['initial_plan_xy_sha256']
    assert abs(first['navigation_start_sim_s'] - repro['navigation_start_sim_s']) < .005
    assert first['body_sampled_overlap'] and not repro['body_sampled_overlap']
    divergence = {}
    for threshold in (.05, .1, .5):
        hit = None
        for n in range(400, 827):
            t = n * .04
            a = nearest(first['_trajectory'], t)
            b = nearest(repro['_trajectory'], t)
            distance = math.dist((a['x'], a['y']), (b['x'], b['y']))
            if distance > threshold:
                hit = {'nominal_sim_s': t, 'first_pose_sim_s': a['t'],
                       'reproduction_pose_sim_s': b['t'], 'xy_separation_m': distance}
                break
        divergence[str(threshold)] = hit
    del first['_trajectory'], repro['_trajectory']
    result = {
        'schema': 'rm_dynamic_prediction_navfn_repeatability/v1',
        'accepted_for_deployment': False,
        'source_manifest_sha256': digest(OLD),
        'source_files_verified': len(frozen),
        'scope': 'Frozen phase-0 trials only; no new navigation run or same-cycle MPPI trace.',
        'first': first, 'reproduction': repro,
        'first_pose_separation_threshold_crossings': divergence,
        'limitation': 'Throttled critic logs use nearest recorder wall timestamp, not a synchronized MPPI cycle. This audit establishes divergence, not its unique cause.',
    }
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
