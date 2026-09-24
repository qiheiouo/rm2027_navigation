#!/usr/bin/env python3
"""Evaluate one fixed physical-size envelope on two existing QP failure trials."""
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OLD = ROOT / 'build/tdt_p2b/runs/dynamic_reference_pilot_v2/tdt_qp_1'
LIVE = ROOT / 'build/tdt_p2b/runs/dynamic_prediction_tracker_live_v2/tdt_qp_1'
INPUTS = ROOT / 'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json'
PROBE = ROOT / 'docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/probe_v2.json'
sys.path.insert(0, str(HERE))
from envelope import predicted_box, fixture_target_acceleration
sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_tracker_probe_20260924'))
from probe import interpolator
sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport, obstacle_polygon, placed, polygon_distance

SIZE = (.45, .55)
HORIZONS = (0, .2, .4, .6, .8, 1.)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def first_body_clearance_breach(poses, polygon, obstacle):
    for row in poses:
        distance = polygon_distance(placed(polygon, row['robot']),
                                    placed(obstacle, row['obstacle']))
        if distance < .05:
            return row['t']
    raise AssertionError('expected a frozen dynamic geometry failure')


def observations(kind, at, rows):
    for row in rows:
        if kind == 'frozen_core':
            if not row['confirmed_near_box']: continue
            yield row['t'], 0., row['estimated_xy'], row['estimated_velocity_xy'], row['estimated_visible_cluster_size_xy']
        else:
            t = row['source_t']
            actual = at(t)
            if actual is None: continue
            tracks = [track for track in row['tracks'] if track['state'] == 2]
            tracks.sort(key=lambda track: math.dist(track['xy'], actual['obstacle'][:2]))
            if not tracks or math.dist(tracks[0]['xy'], actual['obstacle'][:2]) >= 1: continue
            assert row['frame'] == 'map' and row['complete']
            age = row['receive_ros_t'] - t
            assert 0 <= age <= .4
            track = tracks[0]
            yield t, age, track['xy'], track['vxy'], track['size_xy']


def summarize(values):
    ordered = sorted(values)
    return {'median': statistics.median(ordered) if ordered else None,
            'p95': ordered[math.ceil(.95*len(ordered))-1] if ordered else None,
            'max': ordered[-1] if ordered else None}


def evaluate(kind, poses, messages, robot_polygon, obstacle_polygon):
    at = interpolator(poses)
    censor = first_body_clearance_breach(poses, robot_polygon, obstacle_polygon)
    assert max(abs(row['obstacle'][2]) for row in poses) < 1e-8
    accel = fixture_target_acceleration()
    by_horizon = {}
    for horizon in HORIZONS:
        points = []
        for t, age, center, velocity, visible_size in observations(kind, at, messages):
            target_t = t + age + horizon
            if not 16 <= t < censor or target_t >= censor: continue
            truth = at(target_t)
            if truth is None: continue
            full_box_center = truth['obstacle'][:2]
            # Gazebo poses are used exclusively by this offline evaluator.
            current = predicted_box(center, velocity, visible_size, SIZE, age, horizon, 0)
            grown = predicted_box(center, velocity, visible_size, SIZE, age, horizon, accel)
            robot = placed(robot_polygon, truth['robot'])
            points.append({'t': t, 'age': age,
                'current_contains': current.contains(full_box_center, SIZE),
                'grown_contains': grown.contains(full_box_center, SIZE),
                'current_robot_gap': polygon_distance(robot, current.polygon()),
                'grown_robot_gap': polygon_distance(robot, grown.polygon()),
                'grown_width_x': grown.max_x - grown.min_x,
                'grown_width_y': grown.max_y - grown.min_y,
                'grown_min_y': grown.min_y, 'grown_max_y': grown.max_y})
        assert points, (kind, horizon)
        by_horizon[str(horizon)] = {
            'samples_before_body_clearance_breach': len(points),
            'no_growth_full_box_covered': sum(p['current_contains'] for p in points),
            'reference_growth_full_box_covered': sum(p['grown_contains'] for p in points),
            'no_growth_actual_future_robot_overlap': sum(p['current_robot_gap'] == 0 for p in points),
            'reference_growth_actual_future_robot_overlap': sum(p['grown_robot_gap'] == 0 for p in points),
            'reference_growth_actual_future_robot_within_005': sum(p['grown_robot_gap'] < .05 for p in points),
            'reference_growth_width_x_m': summarize([p['grown_width_x'] for p in points]),
            'reference_growth_width_y_m': summarize([p['grown_width_y'] for p in points]),
            'reference_growth_min_y_m': min(p['grown_min_y'] for p in points),
            'reference_growth_max_y_m': max(p['grown_max_y'] for p in points),
        }
    return {'first_sampled_body_clearance_breach_s': censor,
            'horizons': by_horizon}


def main():
    inputs = json.loads(INPUTS.read_text())
    robot = inputs['body_polygon_m']
    obstacle = obstacle_polygon()
    assert max(x for x,y in obstacle)-min(x for x,y in obstacle) == SIZE[0]
    assert max(y for x,y in obstacle)-min(y for x,y in obstacle) == SIZE[1]
    frozen_poses = rows_from_transport(OLD / 'gazebo_poses.jsonl')
    live_poses = rows_from_transport(LIVE / 'gazebo_poses.jsonl')
    frozen_rows = json.loads(PROBE.read_text())['rows']
    live_rows = [json.loads(line) for line in (LIVE / 'predictions.jsonl').open()]
    sources = [INPUTS, PROBE, OLD / 'gazebo_poses.jsonl',
               LIVE / 'gazebo_poses.jsonl', LIVE / 'predictions.jsonl',
               ROOT / 'src/rm_simulation/models/moving_obstacle.sdf',
               ROOT / 'src/rm_simulation/src/moving_obstacle_controller.cpp',
               ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922/dynamic.launch.py']
    result = {'schema': 'rm_dynamic_prediction_v1_extent_shadow/v1',
        'scope': 'Read-only offline evaluation of a single physically sourced extent rule. Actual robot and box future poses are used only to audit coverage and realized-path conflict.',
        'source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in sources},
        'known_fixture_object_extent_xy_m': SIZE,
        'target_amplitude_m': .9, 'target_period_s': 8.,
        'reference_target_acceleration_m_s2': fixture_target_acceleration(),
        'acceleration_is_verified_actual_bound': False,
        'physical_rule': 'Visible cluster center +/- half visible size +/- full known box dimension in each axis; translate by KF velocity*(age+horizon).',
        'optional_growth_rule': 'Add 0.5*reference_target_acceleration*(age+horizon)^2 on each side; target acceleration is not an actuator bound.',
        'frozen_core_replay': evaluate('frozen_core', frozen_poses, frozen_rows, robot, obstacle),
        'live_ros_messages': evaluate('live_ros', live_poses, live_rows, robot, obstacle),
        'mpii_consumer_or_navigation_candidate_run': False,
        'accepted_for_deployment': False}
    with (HERE / 'extent_shadow.json').open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False); output.write('\n')
    for name in ('frozen_core_replay','live_ros_messages'):
        print(name, result[name]['first_sampled_body_clearance_breach_s'])
        for horizon, row in result[name]['horizons'].items():
            print(horizon,row['samples_before_body_clearance_breach'],row['no_growth_full_box_covered'],row['reference_growth_full_box_covered'],row['reference_growth_actual_future_robot_overlap'],row['reference_growth_width_y_m']['median'])


if __name__ == '__main__': main()
