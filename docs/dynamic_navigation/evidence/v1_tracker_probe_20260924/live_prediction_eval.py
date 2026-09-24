#!/usr/bin/env python3
"""Evaluate live shadow forecasts against pre-breach fixture truth, offline."""
import json
import math
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRIAL = ROOT / 'build/tdt_p2b/runs/dynamic_prediction_tracker_live_v2/tdt_qp_1'
sys.path.insert(0, str(HERE))
from probe import interpolator
from coverage_audit import box_size, deficits
sys.path.insert(0, str(ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport


def contained(row, size):
    return all(value <= 1e-9 for axis in deficits(row, size).values() for value in axis.values())


def summarize(rows):
    def stats(key):
        values = sorted(row[key] for row in rows)
        if not values: return {'median_m': None, 'p95_m': None, 'max_m': None}
        return {'median_m': statistics.median(values),
                'p95_m': values[math.ceil(.95*len(values))-1], 'max_m': values[-1]}
    return {'samples': len(rows), 'source_forecast_error': stats('source_error'),
            'measured_age_compensated_error': stats('aged_error'),
            'source_visible_bbox_contains_true_box': sum(row['source_contains'] for row in rows),
            'aged_visible_bbox_contains_true_box': sum(row['aged_contains'] for row in rows)}


def main():
    nav = json.loads((HERE / 'live_audit_v2.json').read_text())
    censor = nav['first_sampled_body_clearance_below_005_s']
    assert censor is not None
    poses = rows_from_transport(TRIAL / 'gazebo_poses.jsonl')
    at = interpolator(poses)
    messages = [json.loads(line) for line in (TRIAL / 'predictions.jsonl').open()]
    evaluations = {key: [] for key in ('0.2', '0.4', '0.6', '0.8', '1.0')}
    for message in messages:
        t = message['source_t']
        if not nav['navigation_interval_s'][0] <= t < censor: continue
        actual = at(t)
        if actual is None: continue
        tracks = [track for track in message['tracks'] if track['state'] == 2]
        tracks.sort(key=lambda track: math.dist(track['xy'], actual['obstacle'][:2]))
        if not tracks or math.dist(tracks[0]['xy'], actual['obstacle'][:2]) >= 1: continue
        track = tracks[0]
        age = message['receive_ros_t'] - t
        assert age >= 0
        for key, index in (('0.2',1), ('0.4',3), ('0.6',5), ('0.8',7), ('1.0',9)):
            horizon = float(key)
            if message['receive_ros_t'] + horizon >= censor: continue
            at_source_future = at(t + horizon)
            at_aged_future = at(message['receive_ros_t'] + horizon)
            if at_source_future is None or at_aged_future is None: continue
            source_xy = track['future_xy'][index]
            aged_xy = [track['xy'][axis] + track['vxy'][axis]*(age+horizon) for axis in (0,1)]
            source_truth = at_source_future['obstacle'][:2]
            aged_truth = at_aged_future['obstacle'][:2]
            evaluations[key].append({
                'source_error': math.dist(source_xy, source_truth),
                'aged_error': math.dist(aged_xy, aged_truth),
                'source_contains': contained({'estimated_xy': source_xy,
                    'estimated_visible_cluster_size_xy': track['size_xy'],
                    'actual_obstacle_xy': source_truth}, box_size()),
                'aged_contains': contained({'estimated_xy': aged_xy,
                    'estimated_visible_cluster_size_xy': track['size_xy'],
                    'actual_obstacle_xy': aged_truth}, box_size()),
            })
    result = {'schema': 'rm_dynamic_prediction_v1_live_forecast_eval/v1',
        'scope': 'Actual ROS shadow messages and measured recorder callback age; Gazebo future is offline evaluation truth only. All target times precede first sampled 0.05m body-clearance breach.',
        'censor_sim_s': censor,
        'source_forecast_rule': 'Use published prediction[index] at source stamp + (index+1)*dt.',
        'age_compensated_rule': 'Use published state [x,y,vx,vy] at source stamp, propagate CV for measured (recorder_receive_ros_stamp - source_stamp) + horizon.',
        'fixture_full_box_size_xy_m': box_size(),
        'visible_bbox_warning': 'Published size is a visible scan cluster, not confirmed full obstacle extent.',
        'horizons': {key: summarize(values) for key,values in evaluations.items()},
        'controller_consumer_tested': False,
        'accepted_for_deployment': False}
    with (HERE / 'live_prediction_eval_v2.json').open('x') as out:
        json.dump(result, out, indent=2, allow_nan=False); out.write('\n')
    print(json.dumps(result['horizons'], indent=2))


if __name__ == '__main__':
    main()
