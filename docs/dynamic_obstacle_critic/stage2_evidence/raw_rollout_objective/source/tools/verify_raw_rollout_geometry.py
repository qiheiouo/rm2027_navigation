#!/usr/bin/env python3
"""Independent scalar grid-distance verification of the original raw paths."""
import argparse
import bisect
import json
import math
from pathlib import Path
from analyze_trial import load_truth_rows, rotation, distance
from audit_mechanical_footprint import components, circle_box_gap
from audit_scan_geometry import fixture
from native_snapshot_io import read_snapshot


def verify(source, report):
    native, blocks = read_snapshot(source/'native_cycles/cycle_298.json')
    if report['native_source_ordinal'] != 298 or len(report['rows']) != 300:
        raise ValueError('registered complete raw report required')
    truth = load_truth_rows(source, json.loads((source/'execution.json').read_text()))
    times = [r['t'] for r in truth]
    angles = [truth[0]['obstacle'][2]]
    for a, b in zip(truth, truth[1:]):
        angles.append(angles[-1]+math.remainder(b['obstacle'][2]-a['obstacle'][2], 2*math.pi))
    def actual(query):
        j = bisect.bisect_right(times, query)
        if not 0 < j < len(times):
            raise ValueError('scalar grid truth bracket missing')
        ratio = (query-times[j-1])/(times[j]-times[j-1])
        xy = [truth[j-1]['obstacle'][i]+ratio*(truth[j]['obstacle'][i]-truth[j-1]['obstacle'][i]) for i in (0, 1)]
        return [*xy, angles[j-1]+ratio*(angles[j]-angles[j-1])]
    shapes = components(source/'scene_inputs')
    body = next(s['points'] for s in shapes if s['kind']=='polygon' and s['name']=='base_link/base_collision')
    x, y = fixture(source/'scene_inputs')['actor_dimensions'][:2]
    actor = [(-x/2, -y/2), (x/2, -y/2), (x/2, y/2), (-x/2, y/2)]
    results = {}
    top = report['highest_weight_raw_row']['raw_native_row']
    for epoch, summary in report['epoch_summaries'].items():
        errors = {name: 0. for name in ['body', 'mechanical', 'padded_native']}
        contacts = 0
        first_top_contact = None
        obstacles = [actual(summary['epoch']+(step+1)*native['model_dt']) for step in range(30)]
        for row, label in enumerate(report['rows']):
            if label['raw_native_row'] != row:
                raise ValueError('scalar raw row identity')
            minima = {name: math.inf for name in errors}
            for step, obstacle in enumerate(obstacles):
                index = row*30+step
                robot = [blocks[name]['values'][index] for name in ['x', 'y', 'yaw']]
                obstacle_polygon = rotation(obstacle, actor)
                body_gap = distance(rotation(robot, body), obstacle_polygon)
                padded_gap = distance(rotation(robot, native['padded_footprint']), obstacle_polygon)
                mechanical_gap = body_gap
                for shape in shapes:
                    if shape['kind'] == 'circle':
                        center = rotation(robot, [shape['center']])[0]
                        mechanical_gap = min(mechanical_gap, circle_box_gap(center, shape['radius'], obstacle, (x, y)))
                for name, value in [('body', body_gap), ('mechanical', mechanical_gap), ('padded_native', padded_gap)]:
                    minima[name] = min(minima[name], value)
                if row == top and padded_gap == 0 and first_top_contact is None:
                    first_top_contact = {'step': step+1, 'future_seconds': (step+1)*native['model_dt'],
                                         'truth_epoch': summary['epoch']+(step+1)*native['model_dt'],
                                         'robot_pose': robot, 'actor_pose': obstacle,
                                         'scalar_body_gap': body_gap, 'scalar_mechanical_gap': mechanical_gap,
                                         'scalar_padded_gap': padded_gap}
            contacts += minima['padded_native'] == 0
            for name, value in minima.items():
                captured = label['physical_dynamic_labels'][epoch]['geometry'][name]['original_rollout_grid_min']
                errors[name] = max(errors[name], abs(value-captured))
                if (value == 0) != (captured == 0):
                    raise ValueError('scalar contact classification differs')
        if max(errors.values()) > 2e-12 or contacts != summary['padded_contact_on_original_rollout_grid_rows']:
            raise ValueError('independent scalar grid verification failed')
        results[epoch] = {'raw_grid_poses': 9000, 'maximum_scalar_vector_distance_error': errors,
                          'padded_grid_contact_rows': contacts, 'highest_weight_first_padded_contact': first_top_contact}
    return {'verdict': 'PASS', 'epochs': results, 'numerical_comparison_allowance_only': 2e-12,
            'scope': 'Independent scalar interpolation/polygon/circle distances; exact contact labels, no physical gate tolerance change.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'report', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = verify(args.source, json.loads(args.report.read_text()))
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result), flush=True)
