"""Finite A21 recorded-value comparison; no executed R4 behavior claim."""
import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

root, output = map(Path, sys.argv[1:3])

def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def stats(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    return {'n': len(values), 'median': statistics.median(values),
            'p95': values[max(0, math.ceil(.95*len(values))-1)], 'max': values[-1], 'min': values[0]}

def valid(row):
    return row['valid'] == '1'

def wait(row):
    return math.hypot(float(row['vx']), float(row['vy'])) < .02

def reversals(selected, key):
    count, prior = 0, 0
    for row in selected:
        if not valid(row):
            prior = 0
            continue
        value = float(row[key])
        sign = 1 if value > .02 else (-1 if value < -.02 else 0)
        if sign:
            count += prior != 0 and sign != prior
            prior = sign
    return count

def resume(selected, clear_ns):
    # Three consecutive available forward proposals >.05 m/s after clear event.
    streak = []
    for row in selected:
        epoch = int(row['epoch_ns'])
        if epoch < clear_ns:
            continue
        if valid(row) and float(row['forward']) > .05 and (not streak or epoch-int(streak[-1]['epoch_ns']) <= 100_000_000):
            streak.append(row)
            if len(streak) == 3:
                return (int(streak[0]['epoch_ns'])-clear_ns)*1e-9
        else:
            streak = []
    return None

replay = rows(output/'replay.csv')
probes = rows(output/'probe.csv')
summary = {'stage': 'Research', 'scope': 'Original A16 S1/S2 recorded sources; virtual world XY histories; no R4 command execution',
           'circle_radius_m': math.hypot(.30, .25)+.03,
           'thresholds': {'wait_speed_mps': .02, 'slow_forward_mps': .05,
                          'paired_forward_drop_mps': .05, 'axis_reversal_deadband_mps': .02,
                          'resume': '3 consecutive available forward proposals > .05 m/s'}, 'scenes': {}}
for scene in ('S1', 'S2'):
    events = json.loads((root/f'experiments/r4_corrected_runtime_shadow/evidence/{scene}_events.json').read_text())['events']
    begin = next(e['ROS_ns'] for e in events if e['kind'] == 'goal_accepted')
    motion = next(e['ROS_ns'] for e in events if e['kind'] == 'obstacle_motion_start')
    clear = next(e['ROS_ns'] for e in events if e['kind'] == 'clear_target')
    values = [r for r in replay if r['scene'] == scene]
    source_paths = {r['cycle']: r['path_revision'] for r in rows(root/f'experiments/r4_corrected_runtime_shadow/evidence/{scene}_cycles.csv')}
    for row in values:
        row['path_revision'] = source_paths[row['cycle']]
    conditions = {}
    for label in ('observed', 'no_observed_dynamic'):
        selected = [r for r in values if r['condition'] == label]
        available = [r for r in selected if valid(r)]
        window = [r for r in selected if motion <= int(r['epoch_ns']) < clear]
        active = [r for r in window if valid(r)]
        differences = [math.hypot(float(a['vx'])-float(b['vx']), float(a['vy'])-float(b['vy']))
                       for a, b in zip(selected, selected[1:]) if valid(a) and valid(b) and int(b['epoch_ns'])-int(a['epoch_ns']) <= 100_000_000]
        conditions[label] = {
            'samples': len(selected), 'valid': len(available),
            'reasons': dict(Counter(r['reason'] for r in selected)),
            'dynamic_window': {'samples': len(window), 'valid': len(active),
                               'nonzero_nominal_cost': sum(float(r['nominal_cost']) > 1e-8 for r in active),
                               'nonzero_solved_cost': sum(float(r['solved_cost']) > 1e-8 for r in active),
                               'wait': sum(wait(r) for r in active),
                               'slow_or_backward_forward': sum(float(r['forward']) < .05 for r in active)},
            'solver_ms_all_invoked': stats([float(r['solver_ms']) for r in selected if float(r['solver_ms']) > 0]),
            'elapsed_ms_available': stats([float(r['elapsed_ms']) for r in available]),
            'warm': sum(r['used_warm'] == '1' for r in available),
            'accepted_nonzero_history_wz': sum(float(r['raw_history_wz']) != 0 for r in available),
            'accepted_nonzero_measured_wz': sum(float(r['raw_measured_wz']) != 0 for r in available),
            'future_observed_support_overlap': sum(float(r['minimum_clearance']) < 0 for r in available),
            'current_observed_support_overlap': sum(float(r['current_clearance']) < 0 for r in available),
            'resume_after_clear_s': resume(selected, clear),
            'proposal_reversals': {k: reversals(selected, k) for k in ('vx', 'vy', 'forward')},
            'dynamic_window_proposal_reversals': {k: reversals(window, k) for k in ('vx', 'vy', 'forward')},
            'source_path_updates': sum(a['path_revision'] != b['path_revision'] for a,b in zip(selected, selected[1:])),
            'adjacent_proposal_delta_norm_mps': stats(differences),
        }
    observed = [r for r in values if r['condition'] == 'observed']
    ablation = {r['cycle']: r for r in values if r['condition'] == 'no_observed_dynamic'}
    paired = [(r, ablation[r['cycle']]) for r in observed if valid(r) and valid(ablation[r['cycle']])]
    dynamic = [(r,b) for r,b in paired if motion <= int(r['epoch_ns']) < clear]
    slowed = [(r,b) for r,b in dynamic if float(b['forward'])-float(r['forward']) >= .05]
    advance = [(r,b) for r,b in slowed if float(r['current_clearance']) > 0 and float(r['nominal_cost']) > 1e-8]
    summary['scenes'][scene] = {'conditions': conditions, 'paired': {
        'valid_pairs': len(paired), 'dynamic_pairs': len(dynamic), 'dynamic_forward_drop_at_least_05': len(slowed),
        'advance_response_pairs': len(advance),
        'first_advance_since_goal_s': (int(advance[0][0]['epoch_ns'])-begin)*1e-9 if advance else None,
        'forward_drop_dynamic_mps': stats([float(b['forward'])-float(r['forward']) for r,b in dynamic]),
        'command_difference_dynamic_mps': stats([math.hypot(float(r['vx'])-float(b['vx']),float(r['vy'])-float(b['vy'])) for r,b in dynamic]),
    }}
summary['controlled_probes'] = {}
for kind in dict.fromkeys(r['case'] for r in probes):
    selected = [r for r in probes if r['case'] == kind]
    available = [r for r in selected if valid(r)]
    result = {'samples': len(selected), 'valid': len(available),
              'reasons': dict(Counter(r['reason'] for r in selected)),
              'statuses': dict(Counter(r['status'] for r in selected)),
              'source_preserved': sum(r['source_preserved'] == '1' for r in selected),
              'paired_yaw_wz_solution_difference_max': max(float(r['paired_difference']) for r in selected),
              'world_rollout_error_max': max(float(r['rollout_error']) for r in selected),
              'solver_ms': stats([float(r['solver_ms']) for r in available]),
              'future_support_overlap': sum(float(r['minimum_clearance']) < 0 for r in available)}
    if kind == 'world_hold_clear':
        for label, window in (('hold', available[:30]), ('clear', available[30:])):
            result[label] = {'wait': sum(wait(r) for r in window),
                             'forward_mps': stats([float(r['vx']) for r in window]),
                             'progress_end': float(window[-1]['progress_end']) if window else None}
    if kind.startswith('feedback_'):
        hold_cycles = 120 if kind == 'feedback_long_hold' else 30
        held = [r for r in selected if int(r['cycle']) < hold_cycles]
        held_valid = [r for r in held if valid(r)]
        clear_valid = [r for r in available if int(r['cycle']) >= hold_cycles]
        consecutive = 0
        resumed = None
        for r in [r for r in selected if int(r['cycle']) >= hold_cycles]:
            if valid(r) and float(r['vx']) > .05:
                consecutive += 1
                if consecutive == 3:
                    resumed = (int(r['cycle'])-2-hold_cycles)*.05
                    break
            else:
                consecutive = 0
        result['ideal_feedback'] = {
            'plant': 'Ideal world-velocity ZOH, independent source/body yaw .6 rad/s; no Gazebo/Nav2/MPPI execution',
            'hold_wait': sum(wait(r) for r in held_valid),
            'hold_negative_forward': sum(float(r['vx']) < -.02 for r in held_valid),
            'hold_axis_reversals': {k: reversals(held, k) for k in ('vx', 'vy')},
            'last_20_hold_speed_mps': stats([math.hypot(float(r['vx']), float(r['vy'])) for r in held_valid[-20:]]),
            'hold_mechanical_min_clearance_m': min(float(r['mechanical_clearance']) for r in held),
            'hold_mechanical_overlap': sum(float(r['mechanical_clearance']) < 0 for r in held),
            'resume_after_clear_s': resumed,
            'last_available_plant_x': float(available[-1]['plant_x']) if available else None,
            'first_unavailable_cycle': next((int(r['cycle']) for r in selected if not valid(r)), None),
            'hold_forward_mps': stats([float(r['vx']) for r in held_valid]),
            'clear_forward_mps': stats([float(r['vx']) for r in clear_valid]),
        }
    summary['controlled_probes'][kind] = result
before = json.loads((root/'experiments/r4_rotation_value/evidence/rate_coordinates/fixed_after.json').read_text())
after = json.loads((output/'fixed_after.json').read_text())
summary['legacy_fixed_baseline'] = [{'scenario': a['scenario'],
    'control_max_abs_difference': max(abs(x-y) for left,right in zip(b['controls'],a['controls']) for x,y in zip(left,right)),
    'stage_max_abs_difference': max(abs(x-y) for left,right in zip(b['stages'],a['stages']) for x,y in zip(left,right))}
    for b,a in zip(before,after)]
focused = output/'cold144/replay.csv'
if focused.exists():
    cold = rows(focused)
    warm_row = next(r for r in replay if r['scene'] == 'S2' and r['condition'] == 'observed' and r['cycle'] == '144')
    cold_row = next(r for r in cold if r['scene'] == 'S2' and r['condition'] == 'observed' and r['cycle'] == '144')
    summary['focused_warm_reset'] = {'scope': 'Only S2 cycle144 reset_warm; retain preceding proposal and source, unchanged limits/solver/weights',
                                    'original': warm_row, 'reset': cold_row,
                                    'diagnostic_prefix_samples': sum(r['condition'] == 'observed' for r in cold)}
(output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
