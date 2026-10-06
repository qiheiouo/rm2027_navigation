"""A22 decision metrics; only value/feedback, no actual-output claim."""
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

root, diagnostics, value = map(Path, sys.argv[1:4])


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def conditions(selected, timed=False):
    output = {}
    for condition in dict.fromkeys(r['condition'] for r in selected):
        current = [r for r in selected if r['condition'] == condition]
        valid = [r for r in current if r['valid'] == '1']
        last = current[-1]
        position = [float(last[k]) for k in ('x', 'y')]
        if last['valid'] == '1':
            position = [p + .05 * float(last[k]) for p, k in zip(position, ('vx', 'vy'))]
        hold = {'clear': 0, 'hold30': 30, 'hold120': 120}[condition]
        held = [r for r in valid if int(r['cycle']) < hold]
        streak, resumed = 0, None
        for r in [r for r in current if int(r['cycle']) >= hold]:
            streak = streak + 1 if r['valid'] == '1' and float(r['vx']) > .05 else 0
            if streak == 3:
                resumed = (int(r['cycle']) - 2 - hold) * .05
                break
        reversal = {}
        for key in ('vx', 'vy'):
            prior, count = 0, 0
            for r in held:
                v = float(r[key]); sign = 1 if v > .02 else (-1 if v < -.02 else 0)
                if sign:
                    count += bool(prior and sign != prior); prior = sign
            reversal[key] = count
        output[condition] = {
            'cycles': len(current), 'valid': len(valid),
            'first_unavailable_cycle': next((int(r['cycle']) for r in current if r['valid'] != '1'), None),
            'last_status': last['status'], 'last_reason': last['reason'],
            'goal_distance_after_last_applied_m': math.hypot(1-position[0], position[1]),
            'goal_reached': last['valid'] == '1' and math.hypot(1-position[0], position[1]) <= .05,
            'duration_s': (int(last['cycle']) + 1) * .05,
            'hold_wait_cycles': sum(math.hypot(float(r['vx']), float(r['vy'])) < .02 for r in held),
            'hold_axis_reversals': reversal,
            'last_20_hold_speed_max_mps': max((math.hypot(float(r['vx']), float(r['vy'])) for r in held[-20:]), default=None),
            'hold_oracle_min_clearance_m': min((float(r['mechanical_clearance']) for r in held), default=None),
            'hold_oracle_overlap_samples': sum(float(r['mechanical_clearance']) < 0 for r in held),
            'resume_after_clear_s': resumed if hold else None,
            'future_soft_overlap_cycles': sum(float(r['minimum_predicted_clearance']) < 0 for r in valid),
            'iterations_max': max(int(r['iterations']) for r in current),
        }
        if all(r.get('epoch_ns') for r in current):
            epochs = [int(r['epoch_ns']) for r in current]
            output[condition]['epoch_step_50ms'] = all(b-a == 50_000_000 for a,b in zip(epochs,epochs[1:]))
            output[condition]['epoch_elapsed_ns'] = epochs[-1]-epochs[0]
        if timed:
            output[condition].update({k + '_max': max(float(r[k]) for r in current) for k in ('solver_ms', 'elapsed_ms')})
    return output


summary = {'stage': 'Research', 'baseline': '0374f4b3',
           'scope': 'Original fixtures and S1/S2 sources; ideal world ZOH only. No R4 output, MPPI comparison or physical PASS.',
           'diagnostics': {}, 'value_feedback': conditions(rows(value/'value/endpoint.csv'), True)}
for mode in ('baseline', 'zero_primal', 'interval10', 'rho_at42', 'rho_ratio15',
             'equivalent_rows', 'strict_polish', 'reduced_polish', 'reduced_refine'):
    folder = diagnostics/mode
    # First rho-only all-condition invocation predates its folder rename.
    if mode == 'rho_ratio15' and (diagnostics/'feedback/endpoint.csv').exists():
        folder = diagnostics/'feedback'
    if not (folder/'endpoint.csv').exists():
        continue
    result = conditions(rows(folder/'endpoint.csv'))
    trace = rows(folder/'trace.csv')
    result['trace'] = {'statuses': dict(Counter(r['status'] for r in trace)),
                      'pre_statuses': dict(Counter(r.get('pre_status', r['status_value']) for r in trace)),
                      'solver_rows': sorted({int(r.get('solver_rows', 168)) for r in trace}),
                      'refined_cycles': [{'condition': r['condition'], 'cycle': int(r['cycle']),
                                          'pre_status': int(r['pre_status']), 'post_status': int(r['status_value']),
                                          'polish_status': int(r['polish_status'])}
                                         for r in trace if r.get('pre_status') not in (None, '1')]}
    summary['diagnostics'][mode] = result

# The only element-wise comparison is causal: identical QP AND seed at the
# originally failed cycle, and direct verification of the midpoint identity.
import numpy as np
a = json.loads((diagnostics/'baseline/baseline_clear_42.json').read_text())
b = json.loads((diagnostics/'reduced_refine/reduced_refine_clear_42.json').read_text())
A = np.array(a['A']).reshape(168, 45); lo = np.array(a['lower']); hi = np.array(a['upper'])
x = np.array(b['x']); y = np.array(b['y']); P = np.array(b['P']).reshape(45, 45)
summary['same_qp_at_original_failure'] = {
    'max_abs_differences': {key: float(np.max(abs(np.array(a[key])-np.array(b[key]))))
                            for key in ('P', 'q', 'A', 'lower', 'upper', 'warm')},
    'midpoint_row_identity_max_error': float(max(np.max(abs(A[base+k]-(A[base+k-1]+A[base+k+1])/2))
                                                for base in (75, 106) for k in range(1, 31, 2))),
    'full_168_primal_violation': float(max(0, np.maximum(lo-A@x, A@x-hi).max())),
    'full_168_dual_stationarity_residual': float(np.max(abs(P@x+np.array(b['q'])+A.T@y))),
}
summary['native_world_replay'] = json.loads((value/'summary.json').read_text())
(value/'endpoint_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps({k: v for k, v in summary.items() if k != 'native_world_replay'}, indent=2))
