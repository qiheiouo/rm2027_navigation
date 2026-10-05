"""Bounded A19/A18 comparison; recorded output receipts are only a proxy."""
import bisect
import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

root, output = map(Path, sys.argv[1:3])
old = root / 'experiments/r4_rotation_value/evidence'


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    return {'n': len(values), 'median': statistics.median(values),
            'p95': values[max(0, math.ceil(.95*len(values))-1)], 'max': values[-1]}


def comparison(current, baseline, keys):
    indexed = {(r['case'] if 'case' in r else r['scene'], r['cycle']): r for r in baseline}
    differences = {k: [] for k in keys}
    valid_match = 0
    for r in current:
        identity = (r['case'] if 'case' in r else r['scene'], r['cycle'])
        if identity not in indexed:
            continue
        b = indexed[identity]
        valid_match += r['valid'] == b['valid']
        if r['valid'] == b['valid'] == '1':
            for k in keys:
                differences[k].append(abs(float(r[k])-float(b[k])))
    return {'valid_match': valid_match, 'absolute_difference': {k: stats(v) for k, v in differences.items()}}


replay = rows(output / 'replay.csv')
probes = rows(output / 'probe.csv')
summary = {'stage': 'Research', 'scope': 'finite recorded values and held-source virtual probes; no actual output'}
summary['recorded'] = {}
for scene in ('S1', 'S2'):
    selected = [r for r in replay if r['scene'] == scene]
    valid = [r for r in selected if r['valid'] == '1']
    summary['recorded'][scene] = {
        'samples': len(selected), 'valid': len(valid), 'reasons': dict(Counter(r['reason'] for r in selected)),
        'solver_ms': stats([float(r['solver_ms']) for r in valid]),
        'elapsed_ms': stats([float(r['elapsed_ms']) for r in valid]),
        'warm': sum(r['used_warm'] == '1' for r in valid),
        'nominal_nonzero': sum(float(r['nominal_cost']) > 1e-8 for r in valid),
        'dynamic_nonzero': sum(float(r['solved_cost']) > 1e-8 for r in valid),
        'predicted_overlap_samples': sum(float(r['minimum_clearance']) < 0 for r in valid),
        'cost_increased': sum(float(r['nominal_cost']) > 1e-8 and float(r['solved_cost']) > float(r['nominal_cost'])+1e-4 for r in valid),
        'comparison': comparison(selected, [r for r in rows(old / 'locked_dynamic/replay.csv') if r['scene'] == scene],
                                 ('vx', 'vy', 'wz', 'progress_end', 'yaw_delta', 'nominal_cost', 'solved_cost', 'minimum_clearance')),
    }
summary['probes'] = {}
for kind in dict.fromkeys(r['case'] for r in probes):
    selected = [r for r in probes if r['case'] == kind]
    valid = [r for r in selected if r['valid'] == '1']
    result = {'samples': len(selected), 'valid': len(valid), 'reasons': dict(Counter(r['reason'] for r in selected)),
              'source_preserved': sum(r['source_preserved'] == '1' for r in selected),
              'legacy_accepts': sum(r['legacy_accepts'] == '1' for r in selected),
              'solver_ms': stats([float(r['solver_ms']) for r in valid]),
              'slice_error_max': max(float(r['slice_error']) for r in selected),
              'predicted_overlap_samples': sum(float(r['minimum_clearance']) < 0 for r in valid)}
    if kind.startswith('locked_future'):
        result['comparison'] = comparison(selected, rows(old / 'locked_future/probe.csv'),
            ('vx', 'vy', 'wz', 'progress_end', 'yaw_end', 'nominal_cost', 'solved_cost', 'minimum_clearance'))
    if kind == 'locked_future_hold_clear':
        for label, window in (('hold', valid[:30]), ('clear', valid[30:])):
            result[label] = {'world_forward': stats([math.cos(float(r['yaw_end']))*float(r['vx'])-
                math.sin(float(r['yaw_end']))*float(r['vy']) for r in window]),
                'final_virtual_progress': float(window[-1]['progress_end']) if window else None}
    summary['probes'][kind] = result

before = json.loads((old / 'rate_coordinates/fixed_after.json').read_text())
after = json.loads((output / 'fixed_after.json').read_text())
assert [r['scenario'] for r in before] == [r['scenario'] for r in after]
summary['fixed_baseline'] = []
for b, a in zip(before, after):
    summary['fixed_baseline'].append({'scenario': a['scenario'],
        'control_max_abs_difference': max(abs(x-y) for left, right in zip(b['controls'], a['controls']) for x, y in zip(left, right)),
        'stage_max_abs_difference': max(abs(x-y) for left, right in zip(b['stages'], a['stages']) for x, y in zip(left, right)),
        'dynamic_cost_abs_difference': abs(b['solved_dynamic_cost']-a['solved_dynamic_cost'])})

# This read-only receipt proxy is never a solver seed and proves no owner send.
summary['actual_receipt_proxy'] = {}
for scene in ('S1', 'S2'):
    commands = sorted([r for r in rows(root / f'experiments/r4_corrected_runtime_shadow/evidence/{scene}_references.csv')
                       if r['kind'] == 'actual_output'], key=lambda r: int(r['receipt_ros_ns']))
    stamps = [int(r['receipt_ros_ns']) for r in commands]
    counts = Counter()
    ages, angular = [], []
    for r in replay:
        if r['scene'] != scene:
            continue
        epoch = int(r['epoch_ns'])
        index = bisect.bisect_right(stamps, epoch)-1
        if index < 0:
            counts['missing'] += 1
            continue
        command = commands[index]
        age = (epoch-stamps[index])*1e-6
        wz = float(command['wz'])
        ages.append(age)
        angular.append(abs(wz))
        if age > 100:
            counts['stale_over_100ms'] += 1
        elif wz != 0:
            counts['fresh_nonzero_wz'] += 1
        else:
            counts['fresh_exact_zero_wz'] += 1
    summary['actual_receipt_proxy'][scene] = {'classification': dict(counts), 'age_ms': stats(ages),
        'absolute_wz': stats(angular), 'scope': 'latest original actual_output receipt at/before epoch; no send/last-applied grant'}
(output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
