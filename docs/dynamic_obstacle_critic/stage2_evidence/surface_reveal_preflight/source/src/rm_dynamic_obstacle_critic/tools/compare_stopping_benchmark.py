#!/usr/bin/env python3
"""Compare complete native cost vectors and before/after/before microtimings."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def read(path):
    records = [json.loads(line) for line in path.read_text().splitlines()]
    identity = records.pop(0)
    if identity['kind'] != 'library_mapping' or identity['footprint_padding'] != 0:
        raise ValueError('benchmark identity/footprint missing')
    cases = {}
    for r in records:
        key = tuple(r[k] for k in ('mode', 'map_case', 'rotation', 'motion'))
        if key in cases or r['seed'] != 712824 or r['batch'] != 300 or r['steps'] != 30:
            raise ValueError('benchmark case/grid/seed mismatch')
        if len(r['costs']) != 300 or len(r['score_ms']) != 7 or not all(
                math.isfinite(v) and v >= 0 for v in r['costs'] + r['score_ms']):
            raise ValueError('benchmark incomplete or nonfinite result')
        cases[key] = r
    expected = {(m, g, r, v) for m in range(3) for g in range(4)
                for r in range(2) for v in range(3)}
    if set(cases) != expected:
        raise ValueError('benchmark case coverage incomplete')
    return identity, cases


def compare(before_a, after, before_b):
    ia, a = read(before_a); io, optimized = read(after); ib, b = read(before_b)
    if ia != ib or ia['path'] == io['path'] or ia['footprint'] != io['footprint']:
        raise ValueError('before/after loader or footprint identity mismatch')
    differences = []
    per_case = []
    for key in sorted(a):
        old, current, repeated = a[key], optimized[key], b[key]
        for i, (x, y, z) in enumerate(zip(old['costs'], current['costs'], repeated['costs'])):
            if x != y or x != z:
                differences.append({'case': key, 'index': i, 'before_a': x, 'after': y, 'before_b': z})
        old_ms = statistics.median(old['score_ms'] + repeated['score_ms'])
        new_ms = statistics.median(current['score_ms'])
        per_case.append({'case': key, 'before_median_ms': old_ms, 'after_median_ms': new_ms,
                         'median_speedup': old_ms / new_ms})
    timing_groups = {}
    for name, keys in [('all', list(a)), ('soft_all', [k for k in a if k[0] == 2]),
                       ('soft_nonempty_masks', [k for k in a if k[0] == 2 and k[1] in (1, 2)])]:
        ta, to, tb = (sum(sum(cases[k]['score_ms']) for k in keys)
                      for cases in (a, optimized, b))
        timing_groups[name] = {'cases': len(keys), 'seven_repeat_total_before_a_ms': ta,
            'seven_repeat_total_after_ms': to, 'seven_repeat_total_before_b_ms': tb,
            'aggregate_speedup_against_mean_before': (ta + tb) / (2 * to),
            'baseline_b_over_a': tb / ta}
    return {'scope': 'fixed synthetic native stopping-plugin microbenchmark; no historical candidate, SG or physical safety evidence',
        'verdict': 'PASS' if not differences else 'FAILED', 'cases': len(a),
        'unique_cost_values_compared': len(a) * 300, 'exact_vector_differences': differences,
        'library_mapping': {'before': ia, 'after': io}, 'timing_groups': timing_groups,
        'per_case': per_case, 'input_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                             for p in (before_a, after, before_b)},
        'limits': 'Identical executable, clipping inputs and parameters; old/new/old process order. Timings include native critic call, not sensor, optimizer, SG, smoother or DDS chain. Every fixture retained. Whole-chain deadline and physics must be checked independently.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('before_a', type=Path); parser.add_argument('after', type=Path)
    parser.add_argument('before_b', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = compare(args.before_a, args.after, args.before_b)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'per_case'}))
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
