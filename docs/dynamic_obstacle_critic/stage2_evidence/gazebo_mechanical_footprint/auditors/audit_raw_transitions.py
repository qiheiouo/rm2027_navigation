#!/usr/bin/env python3
"""Counterfactual same-pose raw-map transition audit; offline fixed-world labels only."""
import argparse
import bisect
import json
import math
from pathlib import Path
import yaml
from analyze_trial import distance, load_truth_rows, rotation, travel
from audit_guard_rejections import current_map_clear, local_pose
from trial_io import rows


def interpolate(physical, times, t):
    i = bisect.bisect_right(times, t)
    if i == 0 or i == len(times):
        return None
    a, b = physical[i - 1], physical[i]
    u = (t - a['t']) / (b['t'] - a['t'])
    p, q = a['robot'], b['robot']
    return (p[0] + u * (q[0] - p[0]), p[1] + u * (q[1] - p[1]),
            p[2] + u * math.remainder(q[2] - p[2], 2 * math.pi))


def hits(m, pose, footprint):
    poly = rotation(local_pose(pose, m['origin']), footprint)
    res = m['resolution']; width, height = m['size']; result = []
    x0 = max(0, math.floor(min(x for x, y in poly) / res))
    x1 = min(width - 1, math.floor(max(x for x, y in poly) / res))
    y0 = max(0, math.floor(min(y for x, y in poly) / res))
    y1 = min(height - 1, math.floor(max(y for x, y in poly) / res))
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            cost = m['data'][y * width + x]
            if cost < 203: continue
            box = [(x * res, y * res), ((x + 1) * res, y * res),
                   ((x + 1) * res, (y + 1) * res), (x * res, (y + 1) * res)]
            if distance(poly, box) <= 1e-9:
                result.append({'cell': [x, y], 'cost': cost})
    return result


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    if execution['execution'] != 'PASS':
        return {'scope': 'infrastructure_before_goal', 'map_introduced_transitions': []}
    cfg = yaml.safe_load((root / 'installed_inputs/guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
    footprint = list(zip(cfg['footprint'][::2], cfg['footprint'][1::2]))
    radius = max(math.hypot(*p) for p in footprint)
    physical = load_truth_rows(root, execution); times = [r['t'] for r in physical]
    maps = sorted((r for r in rows(root, 'observations.jsonl') if r['kind'] == 'costmap'), key=lambda r: r['receive_sim'])
    transitions = []
    for a, b in zip(maps, maps[1:]):
        old = interpolate(physical, times, a['receive_sim'])
        new = interpolate(physical, times, b['receive_sim'])
        if old is None or new is None or a['frame'] != cfg['world_frame'] or b['frame'] != cfg['world_frame']: continue
        if not current_map_clear(a, new, footprint, 203) or current_map_clear(b, new, footprint, 203): continue
        between = [r['robot'] for r in physical if a['receive_sim'] <= r['t'] <= b['receive_sim']]
        deviation = max([travel(old, new, radius)] + [travel(old, p, radius) for p in between])
        hit = hits(b, new, footprint)
        aligned = a['origin'] == b['origin'] and a['size'] == b['size'] and a['resolution'] == b['resolution']
        if aligned:
            for h in hit:
                x, y = h['cell']; h['previous_cost_same_cell'] = a['data'][y * a['size'][0] + x]
        transitions.append({'old_stamp': a['stamp'], 'new_stamp': b['stamp'], 'new_receive_sim': b['receive_sim'],
            'physical_pose': new, 'max_physical_pose_deviation_m': deviation, 'stationary_within_1um': deviation <= 1e-6,
            'same_grid_geometry': aligned, 'old_origin': a['origin'], 'new_origin': b['origin'], 'intersecting_cells': hit})
    return {'scope': 'same physical pose against before/after observer-received raw maps; source of marking not inferred',
        'map_introduced_transition_count': len(transitions),
        'stationary_map_introduced_count': sum(t['stationary_within_1um'] for t in transitions),
        'map_introduced_transitions': transitions,
        'limits': 'Physical poses use linear interpolation; stationary classification also checks every intermediate recorded truth pose. No exact Nav2 scan-consumer identity, Gaussian-noise attribution or continuous-time guarantee.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); result = analyze(args.trial)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'map_introduced_transitions'}))


if __name__ == '__main__': main()
