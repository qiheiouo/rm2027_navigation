#!/usr/bin/env python3
"""Offline audit of the guard's consumer-reported witness against raw map receipts."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import yaml
from analyze_trial import distance, rotation
from trial_io import rows


def local_pose(world, origin):
    x, y, _, _, z, w = origin
    angle = math.atan2(2 * w * z, 1 - 2 * z * z)
    dx, dy = world[0] - x, world[1] - y
    return (math.cos(angle) * dx + math.sin(angle) * dy,
            -math.sin(angle) * dx + math.cos(angle) * dy, world[2] - angle)


def current_map_clear(record, world_pose, footprint, threshold):
    poly = rotation(local_pose(world_pose, record['origin']), footprint)
    res = record['resolution']; width, height = record['size']
    x0 = math.floor(min(p[0] for p in poly) / res)
    y0 = math.floor(min(p[1] for p in poly) / res)
    x1 = math.floor(max(p[0] for p in poly) / res)
    y1 = math.floor(max(p[1] for p in poly) / res)
    if x0 < 0 or y0 < 0 or x1 >= width or y1 >= height:
        return False
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if record['data'][y * width + x] < threshold:
                continue
            cell = [(x * res, y * res), ((x + 1) * res, y * res),
                    ((x + 1) * res, (y + 1) * res), (x * res, (y + 1) * res)]
            if distance(poly, cell) <= 1e-9:
                return False
    return True


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    cfg = yaml.safe_load((root / 'installed_inputs' / 'guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
    footprint = list(zip(cfg['footprint'][::2], cfg['footprint'][1::2]))
    maps = {}; guards = []
    for row in rows(root, 'observations.jsonl'):
        if row['kind'] == 'costmap':
            maps[row['stamp']] = row
        if row['kind'] == 'guard' and execution['start_sim'] is not None and execution['start_sim'] <= row['receive_sim'] <= execution['last_sim']:
            guards.extend({'time': row.get('stamp', row['receive_sim']), **s} for s in row['statuses'])
    reasons = Counter(g['reason'] for g in guards)
    rejects = [g for g in guards if g['reason'] == 'static_collision_or_unknown']
    branches = Counter(); costs = Counter(); types = Counter(); times = []; speeds = []
    matched = 0; current_clear = 0; mismatches = 0; witnesses = []
    for g in rejects:
        v = g['values']; branch = int(float(v['collision_branch']))
        branches[str(branch)] += 1; costs[v['static_cell_cost']] += 1; types[v['static_rejection']] += 1
        times.append(float(v['TTC'])); speeds.append(math.hypot(float(v['proposed_vx']), float(v['proposed_vy'])))
        stamp = float(v['costmap_stamp']); m = maps.get(stamp)
        if m is None:
            continue
        matched += 1
        clear = current_map_clear(m, tuple(float(v[k]) for k in ('pose_x', 'pose_y', 'pose_yaw')), footprint, cfg['collision_threshold'])
        current_clear += clear
        x, y = int(float(v['static_cell_x'])), int(float(v['static_cell_y']))
        if x >= 0 and y >= 0:
            res = m['resolution']; cell = [(x * res, y * res), ((x + 1) * res, y * res),
                ((x + 1) * res, (y + 1) * res), (x * res, (y + 1) * res)]
            future = tuple(float(v[k]) for k in ('collision_pose_x', 'collision_pose_y', 'collision_pose_yaw'))
            gap = distance(rotation(local_pose(future, m['origin']), footprint), cell)
            reported_cost = int(float(v['static_cell_cost']))
            mismatch = m['data'][y * m['size'][0] + x] != reported_cost or abs(gap - float(v['static_cell_distance'])) > 1e-6
            mismatches += mismatch
        if len(witnesses) < 3:
            witnesses.append({'time': g['time'], 'current_footprint_raw203_clear': clear, 'values': v})
    return {'scope': 'offline association by guard-reported map source stamp; observer timing is not exact consumer timing',
        'reasons': dict(reasons), 'static_rejections': len(rejects), 'branches': dict(branches),
        'cell_costs': dict(costs), 'static_rejection_types': dict(types),
        'rejected_TTC_median': statistics.median(times) if times else None,
        'rejected_TTC_min': min(times) if times else None,
        'rejected_proposed_speed_median': statistics.median(speeds) if speeds else None,
        'matching_raw_map_receipts': matched, 'current_footprint_clear_among_matches': current_clear,
        'map_cell_or_distance_mismatches': mismatches, 'first_witnesses': witnesses,
        'limits': 'No claim of optimizer candidate identity, final SG history or sampler coverage. Cell witness is first rejecting cell, not nearest.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path)
    parser.add_argument('--timing-only', action='store_true'); args = parser.parse_args()
    if args.timing_only:
        execution = json.loads((args.trial / 'execution.json').read_text())
        counts = {'active': Counter(), 'tail': Counter()}
        if execution['start_sim'] is not None:
            for row in rows(args.trial, 'observations.jsonl'):
                if row['kind'] != 'guard': continue
                time = row.get('stamp', row['receive_sim'])
                if not execution['start_sim'] <= time <= execution['last_sim']: continue
                phase = 'active' if time < execution['end_sim'] else 'tail'
                counts[phase].update(s['reason'] for s in row['statuses'])
        result = {'scope': 'guard evaluation ROS stamp split by goal window and cancellation tail; no claim of exact action cancellation acknowledgement',
                  'reasons_by_phase': {k: dict(v) for k, v in counts.items()}}
        name = 'timing_audit.json'
    else:
        result = analyze(args.trial); name = 'guard_audit.json'
    (args.trial / name).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
