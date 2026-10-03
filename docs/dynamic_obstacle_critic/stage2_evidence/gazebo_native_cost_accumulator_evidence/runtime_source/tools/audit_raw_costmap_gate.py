#!/usr/bin/env python3
"""Independent sampled physical footprint/raw203 audit in the fixed simulation world.

Uses observer-available current costmaps. It does not claim exact consumer timing
or continuous-time occupancy between map/physics samples.
"""
import argparse
import bisect
import json
from pathlib import Path
import yaml
from analyze_trial import load_truth_rows
from audit_guard_rejections import current_map_clear
from trial_io import rows


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    if execution['execution'] != 'PASS':
        return {'verdict': 'FAILED', 'reason': 'infrastructure_before_goal'}
    policy = json.loads((root / 'policy.json').read_text())
    config = yaml.safe_load((root / 'profile.yaml').read_text())
    cm = config['local_costmap']['local_costmap']['ros__parameters']
    guard = yaml.safe_load((root / 'installed_inputs' / 'guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
    body = yaml.safe_load(cm['footprint']); pad = cm['footprint_padding']
    footprint = [tuple(v + (pad if v > 0 else -pad if v < 0 else 0) for v in xy) for xy in body]
    maps = sorted((r for r in rows(root, 'observations.jsonl') if r['kind'] == 'costmap'), key=lambda r: r['receive_sim'])
    arrivals = [r['receive_sim'] for r in maps]
    physical = load_truth_rows(root, execution)
    missing = 0; violations = 0; checked = 0; first = []
    for truth in physical:
        index = bisect.bisect_right(arrivals, truth['t']) - 1
        if index < 0:
            missing += 1; continue
        m = maps[index]; age = truth['t'] - m['stamp']
        if m['frame'] != cm['global_frame'] or m['frame'] != guard['world_frame'] or not 0 <= age <= guard['costmap_timeout']:
            missing += 1; continue
        checked += 1
        if not current_map_clear(m, truth['robot'], footprint, policy['raw_costmap_threshold']):
            violations += 1
            if len(first) < 5:
                first.append({'time': truth['t'], 'physical_pose': truth['robot'], 'map_stamp': m['stamp'],
                    'map_receive_sim': m['receive_sim'], 'map_age': age})
    return {'verdict': 'CONDITIONAL PASS' if checked > 0 and missing == 0 and violations == 0 else 'FAILED',
        'scope': 'sampled physical padded footprint against observer-available current raw costmap; fixed Gazebo world/odom identity only',
        'threshold': policy['raw_costmap_threshold'], 'truth_samples': len(physical), 'checked_samples': checked,
        'missing_wrong_frame_or_stale_map_samples': missing, 'padded_footprint_raw203_violations': violations,
        'first_violations': first,
        'limits': 'Observer arrival differs from guard/MPPI consumption. Requires fixed simulation world/odom identity. No continuous-time raw-grid or hardware acceptance claim; full body/padded geometry and task gates are separate.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); args = parser.parse_args()
    result = analyze(args.trial)
    (args.trial / 'raw_costmap_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
