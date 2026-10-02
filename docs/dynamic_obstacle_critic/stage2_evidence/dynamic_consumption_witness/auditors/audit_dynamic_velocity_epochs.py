#!/usr/bin/env python3
"""Describe score and snapshot epochs without relaxing the frozen age gate."""
import argparse
import json
from pathlib import Path
import struct
import yaml
from trial_io import rows


def audit(root):
    join = json.loads((root/'dynamic_consumption_audit.json').read_text())
    if join['verdict'] != 'EXACT CONSUMPTION/NATIVE JOIN PASS': raise ValueError('exact dynamic/native identity required')
    values = json.loads((root/'effective_velocity_audit.json').read_text())
    follow = yaml.safe_load((root/'profile.yaml').read_text())['controller_server']['ros__parameters']
    thresholds = [follow[k] for k in ('min_x_velocity_threshold', 'min_y_velocity_threshold', 'min_theta_velocity_threshold')]
    def speed(v): return [x if abs(x) > t else 0. for x, t in zip(v, thresholds)]
    def bits(v): return struct.pack('<3d', *v)
    canonical = [r for r in rows(root, 'observations.jsonl') if r['kind'] == 'canonical_odom']
    limit = values['age_limit_from_frozen_guard']; records = []
    for row in join['cycles']:
        measured = values['records'][row['native_ordinal']]
        target = bits([measured['native_speed'][i] for i in (0, 1, 5)])
        score_matches = [r['stamp'] for r in canonical if 0 <= row['score_stamp']-r['stamp'] <= limit and bits(speed(r['velocity'])) == target]
        pose_matches = [r['stamp'] for r in canonical if r['stamp'] == row['pose_stamp'] and bits(speed(r['velocity'])) == target]
        records.append({'native_ordinal': row['native_ordinal'], 'score_ordinal': row['score_ordinal'],
            'pose_stamp': row['pose_stamp'], 'score_stamp': row['score_stamp'], 'capture_stamp': row['capture_stamp'],
            'score_pose_age': row['score_stamp']-row['pose_stamp'],
            'capture_pose_age': row['capture_stamp']-row['pose_stamp'],
            'score_matches_in_original_age_window': score_matches, 'exact_value_at_source_pose': bool(pose_matches),
            'snapshot_matches_in_original_age_window': measured['matching_source_stamps'], 'actual_consumer_source_stamp': None})
    return {'schema': 1, 'scope': 'Descriptive epoch audit only; original snapshot age verdict stays unchanged.',
        'frozen_age_limit': limit, 'snapshot_verdict': values['verdict'], 'cycles': len(records),
        'exact_at_source_pose_cycles': sum(r['exact_value_at_source_pose'] for r in records),
        'score_clock_matching_cycles': sum(bool(r['score_matches_in_original_age_window']) for r in records),
        'snapshot_clock_matching_cycles': sum(bool(r['snapshot_matches_in_original_age_window']) for r in records),
        'snapshot_failed_ordinals': [r['native_ordinal'] for r in records if not r['snapshot_matches_in_original_age_window']],
        'records': records, 'limits': 'Matching public values does not identify the subscriber callback stamp. Score and final snapshot clocks differ across computation; no age tolerance change or physical execution certificate.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); report = audit(args.trial)
    with args.output.open('x') as stream: stream.write(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}))
