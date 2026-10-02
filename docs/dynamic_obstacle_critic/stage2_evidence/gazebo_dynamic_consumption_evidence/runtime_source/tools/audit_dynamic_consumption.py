#!/usr/bin/env python3
"""Join actual dynamic scores to native batches by complete exact identities."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import yaml
from dynamic_consumption_io import read_consumption, cost_relation_exact
from native_snapshot_io import read_snapshot, COST

ORIGINAL = ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']


def identity(meta, blocks):
    # A hash indexes identities; matches still compare every source byte below.
    raw = b''.join(blocks[n]['bytes'] for n in ('x', 'y', 'yaw'))
    raw += json.dumps([blocks[n]['shape'] for n in ('x', 'y', 'yaw')], separators=(',', ':')).encode()
    raw += struct.pack('<14d', *meta['pose'], *meta['speed'], meta['model_dt'])
    raw += json.dumps([meta[n] for n in ('pose_stamp_sec', 'pose_stamp_nanosec',
        'pose_frame', 'evaluation_frame', 'base_frame')], separators=(',', ':')).encode()
    raw += b''.join(struct.pack('<2d', *p) for p in meta['padded_footprint'])
    return hashlib.sha256(raw).hexdigest(), raw


def audit(trial, replay):
    follow = yaml.safe_load((trial/'profile.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    if follow['critics'] != ORIGINAL+['DynamicObstacleCritic', 'StaticStoppingCritic', 'NativeCycleSnapshotCritic']:
        raise ValueError('dynamic subtotal attribution requires exact registered critic order')
    native = {}; native_count = 0; native_ordinals = set()
    for path in sorted((trial/'native_cycles').glob('cycle_*.json'), key=lambda p: int(p.stem.split('_')[-1])):
        meta, blocks = read_snapshot(path); key, raw = identity(meta, blocks)
        if meta['ordinal'] in native_ordinals or meta['ordinal'] != int(path.stem.split('_')[-1]):
            raise ValueError('native score identity ordering')
        native_ordinals.add(meta['ordinal'])
        native.setdefault(key, []).append((meta, blocks, raw)); native_count += 1
    if not 1 <= native_count <= 1000: raise ValueError('native batch budget')
    results = [json.loads(line) for line in replay.read_text().splitlines()]
    replays = {r['ordinal']: r for r in results}
    if len(replays) != len(results): raise ValueError('duplicate risk replay ordinal')
    joined = []; unmatched = []; ambiguous = []; matched_native = set()
    scores = sorted((trial/'dynamic_scores').glob('score_*.json'), key=lambda p: int(p.stem.split('_')[-1]))
    if not 1 <= len(scores) <= 1000: raise ValueError('dynamic score count budget')
    previous = -1
    for path in scores:
        meta, blocks = read_consumption(path); ordinal = meta['ordinal']; batch, steps = blocks['x']['shape']
        if ordinal <= previous or ordinal != int(path.stem.split('_')[-1]): raise ValueError('dynamic score identity ordering')
        previous = ordinal; result = replays.get(ordinal)
        if (not cost_relation_exact(blocks) or result is None or result['batch'] != batch or result['steps'] != steps
                or result['risk_double_bit_exact_rows'] != batch or result['after_float_bit_exact_rows'] != batch
                or len(result['risk']) != batch or any(not math.isfinite(v) for v in result['risk'])
                or b''.join(struct.pack('<d', v) for v in result['risk']) != blocks['dynamic_risk_double']['bytes']):
            raise ValueError('exact dynamic replay/float cost relation failed')
        for field in ('minimum_clearance', 'collision_time'):
            if len(result[field]) != batch or any(v is not None and not math.isfinite(v) for v in result[field]):
                raise ValueError('risk replay finite geometry metadata')
        key, raw = identity(meta, blocks); matches = [entry for entry in native.get(key, []) if entry[2] == raw]
        if not matches: unmatched.append(ordinal); continue
        if len(matches) != 1: ambiguous.append(ordinal); continue
        nmeta, nblocks, _ = matches[0]
        if nmeta['ordinal'] in matched_native: raise ValueError('two scores matched one native batch')
        matched_native.add(nmeta['ordinal'])
        before = blocks['costs_before_dynamic']['values']; risk = blocks['dynamic_risk_double']['values']
        after = blocks['costs_after_dynamic']['values']; total = nblocks[COST]['values']
        if len(total) != batch or nmeta['fail_flag'] is not False: raise ValueError('native successful cost batch required')
        residual = [float(t)-float(a) for t, a in zip(total, after)]
        if any(value < 0 for value in residual): raise ValueError('negative rounded static-stage increment')
        joined.append({'score_ordinal': ordinal, 'native_ordinal': nmeta['ordinal'],
            'score_stamp': meta['score_stamp'], 'capture_stamp': nmeta['capture_stamp'],
            'pose_stamp': meta['pose_stamp_sec']+meta['pose_stamp_nanosec']*1e-9,
            'score_to_snapshot_clock_difference': nmeta['capture_stamp']-meta['score_stamp'],
            'source_age_used': meta['source_age_used'], 'consumed_source_stamp': meta['input_used']['stamp_sec']+meta['input_used']['stamp_nanosec']*1e-9,
            'track_count': len(meta['input_used']['tracks']), 'identity_sha256': key,
            'risk_double_bit_exact_rows': batch, 'after_float_bit_exact_rows': batch,
            'original_seven_subtotal_range': [min(before), max(before)],
            'actual_dynamic_double_risk_range': [min(risk), max(risk)],
            'rounded_static_stage_increment_range': [min(residual), max(residual)],
            'native_pre_gamma_total_range': [min(total), max(total)],
            'raw_candidates_with_predicted_margin_violation': sum(v is not None and v <= meta['cost_parameters']['safety_margin'] for v in result['minimum_clearance'])})
    if set(replays) != {int(p.stem.split('_')[-1]) for p in scores}: raise ValueError('risk replay coverage differs from recorded scores')
    complete = not unmatched and not ambiguous and len(matched_native) == native_count
    return {'schema': 1, 'verdict': 'EXACT CONSUMPTION/NATIVE JOIN PASS' if complete else 'INCOMPLETE EXACT CONSUMPTION/NATIVE JOIN',
        'native_batches': native_count, 'dynamic_scores': len(scores), 'exact_joined_batches': len(joined),
        'unmatched_scores': unmatched, 'ambiguous_scores': ambiguous,
        'unmatched_native_ordinals': sorted(native_ordinals-matched_native), 'cycles': joined,
        'scope': 'Exact used CV fields/risk and source-pose/full-rollout match. Original seven subtotal and rounded static-stage increment; no individual upstream critic attribution.',
        'limits': ['Unconsumed fields and TF/map/odometry subscriber source stamps unavailable.',
            'Post-dynamic and final native float difference includes rounding, not exact StaticStopping double risk.',
            'Raw rollout score labels differ from individually constrained/SG counterfactual and weighted output.',
            'Exact replay does not certify perception geometry, future motion or physical acceptance.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); parser.add_argument('native_risk_replay', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); report = audit(args.trial, args.native_risk_replay)
    with args.output.open('x') as stream: stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ('verdict', 'native_batches', 'dynamic_scores', 'exact_joined_batches')}))
