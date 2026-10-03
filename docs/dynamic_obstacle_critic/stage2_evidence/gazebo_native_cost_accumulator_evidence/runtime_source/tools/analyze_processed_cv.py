#!/usr/bin/env python3
"""Same-score-epoch physical labels for explicitly processed CV hypotheses."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from analyze_native_weights import contents
from analyze_trial import load_truth_rows
from audit_mechanical_footprint import components
from audit_raw_rollout_objective import geometry
from audit_scan_geometry import fixture
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot
from native_witness_io import records
from prepare_processed_cv import verify
from verify_raw_rollout_geometry import verify as verify_scalar


def analyze(source, witness, prepared, cpp):
    verify(source)
    verify(witness)
    identity = json.loads((prepared/'input_identity.json').read_text())
    for path, key in [(prepared/'actual_input/input.bin', 'actual_input_sha256'),
                      (prepared/'processed_paths.bin', 'processed_paths_sha256')]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != identity[key]:
            raise ValueError('processed prepared input identity')
    if (identity['source_manifest_sha256'] != hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest() or
            identity['witness_manifest_sha256'] != hashlib.sha256((witness/'manifest.json').read_bytes()).hexdigest() or
            identity['source_native_ordinal'] != 298):
        raise ValueError('registered processed source identity')
    record = list(records(witness/'selected/witnesses.bin.gz'))[0]
    paths = {name: {'values': record['blocks'][name], 'shape': [311, 30]} for name in ['x', 'y', 'yaw']}
    output = [json.loads(line) for line in cpp.read_text().splitlines()]
    header, costs = output[0], output[1:]
    if (header.get('scope') != 'offline processed CV hypotheses, not actual critic consumption' or
            header['source_ordinal'] != 298 or header['rows'] != 311 or header['steps'] != 30 or
            header['model_dt'] != identity['model_dt'] or header['actual_raw_risk_bit_exact_rows'] != 300 or
            header['actual_raw_after_bit_exact_rows'] != 300 or len(costs) != 311 or
            [r['row'] for r in costs] != list(range(311))):
        raise ValueError('explicit processed CPP output schema/context')
    for row in costs:
        if (not math.isfinite(row['risk']) or row['risk'] < 0 or
                row['minimum_clearance'] is None or not math.isfinite(row['minimum_clearance']) or
                (row['collision_time'] is not None and not 0 < row['collision_time'] <= 30*identity['model_dt'])):
            raise ValueError('processed finite cost/TTC')
    native, _ = read_snapshot(source/'native_cycles/cycle_298.json')
    consumed, _ = read_consumption(source/'dynamic_scores/score_298.json')
    margin = consumed['cost_parameters']['safety_margin']
    if consumed['score_stamp'] != identity['score_clock']:
        raise ValueError('processed actual score epoch mismatch')
    scene = fixture(source/'scene_inputs')
    shapes = components(source/'scene_inputs')
    truth = load_truth_rows(source, json.loads((source/'execution.json').read_text()))
    policy = json.loads((source/'policy.json').read_text())
    epochs = {'actual_score_clock': identity['score_clock'], 'source_pose_clock_sensitivity': identity['source_pose_clock']}
    physical = {name: geometry(native, paths, record['initial_yaw'], truth, shapes, scene, epoch, policy) for name, epoch in epochs.items()}
    original = json.loads(contents(witness/'selected/analysis.json.gz'))['records'][0]['proposals']
    max_legacy_error = 0.
    for row in range(311):
        for name in ['body', 'mechanical', 'padded_native']:
            current = physical['source_pose_clock_sensitivity']['rows'][row]['geometry'][name]
            for key, old in [('sample_min', 'dynamic_sample_min'), ('interval_lower', 'dynamic_lower')]:
                max_legacy_error = max(max_legacy_error, abs(current[key]-original[row]['geometry'][name][old]))
    if max_legacy_error > 2e-12:
        raise ValueError('source-pose geometry changed from frozen witness')
    rows = []
    for row, cost in enumerate(costs):
        label = identity['rows'][row]
        if label['row'] != row or original[row]['name'] != label['name']:
            raise ValueError('processed names do not match frozen witness')
        rows.append({'processed_row': row, 'name': label['name'], 'kind': label['kind'],
                     'CV_risk': cost['risk'], 'CV_minimum_clearance': cost['minimum_clearance'],
                     'CV_collision_time': cost['collision_time'], 'CV_model_margin_clear': cost['minimum_clearance']>margin,
                     'physical_dynamic_labels': {name: report['rows'][row] for name, report in physical.items()},
                     'original_pose_epoch_all_gates_safe_progress': original[row]['conditional_safe_control_with_progress']})
    summaries = {}
    for epoch, report in physical.items():
        summaries[epoch] = {'epoch': report['epoch'], 'horizon': report['horizon'], 'samples': report['samples'],
                            'max_sample_interval': report['max_sample_interval'],
                            'physical_dynamic_clear_rows': sum(r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates'] for r in rows),
                            'model_clear_but_dynamic_geometry_failed_rows': sum(r['CV_model_margin_clear'] and not r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates'] for r in rows),
                            'padded_contact_on_processed_rollout_grid_rows': sum(r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in rows),
                            'model_clear_but_padded_grid_contact_rows': sum(r['CV_model_margin_clear'] and r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in rows)}
    report = {'schema': 1, 'native_source_ordinal': 298, 'input_identity_sha256': hashlib.sha256((prepared/'input_identity.json').read_bytes()).hexdigest(),
              'cpp_output_sha256': hashlib.sha256(cpp.read_bytes()).hexdigest(), 'processed_rows': 311,
              'model_dt': identity['model_dt'], 'model_margin': margin,
              'score_minus_source_pose_seconds': identity['score_clock']-identity['source_pose_clock'],
              'legacy_pose_epoch_dynamic_geometry_max_error': max_legacy_error,
              'epoch_summaries': summaries, 'registered_proposals': rows[:11], 'rows': rows,
              'scope': 'Verified reconstructed SDK aggregate and conditional processed paths, exact consumed CV score parameters; dynamic fixture labels only.',
              'limits': ['Model costs are new offline evaluations, not actual critic score events or native post-SG costs.',
                         'Counterfactual rows do not have new optimizer weights; they are not executed or re-ranked.',
                         'Original all-gate labels use the source-pose epoch. New labels are dynamic-only and retain the actual 24ms score epoch difference.',
                         'No static/raw203, final bounds, execution or goal certificate. Existing FAILED/NOT ESTABLISHED remains.']}
    scalar = verify_scalar(source, report, paths, 'processed_row', 0, 'padded_contact_on_processed_rollout_grid_rows')
    return report, scalar


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'witness_archive', 'prepared', 'cpp_output', 'output', 'scalar_output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report, scalar = analyze(args.source.resolve(), args.witness_archive.resolve(), args.prepared, args.cpp_output)
    for path, result in [(args.output, report), (args.scalar_output, scalar)]:
        with path.open('x') as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['rows', 'registered_proposals']}), flush=True)
    print(json.dumps({'registered': [{k:r[k] for k in ['processed_row','name','CV_risk','CV_minimum_clearance','CV_collision_time']} for r in report['registered_proposals']]}), flush=True)
    print(json.dumps(scalar), flush=True)
