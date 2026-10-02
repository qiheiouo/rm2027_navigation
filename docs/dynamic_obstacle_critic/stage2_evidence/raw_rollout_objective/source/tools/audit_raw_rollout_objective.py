#!/usr/bin/env python3
"""Exact raw CV model costs versus same-path conditional physical geometry."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import tempfile
import numpy as np
from analyze_trial import load_truth_rows
from audit_dynamic_consumption import audit as audit_consumption
from audit_mechanical_footprint import components
from audit_scan_geometry import fixture
from analyze_native_weights import analyze as audit_weights, contents
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot
from native_witness_io import records as witness_records
from witness_geometry import rotate, polygon_distance, circle_box_gap, interpolation_lower


def verify(root):
    for name, digest in json.loads((root/'manifest.json').read_text())['files'].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('frozen identity mismatch: '+name)


def geometry(native, blocks, initial_yaw, truth, shapes, scene, epoch, policy):
    count, steps = blocks['x']['shape']
    dt = native['model_dt']
    horizon = steps*dt
    times = np.array([r['t'] for r in truth])
    if epoch < times[0] or epoch+horizon > times[-1]:
        raise ValueError('raw full-horizon truth bracket missing')
    grid = epoch+np.arange(steps+1)*dt
    samples = np.unique(np.concatenate((grid, times[(times>epoch)&(times<epoch+horizon)])))
    poses = np.empty((count, len(samples), 3))
    discrete = np.empty((count, steps, 3))
    for axis, name in enumerate(['x', 'y', 'yaw']):
        values = np.asarray(blocks[name]['values'], dtype=np.float32).reshape(count, steps).astype(float)
        discrete[:, :, axis] = values
        initial = native['pose'][axis] if axis < 2 else initial_yaw
        values = np.column_stack((np.full(count, initial), values))
        if axis == 2:
            values = np.unwrap(values, axis=1)
        for row in range(count):
            poses[row, :, axis] = np.interp(samples, grid, values[row])
    actor_truth = np.array([r['obstacle'] for r in truth])
    actor_truth[:, 2] = np.unwrap(actor_truth[:, 2])
    def actor_at(query):
        return np.stack([np.interp(query, times, actor_truth[:, axis]) for axis in range(3)], axis=-1)
    dimensions = scene['actor_dimensions']
    actor = np.array([[-dimensions[0]/2, -dimensions[1]/2], [dimensions[0]/2, -dimensions[1]/2],
                      [dimensions[0]/2, dimensions[1]/2], [-dimensions[0]/2, dimensions[1]/2]])
    actor_radius = np.linalg.norm(actor, axis=1).max()
    obstacle = actor_at(samples)
    actor_polygons = rotate(obstacle, actor)
    discrete_obstacle = actor_at(grid[1:])
    body = np.array(next(s['points'] for s in shapes if s['kind']=='polygon' and s['name']=='base_link/base_collision'))
    metrics = {}
    for name, points in [('body', body), ('padded_native', np.array(native['padded_footprint']))]:
        gap = polygon_distance(rotate(poses, points), actor_polygons)
        discrete_gap = polygon_distance(rotate(discrete, points), rotate(discrete_obstacle, actor))
        radius = np.linalg.norm(points, axis=1).max()
        metrics[name] = {'sample_min': gap.min(axis=1),
                         'interval_lower': interpolation_lower(gap, poses, radius, obstacle, actor_radius),
                         'original_rollout_grid_min': discrete_gap.min(axis=1)}
        if name == 'body':
            mechanical_gap = gap.copy()
            mechanical_discrete = discrete_gap.copy()
            mechanical_radius = radius
    for shape in shapes:
        if shape['kind'] != 'circle':
            continue
        center = np.array([shape['center']])
        mechanical_radius = max(mechanical_radius, np.linalg.norm(center)+shape['radius'])
        mechanical_gap = np.minimum(mechanical_gap, circle_box_gap(rotate(poses, center)[..., 0, :], shape['radius'], obstacle, dimensions))
        mechanical_discrete = np.minimum(mechanical_discrete, circle_box_gap(rotate(discrete, center)[..., 0, :], shape['radius'], discrete_obstacle, dimensions))
    metrics['mechanical'] = {'sample_min': mechanical_gap.min(axis=1),
                            'interval_lower': interpolation_lower(mechanical_gap, poses, mechanical_radius, obstacle, actor_radius),
                            'original_rollout_grid_min': mechanical_discrete.min(axis=1)}
    result = []
    for row in range(count):
        row_metrics = {name: {field: float(values[row]) for field, values in fields.items()} for name, fields in metrics.items()}
        gates = {'body_dynamic_margin': row_metrics['body']['interval_lower'] >= policy['body_clearance'],
                 'mechanical_dynamic_margin': row_metrics['mechanical']['interval_lower'] >= policy['body_clearance'],
                 'padded_dynamic_no_contact': row_metrics['padded_native']['interval_lower'] > 0}
        result.append({'gates': gates, 'all_dynamic_geometry_gates': all(gates.values()), 'geometry': row_metrics})
    return {'epoch': epoch, 'horizon': horizon, 'samples': len(samples),
            'max_sample_interval': float(np.diff(samples).max()), 'rows': result}


def analyze(source, witness_archive):
    verify(source)
    verify(witness_archive)
    provenance = json.loads((witness_archive/'provenance.json').read_text())
    if (provenance['selected_source_ordinal'] != 298 or provenance['source_manifest_sha256'] !=
            hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()):
        raise ValueError('registered source298/source manifest required')
    risk_path = source/'dynamic_replay/native_risk.jsonl.gz'
    with tempfile.TemporaryDirectory(prefix='raw_rollout_risk_') as temporary:
        replay = Path(temporary)/'risk.jsonl'
        replay.write_bytes(gzip.open(risk_path, 'rb').read())
        join = audit_consumption(source, replay)
        risk_rows = [json.loads(line) for line in replay.read_text().splitlines()]
    if (join['verdict'] != 'EXACT CONSUMPTION/NATIVE JOIN PASS' or
            join != json.loads((source/'dynamic_consumption_audit.json').read_text())):
        raise ValueError('full actual consumption join does not reproduce frozen proof')
    selected = witness_archive/'selected/analysis.json.gz'
    weights_path = source/'native_optimizer/weights_output.jsonl.gz'
    weight_proof = audit_weights(weights_path, source/'native_optimizer/sdk_final_output.jsonl.gz', selected)
    if weight_proof['verdict'] != 'NATIVE WEIGHTS REAGGREGATION EXACT':
        raise ValueError('actual native weights required')
    native, blocks = read_snapshot(source/'native_cycles/cycle_298.json')
    matched = next(r for r in join['cycles'] if r['native_ordinal']==298)
    consumed, _ = read_consumption(source/'dynamic_scores'/f"score_{matched['score_ordinal']}.json")
    if consumed['world_transform'] != [0, 0, 0] or consumed['evaluation_frame'] != 'odom':
        raise ValueError('this frozen truth audit requires identity world alignment')
    record = list(witness_records(witness_archive/'selected/witnesses.bin.gz'))
    if (len(record) != 1 or not record[0]['raw_velocity_pose_exact'] or
            not record[0]['aggregate_SG_exact'] or not record[0]['actual_command_double_bit_exact']):
        raise ValueError('verified SDK source pose context required')
    if blocks['x']['shape'] != [300, 30]:
        raise ValueError('registered original 300x30 raw batch required')
    policy = json.loads((source/'policy.json').read_text())
    truth = load_truth_rows(source, json.loads((source/'execution.json').read_text()))
    shapes = components(source/'scene_inputs')
    scene = fixture(source/'scene_inputs')
    epochs = {'actual_score_clock': consumed['score_stamp'], 'source_pose_clock_sensitivity': matched['pose_stamp']}
    labels = {name: geometry(native, blocks, record[0]['initial_yaw'], truth, shapes, scene, epoch, policy)
              for name, epoch in epochs.items()}
    risk = next(r for r in risk_rows if r['ordinal']==matched['score_ordinal'])
    weight_row = [json.loads(line) for line in contents(weights_path).splitlines()][299]
    if weight_row['ordinal'] != 298:
        raise ValueError('selected native weight ordinal')
    weights = weight_row['native_softmax_weights']
    if any(v is None for v in risk['minimum_clearance']):
        raise ValueError('selected model risk must contain usable obstacles')
    results = []
    margin = consumed['cost_parameters']['safety_margin']
    for row in range(300):
        results.append({'raw_native_row': row, 'actual_dynamic_risk': risk['risk'][row],
                        'actual_model_minimum_clearance': risk['minimum_clearance'][row],
                        'actual_model_margin_clear': risk['minimum_clearance'][row] > margin,
                        'actual_model_collision_time': risk['collision_time'][row], 'native_weight': weights[row],
                        'physical_dynamic_labels': {name: report['rows'][row] for name, report in labels.items()}})
    summaries = {}
    for epoch in labels:
        cells = {}
        for model_clear in [False, True]:
            for physical_clear in [False, True]:
                group = [r for r in results if r['actual_model_margin_clear']==model_clear and
                         r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates']==physical_clear]
                name = f'model_margin_{model_clear}_physical_dynamic_{physical_clear}'
                cells[name] = {'rows': len(group), 'native_weight_mass': math.fsum(r['native_weight'] for r in group)}
        summaries[epoch] = {'epoch': labels[epoch]['epoch'], 'horizon': labels[epoch]['horizon'],
                            'samples': labels[epoch]['samples'], 'max_sample_interval': labels[epoch]['max_sample_interval'],
                            'contingency': cells,
                            'physical_dynamic_clear_rows': sum(r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates'] for r in results),
                            'padded_contact_on_original_rollout_grid_rows': sum(r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in results),
                            'model_clear_but_padded_original_grid_contact_rows': sum(r['actual_model_margin_clear'] and r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in results),
                            'model_clear_but_padded_original_grid_contact_weight_mass': math.fsum(r['native_weight'] for r in results if r['actual_model_margin_clear'] and r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0)}
    top = max(results, key=lambda r: r['native_weight'])
    return {'schema': 1, 'native_source_ordinal': 298, 'actual_consumption_ordinal': matched['score_ordinal'],
            'source_manifest_sha256': provenance['source_manifest_sha256'],
            'witness_manifest_sha256': hashlib.sha256((witness_archive/'manifest.json').read_bytes()).hexdigest(),
            'full_consumption_join': join['verdict'], 'full_native_weights': weight_proof['verdict'],
            'raw_candidates': 300, 'score_minus_source_pose_seconds': epochs['actual_score_clock']-epochs['source_pose_clock_sensitivity'],
            'model_margin': margin, 'model_margin_clear_rows': sum(r['actual_model_margin_clear'] for r in results),
            'epoch_summaries': summaries, 'highest_weight_raw_row': top, 'rows': results,
            'scope': 'Same original raw paths and exact consumed CV model; conditional fixture physical dynamic geometry only.',
            'limits': ['No static/raw203, constrained-SG/final bounds, execution or goal safety certificate.',
                       'Actual score and source-pose epochs are distinct; sensitivity does not certify physical execution timing.',
                       'Full-trace velocity age FAILED, task/raw203 FAILED and witness NOT ESTABLISHED remain.',
                       'No re-ranking, truth-selected parameters, CA/sampler changes or deployment acceptance.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'witness_archive', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report = analyze(args.source.resolve(), args.witness_archive.resolve())
    with args.output.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k!='rows'}), flush=True)
