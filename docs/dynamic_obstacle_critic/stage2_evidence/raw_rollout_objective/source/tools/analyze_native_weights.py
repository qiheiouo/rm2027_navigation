#!/usr/bin/env python3
"""Exact native output-chain check and offline conditional-label weight audit."""
import argparse
import gzip
import json
import math
from pathlib import Path


def contents(path):
    return gzip.open(path, 'rb').read() if path.suffix == '.gz' else path.read_bytes()


def analyze(audit, reference, selected):
    lines = contents(audit).splitlines(keepends=True)
    original = contents(reference).splitlines(keepends=True)
    if len(lines) != len(original) or not 2 <= len(lines) <= 1001 or lines[0] != original[0]:
        raise ValueError('weights native library/count identity')
    data = [json.loads(line) for line in lines[1:]]
    marker = b',"weights_reaggregate_bounded_mean_bit_exact"'
    records = []
    for ordinal, (line, baseline, row) in enumerate(zip(lines[1:], original[1:], data)):
        if row['ordinal'] != ordinal or marker not in line:
            raise ValueError('weights ordinal or audit schema')
        chain_exact = line.split(marker, 1)[0] + b'}\n' == baseline
        weights = row['native_softmax_weights']
        costs = row['native_costs_after_regularization']
        if (len(weights) != 300 or len(costs) != 300
                or not all(math.isfinite(v) and v >= 0 for v in weights)
                or not all(math.isfinite(v) for v in costs)
                or not any(v > 0 for v in weights)
                or type(row['weights_reaggregate_bounded_mean_bit_exact']) is not bool
                or not math.isfinite(row['weights_reaggregate_max_error'])
                or row['weights_reaggregate_max_error'] < 0
                or row['weight_probe'] not in ['native', 'uniform_negative']
                or row['cost_stage'] != 'installed total critic costs after gamma; before softmax'):
            raise ValueError('weights finite/shape/evidence field')
        if row['weights_reaggregate_bounded_mean_bit_exact'] and row['weights_reaggregate_max_error'] != 0:
            raise ValueError('exact reaggregation has nonzero error')
        records.append({
            'ordinal': ordinal, 'original_SG_chain_byte_exact': chain_exact,
            'bounded_mean_bit_exact': row['weights_reaggregate_bounded_mean_bit_exact'],
            'bounded_mean_max_error': row['weights_reaggregate_max_error'],
            'weight_sum_float64': math.fsum(weights),
            'positive_weights': sum(value > 0 for value in weights),
            'effective_sample_size': 1 / math.fsum(value * value for value in weights),
            'max_weight': max(weights)})
    selected_data = json.loads(contents(selected))
    if len({row['weight_probe'] for row in data}) != 1:
        raise ValueError('mixed weights probe modes')
    if len(selected_data['records']) != 1:
        raise ValueError('weights requires the preregistered selected single cycle')
    selected_record = selected_data['records'][0]
    ordinal = selected_record['source_ordinal']
    labels = selected_record['proposals'][11:]
    if len(labels) != 300 or not 0 <= ordinal < len(data):
        raise ValueError('weights complete selected batch')
    row = data[ordinal]
    weights = row['native_softmax_weights']
    costs = row['native_costs_after_regularization']
    safe = [proposal['conditional_safe_control_with_progress'] for proposal in labels]
    total = math.fsum(weights)
    mass = math.fsum(value for value, passed in zip(weights, safe) if passed)
    top = sorted(range(300), key=lambda index: (-weights[index], index))[:10]
    native = all(row['weight_probe'] == 'native' for row in data)
    exact = all(record['original_SG_chain_byte_exact'] and record['bounded_mean_bit_exact'] for record in records)
    return {
        'verdict': 'NATIVE WEIGHTS REAGGREGATION EXACT' if native and exact else 'FAILED',
        'scope': 'installed post-gamma costs, mirrored SDK softmax validated by full bounded mean bits and unchanged actual SG chain',
        'weight_probe': data[0]['weight_probe'], 'cycles': len(data),
        'original_SG_chain_byte_exact_cycles': sum(record['original_SG_chain_byte_exact'] for record in records),
        'bounded_mean_bit_exact_cycles': sum(record['bounded_mean_bit_exact'] for record in records),
        'bounded_mean_max_error': max(record['bounded_mean_max_error'] for record in records),
        'max_weight_sum_error_float64': max(abs(record['weight_sum_float64'] - 1) for record in records),
        'normalization_scope': 'actual float32 SDK exponent/normalization expression; double sum residual is descriptive, not a comparison tolerance',
        'selected_source_ordinal': ordinal, 'selected_pose_stamp': selected_record['source_pose_stamp'],
        'selected_safe_label_count': sum(safe), 'selected_weight_sum': total,
        'selected_safe_label_weight_mass': mass, 'selected_safe_label_weight_fraction': mass / total,
        'selected_failed_label_weight_mass': math.fsum(value for value, passed in zip(weights, safe) if not passed),
        'selected_effective_sample_size': records[ordinal]['effective_sample_size'],
        'selected_failure_weight_mass_by_gate': {key: math.fsum(value for value, proposal in zip(weights, labels) if not proposal['gates'][key])
                                                  for key in labels[0]['gates']},
        'selected_top_weights': [{'row': index, 'weight': weights[index], 'post_gamma_cost': costs[index],
                                 'bounded_SG_label_safe_progress': safe[index], 'gates': labels[index]['gates']}
                                for index in top],
        'records': records,
        'limits': 'Geometry labels belong to individually bounded/SG counterfactual controls, while weights score original raw controls. They do not certify raw trajectories, weighted aggregates or execution. Gate masses overlap. Total costs cannot identify the individual Dynamic/Static/original critic contribution. Actor truth is offline only. No sampler/CA/ranking restoration or deployment acceptance.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['audit', 'reference', 'selected', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = analyze(args.audit, args.reference, args.selected)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ['records', 'selected_top_weights']}))


if __name__ == '__main__':
    main()
