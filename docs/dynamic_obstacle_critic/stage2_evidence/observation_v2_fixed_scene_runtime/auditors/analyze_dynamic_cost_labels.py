#!/usr/bin/env python3
"""Exact raw cost components versus explicitly conditional bounded/SG labels."""
import argparse
import gzip
import json
import math
from pathlib import Path
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot, COST
from analyze_native_weights import analyze, contents


def analyze_components(root, weights_path, reference, selected_path):
    validated = analyze(weights_path, reference, selected_path)
    if validated['verdict'] != 'NATIVE WEIGHTS REAGGREGATION EXACT': raise ValueError('exact native weights required')
    selected = json.loads(contents(selected_path))['records'][0]; ordinal = selected['source_ordinal']
    labels = selected['proposals'][11:]
    join = json.loads((root/'dynamic_consumption_audit.json').read_text())
    if join['verdict'] != 'EXACT CONSUMPTION/NATIVE JOIN PASS': raise ValueError('actual exact consumption/native join required')
    matches = [r for r in join['cycles'] if r['native_ordinal'] == ordinal]
    if len(matches) != 1: raise ValueError('selected consumption identity')
    matched = matches[0]; meta, blocks = read_consumption(root/'dynamic_scores'/f"score_{matched['score_ordinal']}.json")
    _, native = read_snapshot(root/'native_cycles'/f'cycle_{ordinal}.json')
    audit = [json.loads(line) for line in contents(weights_path).splitlines()][ordinal+1]
    records = []
    for row in range(300):
        label = labels[row]
        if label['name'] != f'sampler_{row}_bounds_SG_counterfactual': raise ValueError('complete row identity required')
        before = blocks['costs_before_dynamic']['values'][row]; risk = blocks['dynamic_risk_double']['values'][row]
        after = blocks['costs_after_dynamic']['values'][row]; total = native[COST]['values'][row]
        post_gamma = audit['native_costs_after_regularization'][row]
        records.append({'raw_native_row': row, 'native_weight': audit['native_softmax_weights'][row],
            'original_seven_float_subtotal': before, 'actual_dynamic_double_risk': risk,
            'after_dynamic_float_cost': after, 'rounded_static_stage_increment': total-after,
            'native_pre_gamma_float_total': total, 'rounded_gamma_stage_increment': post_gamma-total,
            'native_post_gamma_float_total': post_gamma,
            'conditional_individual_bounds_SG_safe_progress': label['conditional_safe_control_with_progress'],
            'conditional_gates': label['gates']})
    groups = {}
    for name, passed in [('conditional_safe', True), ('conditional_failed', False)]:
        group = [r for r in records if r['conditional_individual_bounds_SG_safe_progress'] == passed]
        mass = math.fsum(r['native_weight'] for r in group)
        groups[name] = {'rows': len(group), 'native_weight_mass': mass,
            'native_weight_fraction': mass/validated['selected_weight_sum'],
            'ranges': {field: [min((r[field] for r in group), default=None), max((r[field] for r in group), default=None)]
                for field in ['original_seven_float_subtotal', 'actual_dynamic_double_risk', 'rounded_static_stage_increment', 'native_post_gamma_float_total']}}
    top = sorted(records, key=lambda r: (-r['native_weight'], r['raw_native_row']))[:10]
    return {'schema': 1, 'source_native_ordinal': ordinal, 'actual_consumption_ordinal': matched['score_ordinal'],
        'pose_stamp': selected['source_pose_stamp'], 'score_clock': meta['score_stamp'],
        'exact_native_weights_verdict': validated['verdict'], 'groups': groups, 'top_weight_rows': top, 'rows': records,
        'scope': 'Exact original raw candidate cost components and native weights; labels are different individually bounded/SG counterfactual trajectories.',
        'limits': ['Do not treat post-SG labels as raw trajectory safety or per-critic causal attribution.',
            'Only Dynamic risk is the exact consumed double contribution; original seven are a float subtotal and Static/gamma increments include rounding.',
            'A highest-weight row is not the actual weighted/SG command. Full-trace age, CV support, raw203 and task failures remain unchanged.',
            'No CA, sampling or ranking restoration; no deployment acceptance.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ['trial', 'weights', 'reference', 'selected', 'output']: parser.add_argument(name, type=Path)
    args = parser.parse_args(); report = analyze_components(args.trial, args.weights, args.reference, args.selected)
    with args.output.open('x') as stream: stream.write(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['rows', 'top_weight_rows']}))
