#!/usr/bin/env python3
"""Combine conditional geometry with repaired measurement value provenance."""
import argparse
import gzip
import json
from pathlib import Path


def read(path):
    return json.loads(gzip.open(path, 'rt').read() if path.suffix == '.gz' else path.read_text())


def assess(full, selected, velocity):
    full = read(full)
    selected = read(selected)
    velocity = read(velocity)
    record = selected['records'][0]
    ordinal = record['source_ordinal']
    sampler = record['proposals'][11:]
    if len(selected['records']) != 1 or len(sampler) != 300:
        raise ValueError('assessment requires the selected complete native batch')
    input_record = velocity['records'][ordinal]
    if input_record['ordinal'] != ordinal:
        raise ValueError('assessment native velocity association')
    context_safe = lambda proposal: all(value for key, value in proposal['gates'].items()
        if key not in ['strict_positive_progress', 'progress_beyond_source_float_rounding'])
    return {
        'verdict': 'CONDITIONAL MEASURED-INPUT WITNESSES' if
                   velocity['verdict'] == 'EFFECTIVE VELOCITY VALUE CONSISTENCY PASS'
                   and full['verdict'] == 'CONDITIONAL WITNESSES FOUND' else 'NOT ESTABLISHED',
        'scope': 'recorded native measurement values, exact model/SG, full three-second fixture geometry and frozen native map; no physical execution certificate',
        'full_cycles': full['cycles'], 'effective_velocity_value_contract': velocity['verdict'],
        'selected_source_ordinal': ordinal, 'selected_pose_stamp': record['source_pose_stamp'],
        'selected_native_measured_speed': input_record['native_speed'],
        'actual_consumer_velocity_source_stamp': None,
        'fixed_registered_counts': full['counts'],
        'selected_registered_safe_progress': [proposal['name'] for proposal in record['proposals'][:11]
                                              if proposal['conditional_safe_control_with_progress']],
        'sampler_rows': len(sampler), 'sampler_bounds_SG_safe_context': sum(context_safe(proposal) for proposal in sampler),
        'sampler_bounds_SG_safe_progress': sum(proposal['conditional_safe_control_with_progress'] for proposal in sampler),
        'sampler_failure_counts': {key: sum(not proposal['gates'][key] for proposal in sampler)
                                   for key in sampler[0]['gates']},
        'sampler_conclusion': 'The selected native batch contains safe individually bounded/SG counterfactual controls. This cycle does not support complete sampler coverage failure. These rows are not actual weighted MPPI outputs; no global optimizer or sampler attribution.',
        'next_evidence': 'exact DynamicObstacleCritic consumed public prediction, prediction epoch and own per-candidate contribution; compare with native candidate/aggregate context before changing objective',
        'original_physical_trial': 'FAILED', 'main_or_hardware_acceptance': False,
        'limits': 'Actual subscriber stamp remains uncaptured and repeated values can be ambiguous. Pose/score epochs differ. Linear native rollout and actor interpolation are conditional. Actor truth labels are offline only, current frozen map cannot certify future map messages, and CV full physical support still fails. No CA, sampler or ranking restoration.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['full', 'selected', 'velocity', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = assess(args.full, args.selected, args.velocity)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'fixed_registered_counts'}))


if __name__ == '__main__':
    main()
