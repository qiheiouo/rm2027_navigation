#!/usr/bin/env python3
"""Apply the preregistered earliest dynamic-failure/witness selection rule."""
import argparse
import gzip
import json
from pathlib import Path


def select(policy, analysis):
    eligible = []
    if [row['ordinal'] for row in analysis['records']] != list(range(analysis['cycles'])):
        raise ValueError('selection requires complete ordered full-cycle analysis')
    for row in analysis['records']:
        proposals = row['proposals']
        if len(proposals) != 11 or proposals[0]['name'] != 'actual_aggregate':
            raise ValueError('selection requires the fixed eleven registered proposals')
        geometry = proposals[0]['geometry']
        dynamic_failed = (
            geometry['body']['dynamic_lower'] < policy['body_clearance']
            or geometry['mechanical']['dynamic_lower'] < policy['body_clearance']
            or any(geometry[name]['dynamic_lower'] <= policy['padded_clearance_strict']
                   for name in ['padded_configured', 'padded_native']))
        if dynamic_failed and any(proposal['conditional_safe_control_with_progress']
                                  for proposal in proposals[1:]):
            eligible.append(row)
    return {
        'selection': 'earliest actual aggregate dynamic-geometry failure with a registered full-gate progress witness',
        'eligible_cycles': len(eligible),
        'selected_record': eligible[0] if eligible else None,
        'limits': 'Offline selection after full registered analysis. Actor truth labels do not select controls. No coverage or physical execution conclusion.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('analysis', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    text = gzip.open(args.analysis, 'rt').read() if args.analysis.suffix == '.gz' else args.analysis.read_text()
    result = select(json.loads((args.trial / 'policy.json').read_text()), json.loads(text))
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'eligible_cycles': result['eligible_cycles'],
                      'source_ordinal': result['selected_record']['source_ordinal'] if result['selected_record'] else None}))


if __name__ == '__main__':
    main()
