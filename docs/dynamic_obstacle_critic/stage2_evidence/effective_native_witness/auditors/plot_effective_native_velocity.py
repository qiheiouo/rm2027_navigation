#!/usr/bin/env python3
"""Plot recovered native measurement values without inventing consumer epochs."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_effective_native_velocity_v1'
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    values = json.loads((args.trial / 'effective_velocity_audit.json').read_text())
    source = json.loads((args.trial / 'velocity_contract_audit.json').read_text())
    records = values['records']
    if [r['ordinal'] for r in records] != [r['ordinal'] for r in source['records']]:
        raise ValueError('velocity figure source ordinal mismatch')
    times = [r['source_pose_stamp'] for r in records]
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 8.4), sharex=True)
    for index, (axis, label) in enumerate(zip(axes, ['vx (m/s)', 'vy (m/s)', 'wz (rad/s)'])):
        axis.plot(times, [r['canonical_velocity'][index] for r in source['records']],
                  color='#286e9d', linewidth=1.1, label='Canonical value at pose source time')
        axis.plot(times, [r['native_speed'][[0, 1, 5][index]] for r in records],
                  color='#993d24', linestyle='--', linewidth=1, label='Captured native speed')
        axis.set_ylabel(label)
        axis.grid(alpha=.2)
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].set_title('Canonical route repaired: exact native value consistency PASS; physical trial FAILED', fontsize=11)
    axes[-1].plot(times, [1000 * r['latest_matching_value_age']
                         if r['latest_matching_value_age'] is not None else float('nan') for r in records],
                  color='#157a56', linewidth=1)
    axes[-1].axhline(values['age_limit_from_frozen_guard'] * 1000, color='#993d24', linestyle=':')
    axes[-1].set_ylabel('Latest exact value\nage at capture (ms)')
    axes[-1].set_xlabel('Pose source time (simulation seconds)')
    axes[-1].grid(alpha=.2)
    note = (f"Exact thresholded values: {values['cycles_with_exact_thresholded_canonical_value']}/{values['captures']}; "
            f"nonzero native during canonical motion: {values['canonical_motion_cycles_with_nonzero_native']}/{values['canonical_motion_cycles']}.\n"
            'Matching epochs give value consistency; repeated values do not identify the actual subscriber message stamp.')
    fig.text(.5, .015, note, ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .05, 1, 1))
    for extension in ['png', 'svg']:
        options = {'dpi': 160} if extension == 'png' else {'metadata': {'Date': None}}
        fig.savefig(args.trial / ('effective_velocity.' + extension), **options)
    plt.close(fig)


if __name__ == '__main__':
    main()
