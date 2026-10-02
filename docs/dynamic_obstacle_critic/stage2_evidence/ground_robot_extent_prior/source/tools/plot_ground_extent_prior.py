#!/usr/bin/env python3
"""Plot frozen extent hypotheses, without promoting them to runtime bounds."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_ground_extent_prior_v1'
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    consumed_radii = {r['used_radius'] for r in report['records']}
    if len(consumed_radii) != 1:
        raise ValueError('this frozen plot requires a constant consumed radius')
    consumed_radius = consumed_radii.pop()
    args.output.mkdir()
    horizons = ['0.0', '1.0', '2.0', '3.0']
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.8))
    samples = [[r['required_radius_by_horizon'][h] for r in report['records']] for h in horizons]
    axes[0].boxplot(samples, labels=['Now', '+1 s', '+2 s', '+3 s'], whis=(0, 100), showfliers=False)
    for radius, color, label in [(consumed_radius, '#286e9d', f'Consumed radius ({consumed_radius:.3f} m)'),
            (.5 * np.hypot(.8, .8), '#a73d2a', 'Wrong center assumption (0.566 m)'),
            (report['zero_error_radius_hypothesis'], '#157a56', 'All-ground diameter hypothesis (1.697 m)')]:
        axes[0].axhline(radius, color=color, linestyle='--', linewidth=1, label=label)
    axes[0].set_ylabel('Radius needed for full actor projection (m)')
    axes[0].set_title('Exact consumed CV anchor and score clock', fontsize=11)
    axes[0].legend(loc='upper left', fontsize=8)
    labels = ['Consumed radius', 'Wrong center assumption', 'All-ground diameter hypothesis']
    colors = ['#286e9d', '#a73d2a', '#157a56']
    for index, (name, label, color) in enumerate(zip(report['hypotheses'], labels, colors)):
        counts = [report['hypotheses'][name][h]['full_physical_box_covered'] for h in horizons]
        bars = axes[1].bar(np.arange(4)+(index-1)*.26, counts, width=.25, color=color, label=label)
        axes[1].bar_label(bars, fontsize=8, padding=3)
    axes[1].set_xticks(range(4), ['Now', '+1 s', '+2 s', '+3 s'])
    axes[1].set_ylim(0, report['usable_actor_scores'] * 1.2)
    axes[1].set_ylabel(f"Full actor projections covered / {report['usable_actor_scores']} scores")
    axes[1].set_title('Coverage of the same frozen physical actor', fontsize=11)
    axes[1].legend(loc='upper right', fontsize=8)
    for axis in axes:
        axis.grid(axis='y', alpha=.2)
        axis.set_axisbelow(True)
    fig.suptitle('Ground robot extent prior: current geometry and future CV support remain separate', fontsize=12)
    outside = report['filtered_CV_anchor_outside_current_actor_projection_scores']
    fig.text(.5, .065, f"2026 rules are conditional reference bounds; {outside}/{report['usable_actor_scores']} current anchors lie outside the physical projection.", ha='center', fontsize=9)
    fig.text(.5, .025, 'Zero anchor error is a hypothesis. No certified motion/error bound, online parameter change, or deployment acceptance.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .11, 1, .94))
    fig.savefig(args.output/'ground_extent_prior.png', dpi=160)
    fig.savefig(args.output/'ground_extent_prior.svg', metadata={'Date': None})
    plt.close(fig)


if __name__ == '__main__':
    main()
