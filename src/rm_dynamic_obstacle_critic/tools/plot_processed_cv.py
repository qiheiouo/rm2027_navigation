#!/usr/bin/env python3
"""Registered processed proposals: exact CV model and conditional physical gaps."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_processed_cv_v1'
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    args.output.mkdir()
    rows = report['registered_proposals']
    if len(rows)!=11 or rows[0]['kind']!='actual_aggregate_SG':
        raise ValueError('registered processed proposal identities required')
    labels = ['Actual aggregate / SG', 'Zero proposal / SG', 'Hold measured / SG',
              'Initial world (0.2, 0) / SG', 'Initial world (0.2, +0.3) / SG',
              'Initial world (0.2, -0.3) / SG', 'Initial world (0.4, 0) / SG',
              'Initial world (0.4, +0.3) / SG', 'Initial world (0.4, -0.3) / SG',
              'Initial world (0, +0.3) / SG', 'Initial world (0, -0.3) / SG']
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.8), sharey=True)
    epoch = 'actual_score_clock'
    model = [r['CV_minimum_clearance'] for r in rows]
    physical = [r['physical_dynamic_labels'][epoch]['geometry']['mechanical']['interval_lower'] for r in rows]
    axes[0].barh(range(11), model, color=['#157a56' if r['CV_model_margin_clear'] else '#a73d2a' for r in rows])
    axes[1].barh(range(11), physical, color=['#157a56' if r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates'] else '#a73d2a' for r in rows])
    axes[0].axvline(report['model_margin'], color='#333333', linestyle='--', linewidth=1)
    axes[1].axvline(.05, color='#333333', linestyle='--', linewidth=1)
    axes[0].set_yticks(range(11), labels, fontsize=9)
    axes[0].invert_yaxis()
    axes[0].set_xlabel('CV minimum clearance (m), margin 0.02 m')
    axes[1].set_xlabel('Physical mechanical lower bound (m), margin 0.05 m')
    axes[0].set_title('Original CV model, same consumed fields', fontsize=11)
    axes[1].set_title('Full 3 s physical dynamic geometry, score epoch', fontsize=11)
    for axis, values in zip(axes, [model, physical]):
        low, high = min(values), max(values)
        span = high-low
        axis.set_xlim(min(-.05, low-.08*span), high+.28*span)
        for index, value in enumerate(values):
            axis.text(value+.015*span, index, f'{value:.3f}', va='center', fontsize=8,
                      bbox={'facecolor':'white', 'edgecolor':'none', 'pad':1})
        axis.grid(axis='x', alpha=.2)
        axis.set_axisbelow(True)
    fig.suptitle('Source 298 processed paths: actual aggregate has no CV TTC but physical contact', fontsize=12)
    fig.text(.5, .065, 'Green/red indicate the respective model or physical dynamic gates. Only row0 is the actual aggregate; other proposals are counterfactual.', ha='center', fontsize=8)
    fig.text(.5, .025, 'Actual aggregate CV risk 0.5494, minimum 0.1145 m, TTC none; zero CV risk 0. No execution, static/raw203 or task acceptance.', ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .11, 1, .94))
    fig.savefig(args.output/'processed_cv.png', dpi=160)
    fig.savefig(args.output/'processed_cv.svg', metadata={'Date': None})
    plt.close(fig)


if __name__ == '__main__':
    main()
