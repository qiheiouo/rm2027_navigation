#!/usr/bin/env python3
"""Visualize same-raw-path model/physical disagreement, with frozen weights."""
import argparse
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_raw_rollout_objective_v1'
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    args.output.mkdir()
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.8))
    epoch = 'actual_score_clock'
    for passed, color, label in [(False, '#a73d2a', 'Physical dynamic gates fail'),
                                 (True, '#157a56', 'Physical dynamic gates pass')]:
        rows = [r for r in report['rows'] if r['physical_dynamic_labels'][epoch]['all_dynamic_geometry_gates']==passed]
        axes[0].scatter([r['actual_model_minimum_clearance'] for r in rows],
                        [r['physical_dynamic_labels'][epoch]['geometry']['mechanical']['interval_lower'] for r in rows],
                        s=[8+1200*r['native_weight'] for r in rows], alpha=.65, color=color,
                        label=f'{label}: {len(rows)} rows')
    axes[0].axvline(report['model_margin'], color='#286e9d', linestyle='--', linewidth=1)
    axes[0].axhline(.05, color='#333333', linestyle=':', linewidth=1)
    top = report['highest_weight_raw_row']
    x = top['actual_model_minimum_clearance']
    y = top['physical_dynamic_labels'][epoch]['geometry']['mechanical']['interval_lower']
    axes[0].annotate(f"Highest weight: row {top['raw_native_row']}\nModel clearance {x:.5f} m\nPhysical contact on raw grid", xy=(x, y),
                     xytext=(.52, .57), textcoords='axes fraction', fontsize=8,
                     bbox={'facecolor': 'white', 'edgecolor': '#cccccc', 'alpha': .9},
                     arrowprops={'arrowstyle': '->', 'color': '#333333'})
    axes[0].set_xlabel('Exact C++ CV model minimum clearance (m)')
    axes[0].set_ylabel('Same raw path: mechanical interval lower bound (m)')
    axes[0].set_title('300 original paths, actual score epoch', fontsize=11)
    axes[0].legend(loc='upper left', fontsize=8)
    categories = [('model_margin_False_physical_dynamic_False', 'Model concern / physical fails', '#87524a'),
                  ('model_margin_False_physical_dynamic_True', 'Model concern / physical passes', '#bfa765'),
                  ('model_margin_True_physical_dynamic_False', 'Model clear / physical fails', '#a73d2a'),
                  ('model_margin_True_physical_dynamic_True', 'Model clear / physical passes', '#157a56')]
    cells = report['epoch_summaries'][epoch]['contingency']
    total = math.fsum(r['native_weight'] for r in report['rows'])
    fractions = [100*cells[key]['native_weight_mass']/total for key, _, _ in categories]
    axes[1].barh(range(4), fractions, color=[color for _, _, color in categories])
    axes[1].set_yticks(range(4), [label for _, label, _ in categories], fontsize=8)
    axes[1].invert_yaxis()
    axes[1].set_xlim(0, 100)
    for index, (key, _, _) in enumerate(categories):
        fraction = fractions[index]
        percentage = f'{fraction:.2f}%' if fraction >= .001 else '<0.001%'
        axes[1].text(fraction+1, index, f"{cells[key]['rows']} rows; {percentage}", va='center', fontsize=8)
    axes[1].set_xlabel('Native weight fraction (%)')
    axes[1].set_title('Exact native weights grouped by raw-path labels', fontsize=11)
    for axis in axes:
        axis.grid(alpha=.2)
        axis.set_axisbelow(True)
    fig.suptitle('Source 298: CV model geometry misses physical contact on the same raw paths', fontsize=12)
    fig.text(.5, .065, 'Model clear means its discrete minimum exceeds 0.02 m. Physical gates require body/mechanical >=0.05 m and padded >0.', ha='center', fontsize=8)
    fig.text(.5, .025, 'Conditional 3 s fixture labels; no static/raw203, SG/final bounds, execution or task acceptance. Dot size reflects native weight.', ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .11, 1, .94))
    fig.savefig(args.output/'raw_rollout_objective.png', dpi=160)
    fig.savefig(args.output/'raw_rollout_objective.svg', metadata={'Date': None})
    plt.close(fig)


if __name__ == '__main__':
    main()
