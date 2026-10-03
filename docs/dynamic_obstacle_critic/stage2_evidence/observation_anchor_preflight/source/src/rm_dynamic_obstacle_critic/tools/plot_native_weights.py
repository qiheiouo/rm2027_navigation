#!/usr/bin/env python3
"""Native weight distribution vs independently frozen, conditional SG labels."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_native_weights_v1'
import matplotlib.pyplot as plt
from analyze_native_weights import contents


def main():
    parser = argparse.ArgumentParser()
    for name in ['audit', 'analysis', 'selected', 'output_prefix']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report = json.loads(contents(args.analysis))
    source = json.loads(contents(args.selected))['records'][0]
    records = [json.loads(line) for line in contents(args.audit).splitlines()[1:]]
    row = records[report['selected_source_ordinal']]
    labels = source['proposals'][11:]
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.2), gridspec_kw={'width_ratios': [2.2, 1]})
    for safe, color, label in [(True, '#286e9d', 'Safe individual bounded/SG label'),
                               (False, '#a6432d', 'Failed individual bounded/SG label')]:
        indices = [index for index, proposal in enumerate(labels)
                   if proposal['conditional_safe_control_with_progress'] == safe]
        axes[0].scatter([row['native_costs_after_regularization'][index] for index in indices],
                        [row['native_softmax_weights'][index] for index in indices],
                        s=15, color=color, alpha=.7, label=label+f' ({len(indices)})')
    axes[0].set_yscale('log')
    axes[0].set_xlabel('Actual native total cost after gamma')
    axes[0].set_ylabel('Native softmax weight (log scale)')
    axes[0].grid(alpha=.2)
    axes[0].legend(fontsize=8, loc='upper right')
    top = report['selected_top_weights'][0]
    axes[0].annotate(f"Row {top['row']}: weight {top['weight']:.3f}",
                     (top['post_gamma_cost'], top['weight']), xytext=(.2, .75),
                     textcoords='axes fraction', fontsize=9, arrowprops={'arrowstyle': '->', 'color': '#555555'})
    fraction = report['selected_safe_label_weight_fraction']
    axes[1].bar(['Safe labels', 'Failed labels'], [100*fraction, 100*(1-fraction)], color=['#286e9d', '#a6432d'])
    axes[1].set_ylabel('Fraction of native weight (%)')
    axes[1].set_ylim(0, 112)
    axes[1].text(0, 4, f'{100*fraction:.4f}%', ha='center', fontsize=10)
    axes[1].text(1, 102, f'{100*(1-fraction):.4f}%', ha='center', fontsize=10)
    axes[1].grid(axis='y', alpha=.2)
    fig.suptitle(f"Native total costs favor failed SG labels: cycle {report['selected_source_ordinal']}, source {report['selected_pose_stamp']:.3f} s", fontsize=12)
    note = (f"Bounded native mean bit exact: {report['bounded_mean_bit_exact_cycles']}/{report['cycles']}; "
            f"actual SG chain byte exact: {report['original_SG_chain_byte_exact_cycles']}/{report['cycles']}.\n"
            'Labels are individual counterfactual controls after bounds/SG; weights belong to raw controls. No per-critic or execution certificate.')
    fig.text(.5, .022, note, ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .1, 1, .94))
    for extension in ['png', 'svg']:
        options = {'dpi': 160} if extension == 'png' else {'metadata': {'Date': None}}
        fig.savefig(str(args.output_prefix) + '.' + extension, **options)
    plt.close(fig)


if __name__ == '__main__':
    main()
