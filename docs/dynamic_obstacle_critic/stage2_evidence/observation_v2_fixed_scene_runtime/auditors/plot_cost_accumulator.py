#!/usr/bin/env python3
"""Standalone observed cost/weight/epoch figure; no safe-control labels."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_cost_accumulator_v1'
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path)
    parser.add_argument('--title',default='CostCritic accumulator evidence: physical / raw203 / task FAILED')
    args=parser.parse_args();root=args.trial
    cost=json.loads((root/'cost_accumulator_audit.json').read_text())
    chain=json.loads((root/'cost_accumulator_full_chain.json').read_text())
    dynamic=json.loads((root/'dynamic_consumption_audit.json').read_text())
    if chain['verdict']!='EXACT OBSERVED COST/RISK/NATIVE WEIGHTS/SG CHAIN PASS':raise ValueError('verified actual chain required')
    joined={r['native_ordinal']:r for r in cost['cycles'] if 'native_ordinal' in r}
    context={r['native_ordinal']:r for r in dynamic['cycles']}
    times=[context[r['native_ordinal']]['score_stamp'] for r in chain['records']]
    fig,axes=plt.subplots(3,1,figsize=(11.2,8.6),sharex=True)
    axes[0].plot(times,[r['highest_weight_rounded_cost_delta'] for r in chain['records']],label='Observed rounded CostCritic change',color='#286e9d')
    axes[0].plot(times,[r['highest_weight_actual_dynamic_risk'] for r in chain['records']],label='Actual dynamic double risk',color='#a73d2a',alpha=.8)
    axes[0].plot(times,[joined[r['native_ordinal']]['remaining_original_critic_rounded_delta'][r['highest_weight_raw_row']]
        for r in chain['records']],label='Later original critics: rounded subtotal change',color='#7c5b8d',alpha=.7)
    axes[0].set_yscale('symlog',linthresh=.1);axes[0].set_ylabel('Highest-weight raw row\nobserved costs')
    axes[0].set_title(args.title,fontsize=11)
    axes[0].legend(loc='upper left',fontsize=8)
    axes[1].plot(times,[100*r['positive_cost_weight_fraction'] for r in chain['records']],color='#157a56',
        label='Native weight on rows with positive rounded CostCritic change')
    axes[1].plot(times,[100*r['rounded_cost_positive_rows']/300 for r in chain['records']],color='#286e9d',linestyle='--',
        label='Fraction of raw rows with positive change')
    axes[1].set_ylim(-3,103);axes[1].set_ylabel('Descriptive fraction (%)');axes[1].legend(loc='upper left',fontsize=8)
    for which,color,label in [('before_capture_stamp','#286e9d','Before CostCritic minus pose source'),
                              ('after_capture_stamp','#a73d2a','After CostCritic minus pose source')]:
        axes[2].plot(times,[1000*(joined[r['native_ordinal']][which]-context[r['native_ordinal']]['pose_stamp']) for r in chain['records']],
            color=color,label=label,linewidth=1)
    axes[2].plot(times,[1000*(context[r['native_ordinal']]['capture_stamp']-context[r['native_ordinal']]['pose_stamp'])
        for r in chain['records']],color='#157a56',label='Final native snapshot minus pose source',linewidth=1)
    axes[2].axhline(150,color='#993d24',linestyle=':',label='Original age window: 150 ms')
    axes[2].set_ylabel('Epoch separation (ms)');axes[2].set_xlabel('Dynamic score clock (simulation seconds)')
    axes[2].legend(loc='upper left',fontsize=8)
    absent=[r['before_ordinal'] for r in cost['cycles'] if 'native_ordinal' not in r]
    for axis in axes:axis.grid(alpha=.2)
    fig.text(.5,.025,f"{cost['valid_observed_pairs']} observed cost pairs; {chain['cycles']} exact final-output chains; pairs {absent} have no final output.\n"
        'Rounded float change includes accumulation rounding. Weight/cost fractions are not safety labels; private CostCritic branches and real support bounds remain unknown.',
        ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.07,1,1));fig.savefig(root/'cost_accumulator.png',dpi=160)
    fig.savefig(root/'cost_accumulator.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':main()
