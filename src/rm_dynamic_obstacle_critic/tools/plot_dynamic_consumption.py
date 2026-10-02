#!/usr/bin/env python3
"""Plot actual raw score contributions and epochs, never an output certificate."""
import argparse
import gzip
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_dynamic_consumption_v1'
import matplotlib.pyplot as plt
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot, COST


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('weights',type=Path)
    args=parser.parse_args();root=args.trial
    join=json.loads((root/'dynamic_consumption_audit.json').read_text())
    if join['verdict']!='EXACT CONSUMPTION/NATIVE JOIN PASS':raise ValueError('complete exact identity required')
    opener=gzip.open if args.weights.suffix=='.gz' else open
    with opener(args.weights,'rt') as stream:weights=[json.loads(line) for line in stream][1:]
    if len(weights)!=join['native_batches'] or any(not row['weights_reaggregate_bounded_mean_bit_exact'] for row in weights):
        raise ValueError('exact native weights required')
    records=[]
    for row in join['cycles']:
        ordinal=row['native_ordinal'];w=weights[ordinal]
        if w['ordinal']!=ordinal:raise ValueError('native weights ordinal')
        _,blocks=read_consumption(root/'dynamic_scores'/f"score_{row['score_ordinal']}.json")
        _,native=read_snapshot(root/'native_cycles'/f'cycle_{ordinal}.json')
        index=max(range(len(w['native_softmax_weights'])),key=lambda i:w['native_softmax_weights'][i])
        before=blocks['costs_before_dynamic']['values'][index];risk=blocks['dynamic_risk_double']['values'][index]
        residual=native[COST]['values'][index]-blocks['costs_after_dynamic']['values'][index]
        records.append((row['score_stamp'],before,risk,residual,row['actual_dynamic_double_risk_range'][1],
            1000*(row['score_stamp']-row['pose_stamp']),1000*(row['capture_stamp']-row['pose_stamp'])))
    times=[r[0] for r in records];fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for column,color,label in [(1,'#286e9d','Original seven subtotal'),(2,'#a73d2a','Actual dynamic double risk'),(3,'#157a56','Rounded static-stage increment')]:
        axes[0].plot(times,[r[column] for r in records],color=color,linewidth=1,label=label)
    axes[0].set_yscale('symlog',linthresh=1);axes[0].set_ylabel('Raw candidate costs')
    axes[0].set_title('Exact dynamic consumption: cost components of the candidate carrying most native weight',fontsize=11)
    axes[0].legend(fontsize=8,loc='upper left')
    axes[1].plot(times,[r[4] for r in records],color='#a73d2a',label='Maximum actual dynamic risk in batch')
    axes[1].plot(times,[r[2] for r in records],color='#6e4a89',label='Risk of row carrying most weight')
    axes[1].set_yscale('symlog',linthresh=1);axes[1].set_ylabel('Dynamic risk');axes[1].legend(fontsize=8,loc='upper left')
    axes[2].plot(times,[r[5] for r in records],color='#286e9d',label='Score clock minus pose source')
    axes[2].plot(times,[r[6] for r in records],color='#157a56',label='Final snapshot minus pose source')
    axes[2].axhline(150,color='#a73d2a',linestyle=':',label='Frozen velocity age window (150 ms)')
    axes[2].set_ylabel('Epoch separation (ms)');axes[2].set_xlabel('Actual dynamic score clock (simulation seconds)')
    axes[2].legend(fontsize=8,loc='upper left')
    for axis in axes:axis.grid(alpha=.2)
    fig.text(.5,.015,'Raw candidate costs and reconstructed native weights are exact. A highest-weight row is not the weighted/SG output; geometry, CV support and task gates remain separate.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.05,1,1))
    fig.savefig(root/'dynamic_consumption.png',dpi=160);fig.savefig(root/'dynamic_consumption.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':main()
