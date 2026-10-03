#!/usr/bin/env python3
"""Show descriptive safe-individual weight mass and the row63 epoch sensitivity."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    report=json.loads(args.report.read_text());factors=list(report['factor_summaries'])
    plt.rcParams.update({'svg.hashsalt':'rm_native_factor_safe_mass_v1','font.size':10})
    fig,axes=plt.subplots(1,2,figsize=(12,4.6),gridspec_kw={'width_ratios':[1.5,1]})
    colors=['#7d8597','#168aad'];x=np.arange(4)
    for k,(epoch,label) in enumerate([('source_pose_clock_sensitivity','Source pose: 39 individuals'),('actual_score_clock','Actual score: 38 individuals')]):
        values=[100*report['factor_summaries'][f][epoch]['safe_individual_weight_fraction'] for f in factors]
        offset=(-.18,.18)[k];axes[0].bar(x+offset,values,width=.34,color=colors[k],label=label)
        for j,value in enumerate(values):axes[0].text(j+offset,value+.8,f'{value:.2f}',ha='center',fontsize=8)
    axes[0].set(title='Native weight on individually safe SG paths',ylabel='Descriptive fraction (%)',ylim=(0,50))
    axes[0].set_xticks(x,['Actual','Source box','Local CV','Future circle']);axes[0].legend(loc='upper left',fontsize=8);axes[0].grid(axis='y',alpha=.2)
    row=report['rows'][63];labels=['Source pose\n47.980 s','Actual score\n48.004 s']
    fields=['source_pose_clock_sensitivity','actual_score_clock'];lower=[];sample=[]
    for e in fields:
        geo=row['individual_full_labels'][e]['geometry']['padded_native'];lower.append(1000*geo['dynamic_lower']);sample.append(1000*geo['dynamic_sample_min'])
    xx=np.arange(2);axes[1].bar(xx-.18,sample,width=.34,color='#cad2c5',label='Sampled minimum')
    axes[1].bar(xx+.18,lower,width=.34,color=['#7d8597','#bc1f2c'],label='Interval lower bound')
    for j,value in enumerate(sample):axes[1].text(j-.18,value+.4,f'{value:.3f}',ha='center',fontsize=8)
    for j,value in enumerate(lower):axes[1].text(j+.18,value+(.4 if value>=0 else -.5),f'{value:.3f}',ha='center',va='bottom' if value>=0 else 'top',fontsize=8)
    axes[1].axhline(0,color='black',linestyle='--',linewidth=1);axes[1].set_xticks(xx,labels)
    axes[1].set(title='Row63 padded dynamic epoch sensitivity',ylabel='mm',ylim=(-3,16));axes[1].legend(loc='upper right',fontsize=8);axes[1].grid(axis='y',alpha=.2)
    fig.suptitle('Source298: safe individual candidates exist, all four aggregates still fail',fontsize=12)
    fig.text(.5,.018,'Raw costs weight original sampled controls; individual labels use separate bounds/SG paths. No control selection or execution.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.06,1,.93]);args.output.mkdir()
    fig.savefig(args.output/'native_factor_safe_mass.png',dpi=160);fig.savefig(args.output/'native_factor_safe_mass.svg',metadata={'Date':None});plt.close(fig)
    return 0


if __name__=='__main__':raise SystemExit(main())
