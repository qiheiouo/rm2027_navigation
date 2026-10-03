#!/usr/bin/env python3
"""Export original raw-grid/dense query and whole-polygon predicate comparisons."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(report_path,subset_path,cpp_path,output):
    report=json.loads(report_path.read_text());subset=json.loads(subset_path.read_text());selected={73:[],185:[]}
    with cpp_path.open() as stream:
        next(stream)
        for line in stream:
            row=json.loads(line)
            if row['epoch_index']==0 and row['path_kind']==2 and row['path_row'] in selected:selected[row['path_row']].append(row)
    plt.rcParams.update({'svg.hashsalt':'rm_native_raw_map_contract_v1','font.size':10})
    fig,axes=plt.subplots(1,3,figsize=(14,4.8),gridspec_kw={'width_ratios':[1,1,1.25]})
    for ax,index in zip(axes[:2],[73,185]):
        trace=report['rows'][index]['epochs']['actual_score_clock']['dense_SDK_queries'];points=selected[index]
        exact=next(r for r in subset['epochs']['actual_score_clock']['rows'] if r['original_raw_row']==index)
        ax.plot([p['future_seconds'] for p in points],[p['native_footprint_cost'] for p in points],color='#168aad',label='Dense SDK boundary')
        ax.plot([p['future_seconds'] for p in points],[p['center_raw'] for p in points],color='#7d8597',label='Center raw')
        ax.axhline(203,color='#bc1f2c',linestyle='--',linewidth=1);contact=exact['first_contact']
        if contact:ax.axvline(contact['future_seconds'],color='#bc1f2c',linestyle=':',linewidth=1)
        text=f"Boundary max {trace['maximum_native_footprint_cost']}\nWhole polygon "+(f"+{contact['future_seconds']:.3f}s" if contact else 'no sampled contact')
        text+='\nInterval lower '+f"{1000*exact['scalar_exact_raw_interval_margin_m']:.3f} mm"
        ax.text(.04,.72,text,transform=ax.transAxes,fontsize=9,bbox={'facecolor':'white','edgecolor':'#ddd','alpha':.9})
        ax.set(title=f'Original raw row{index}',xlabel='Future time from actual score (s)',xlim=(0,3.03),ylim=(-12,270));ax.grid(alpha=.18)
    axes[0].set_ylabel('Frozen grid cost byte');axes[0].legend(loc='lower left',fontsize=8)
    x=np.arange(3)
    for i,(epoch,label) in enumerate([('actual_score_clock','Actual score'),('source_pose_clock_sensitivity','Source pose')]):
        partitions=report['epoch_summaries'][epoch]['partition'];values=[partitions[j]['rows'] for j in [0,1,3]];offset=(-.18,.18)[i]
        axes[2].bar(x+offset,values,width=.34,color=['#168aad','#7d8597'][i],label=label)
        for j,v in enumerate(values):axes[2].text(j+offset,v+3,str(v),ha='center',fontsize=9)
    axes[2].set_xticks(x,['Vector fail\nSDK >=203','Vector fail\nSDK <203','Vector pass\nSDK <203'])
    axes[2].set(title='All300: distinct map predicates',ylabel='Original raw paths',ylim=(0,245));axes[2].legend(fontsize=8);axes[2].grid(axis='y',alpha=.18)
    fig.suptitle('Original raw30 and dense boundary classification agree in this source298 batch',fontsize=12)
    fig.text(.5,.02,'Vector whole-polygon labels: all300. Independent exact cell/reserve audit: ten selected raw paths. Actual CostCritic state unknown.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.065,1,.94]);output.mkdir()
    fig.savefig(output/'native_raw_map_contract.png',dpi=160);fig.savefig(output/'native_raw_map_contract.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['report','subset','cpp','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();plot(args.report,args.subset,args.cpp,args.output)
