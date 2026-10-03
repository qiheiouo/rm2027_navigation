#!/usr/bin/env python3
"""Show fixed-path factor diagnostics, never an oracle control recommendation."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_cv_support_factors_v1'
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('prepared_identity',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();report=json.loads(args.report.read_text());identity=json.loads(args.prepared_identity.read_text());args.output.mkdir()
    names=list(report['factors']);labels=['Actual\nbaseline','Source geometry\noracle','Local slope\nCV oracle','Future circle\noracle']
    colors=['#286e9d','#987039','#72548e','#157a56']
    fig,axes=plt.subplots(1,3,figsize=(14,5.9))
    for i,(name,color) in enumerate(zip(names,colors)):
        row=next(r for r in report['registered_proposals'] if r['factor']==name and r['row']==0)
        axes[0].bar(i,row['risk'],color=color)
        axes[0].text(i,row['risk']+.02,f"{row['risk']:.3g}",ha='center',va='bottom',fontsize=8)
    axes[0].set_yscale('symlog',linthresh=1);axes[0].set_ylim(0,1e6)
    axes[0].set_xticks(range(4),labels,fontsize=8);axes[0].set_ylabel('CV/circle risk on actual aggregate path')
    axes[0].set_title('Same path, four explicit input conditions',fontsize=10)
    for j,kind in enumerate(['raw','processed']):
        counts=[report['factors'][name][kind]['model_clear_but_physical_padded_grid_contact'] for name in names]
        bars=axes[1].bar(np.arange(4)+(j-.5)*.36,counts,width=.35,color=['#a73d2a','#bc8f55'][j],label=f"{kind}: {'300' if kind=='raw' else '311'} paths")
        axes[1].bar_label(bars,padding=2,fontsize=8)
    axes[1].set_xticks(range(4),labels,fontsize=8);axes[1].set_ylim(0,150)
    axes[1].set_ylabel('Model clear but true padded grid contact (rows)')
    axes[1].set_title('Unchanged same-score-clock physical labels',fontsize=10);axes[1].legend(fontsize=8,loc='upper left')
    elapsed=[t-identity['source_stamp'] for t in identity['future_times']]
    actual_y=[p[1] for p in identity['future_centers']]
    for factor,color in zip(identity['factors'][:3],colors):
        values=factor['xy_vxy_size'];prediction=[values[1]+values[3]*t for t in elapsed]
        axes[2].plot([t-identity['score_clock'] for t in identity['future_times']],prediction,color=color,label=factor['name'].replace('_',' '),linewidth=1.4)
    axes[2].plot([t-identity['score_clock'] for t in identity['future_times']],actual_y,color=colors[3],label='Physical truth future (offline)',linewidth=1.8)
    axes[2].set_xlabel('Future after actual score clock (s)');axes[2].set_ylabel('Actor / predicted anchor y in map (m)')
    axes[2].set_title('Correcting the local slope retains CV turn error',fontsize=10);axes[2].legend(fontsize=7,loc='upper left')
    for axis in axes:axis.grid(alpha=.2);axis.set_axisbelow(True)
    fig.suptitle('Source 298 factor audit: source geometry and local slope do not supply the true 3 s future',fontsize=12)
    fig.text(.5,.065,'All oracle states are offline diagnostics. Source-bracket velocity is finite-sample truth; future circle has inadmissible future information.',ha='center',fontsize=8)
    fig.text(.5,.025,'Paths, original weights and physical labels stay fixed. No new prediction model, ranking, command, static/raw203 or deployment acceptance.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.11,1,.94));fig.savefig(args.output/'cv_support_factors.png',dpi=160);fig.savefig(args.output/'cv_support_factors.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':main()
