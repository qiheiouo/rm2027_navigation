#!/usr/bin/env python3
"""Show full-horizon stage margins and the future-oracle raw-cell contact."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from analyze_trial import rotation
from native_sg_stage_io import read_paths
from native_snapshot_io import read_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','paths','report','raw_report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();record=read_paths(args.paths);report=json.loads(args.report.read_text());raw=json.loads(args.raw_report.read_text())
    native,blocks=read_snapshot(args.source/'native_cycles/cycle_298.json')
    plt.rcParams.update({'svg.hashsalt':'rm_native_sg_stages_v1','font.size':10})
    fig,axes=plt.subplots(1,3,figsize=(15,4.7),gridspec_kw={'width_ratios':[1,1,1.3]})
    x=np.arange(4);colors=['#168aad','#e07a20'];labels=['Actual','Source box','Local CV','Future circle']
    for stage in range(2):
        rows=report['rows'][stage::2];offset=(-.18,.18)[stage]
        mechanical=[r['full_conditional_labels']['actual_score_clock']['geometry']['mechanical']['dynamic_lower'] for r in rows]
        rawmargin=[1000*r['full_conditional_labels']['actual_score_clock']['raw_interval_margin_lower'] for r in rows]
        axes[0].bar(x+offset,mechanical,width=.34,color=colors[stage],label=['Before SG','After SG'][stage])
        axes[1].bar(x+offset,rawmargin,width=.34,color=colors[stage])
        for j,value in enumerate(mechanical):axes[0].text(j+offset,value+(.009 if value>=0 else -.015),f'{value:.3f}',ha='center',va='bottom' if value>=0 else 'top',fontsize=8)
        for j,value in enumerate(rawmargin):axes[1].text(j+offset,value+(.3 if value>=0 else -(.45 if stage==0 else 1.15)),f'{value:.2f}',ha='center',va='bottom' if value>=0 else 'top',fontsize=8)
    axes[0].axhline(.05,color='black',linestyle='--',linewidth=1,label='Original 0.05 m gate')
    axes[0].set(title='Dynamic mechanical lower bound',ylabel='m',ylim=(-.055,.26));axes[0].legend(loc='upper left',fontsize=8)
    axes[1].axhline(0,color='black',linewidth=1);axes[1].set(title='Frozen raw203 lower bound',ylabel='mm',ylim=(-10,3))
    for axis in axes[:2]:axis.set_xticks(x,labels,rotation=18);axis.grid(axis='y',alpha=.2)
    grid=native['map'];res=grid['resolution'];ox,oy=grid['origin'];width,height=grid['size']
    yy,xx=np.where(np.array(blocks['raw_costmap']['values']).reshape(height,width)>=203)
    axes[2].scatter(ox+(xx+.5)*res,oy+(yy+.5)*res,s=9,marker='s',color='#c6c6c6',label='Captured raw >=203')
    for row,color,label in [(6,colors[0],'Future circle before SG'),(7,colors[1],'Future circle after SG')]:
        xy=np.array([[native['pose'][0],native['pose'][1]],*[list(p) for p in zip(record['blocks']['x']['values'][row*30:(row+1)*30],record['blocks']['y']['values'][row*30:(row+1)*30])]])
        axes[2].plot(xy[:,0],xy[:,1],color=color,linewidth=2,label=label)
        contact=raw['epochs']['actual_score_clock']['rows'][row]['first_contact']
        axes[2].add_patch(Polygon(rotation(contact['pose'],native['padded_footprint']),closed=True,fill=False,color=color,linewidth=1.5,linestyle='--' if row==6 else '-'))
    cell=raw['epochs']['actual_score_clock']['rows'][6]['first_contact']['cell'];lo=cell['lo']
    axes[2].add_patch(Rectangle(lo,res,res,facecolor='#bc1f2c',edgecolor='#bc1f2c',label='Contact cell raw216'))
    axes[2].set(title='Raw216 contact exists before SG',xlabel='x (m)',ylabel='y (m)',xlim=(3.1,5.1),ylim=(-.3,1.55),aspect='equal')
    axes[2].legend(loc='upper left',fontsize=7);axes[2].grid(alpha=.2)
    fig.suptitle('Source298 fixed native context: all four factors fail full gates before and after SG',fontsize=12)
    fig.text(.5,.012,'3.0000000447 s, 208 primary samples. Conditional offline paths; future truth is inadmissible online.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.035,1,.93]);args.output.mkdir()
    fig.savefig(args.output/'native_sg_stages.png',dpi=160);fig.savefig(args.output/'native_sg_stages.svg',metadata={'Date':None})
    plt.close(fig);return 0


if __name__=='__main__':raise SystemExit(main())
