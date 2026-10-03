#!/usr/bin/env python3
"""Show dynamic improvement and retained raw203 failure on fixed SDK paths."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_native_factor_aggregation_v1'
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from analyze_trial import rotation
from native_factor_io import read_paths
from native_snapshot_io import read_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','paths','analysis','raw_scalar','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();report=json.loads(args.analysis.read_text());raw=json.loads(args.raw_scalar.read_text());record=read_paths(args.paths)
    native,blocks=read_snapshot(args.source/'native_cycles/cycle_298.json');args.output.mkdir()
    rows=report['rows'];labels=['Actual\nbaseline','Source geometry\noracle','Local slope\nCV oracle','Future circle\noracle'];colors=['#286e9d','#987039','#72548e','#157a56'];epoch='actual_score_clock'
    fig,axes=plt.subplots(1,3,figsize=(14,5.9))
    for offset,kind,color in [(-.18,'mechanical','#286e9d'),(.18,'padded_native','#987039')]:
        values=[r['full_conditional_labels'][epoch]['geometry'][kind]['dynamic_lower'] for r in rows]
        bars=axes[0].bar(np.arange(4)+offset,values,width=.34,color=color,label=kind.replace('_',' '))
        for bar,value in zip(bars,values):
            label_y=value+.005 if value>0 else (-.022 if kind=='mechanical' else -.036)
            axes[0].text(bar.get_x()+bar.get_width()/2,label_y,f'{value:.3f}',ha='center',va='bottom' if value>0 else 'top',fontsize=8,
                         bbox={'facecolor':'white','edgecolor':'none','alpha':.9,'pad':.2})
    axes[0].axhline(.05,color='#555555',linestyle='--',linewidth=1,label='mechanical gate 0.05m')
    axes[0].axhline(0,color='#555555',linewidth=.8);axes[0].set_ylim(-.05,.24)
    axes[0].set_ylabel('Dynamic distance interval lower (m)');axes[0].set_title('Same full 3 s dynamic geometry',fontsize=10);axes[0].legend(fontsize=7,loc='upper left')
    values=[1000*r['full_conditional_labels'][epoch]['raw_interval_margin_lower'] for r in rows]
    bars=axes[1].bar(range(4),values,color=colors);axes[1].bar_label(bars,labels=[f'{v:.3f}' for v in values],padding=3,fontsize=8)
    axes[1].axhline(0,color='#555555',linewidth=.8);axes[1].set_ylim(-9,2)
    axes[1].set_ylabel('Frozen raw203 interval margin (mm)');axes[1].set_title('Raw203 is a separate unchanged gate',fontsize=10)
    for axis in axes[:2]:axis.set_xticks(range(4),labels,fontsize=8);axis.grid(alpha=.2);axis.set_axisbelow(True)
    m=native['map'];ox,oy=m['origin'];w,h=m['size'];res=m['resolution'];values=np.frombuffer(blocks['raw_costmap']['values'],dtype=np.uint8).reshape(h,w)
    axes[2].imshow(values>=203,origin='lower',extent=[ox,ox+w*res,oy,oy+h*res],cmap=ListedColormap(['#ffffff','#dddddd']),interpolation='nearest',zorder=0)
    for i,(row,color) in enumerate(zip(rows,colors)):
        x=[native['pose'][0],*record['blocks']['x']['values'][i*30:(i+1)*30]]
        y=[native['pose'][1],*record['blocks']['y']['values'][i*30:(i+1)*30]]
        axes[2].plot(x,y,color=color,linewidth=1.7,label=labels[i].replace('\n',' '))
    witness=raw['epochs'][epoch]['rows'][3]['first_contact'];cell=witness['cell']
    axes[2].add_patch(Rectangle(cell['lo'],res,res,facecolor='#a83f30',edgecolor='#a83f30',zorder=3))
    axes[2].add_patch(Polygon(rotation(witness['pose'],native['padded_footprint']),fill=False,edgecolor=colors[3],linewidth=1.5,zorder=4))
    axes[2].annotate('Raw216 cell\nFuture oracle +2.982s',xy=(cell['lo'][0]+res/2,cell['lo'][1]+res/2),xytext=(3.45,.26),fontsize=8,arrowprops={'arrowstyle':'->','color':'#a83f30'},bbox={'facecolor':'white','edgecolor':'none','alpha':.9})
    axes[2].scatter([native['pose'][0]],[native['pose'][1]],marker='x',color='black',s=25)
    axes[2].set(xlim=(2.95,4.7),ylim=(.2,1.55),xlabel='x in frozen odom context (m)',ylabel='y (m)');axes[2].set_aspect('equal');axes[2].legend(fontsize=7,loc='upper left')
    axes[2].set_title('Padded pose and captured raw blocked cells',fontsize=10)
    fig.suptitle('Fixed-context native factor replay: future truth improves dynamic clearance; all four retain a failed gate',fontsize=12)
    fig.text(.5,.065,'Original seven critic costs, gamma, temperature, sampled controls, warm sequence and SG history are fixed; hypotheses are independent.',ha='center',fontsize=8)
    fig.text(.5,.025,'Truth oracles are offline only. Raw cell contact is a captured-map label. No execution, goal, future-map or deployment acceptance.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.11,1,.94));fig.savefig(args.output/'native_factor_aggregation.png',dpi=160);fig.savefig(args.output/'native_factor_aggregation.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':main()
