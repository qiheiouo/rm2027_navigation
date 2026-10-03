#!/usr/bin/env python3
"""Frozen-map raw185 stage paths, exact margins and all300 vector counts."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from analyze_native_individual_stages import STAGES,EPOCHS
from native_individual_stage_io import read_stages
from native_snapshot_io import read_snapshot
from witness_geometry import rotate


def plot(source,stage_file,report,scalar,output):
    stage=read_stages(stage_file);native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    output.mkdir();plt.rcParams['svg.hashsalt']='native_individual_stages_v1'
    colors=['#147d92','#cf8b24','#a73c56'];names=['Original raw','Bounded before SG','Actual-history SG']
    fig,axes=plt.subplots(1,3,figsize=(15,5));ax=axes[0]
    grid=native['map'];width,height=grid['size'];ox,oy=grid['origin'];res=grid['resolution']
    cells=np.frombuffer(blocks['raw_costmap']['bytes'],dtype=np.uint8).reshape(height,width)>=203
    ax.imshow(cells,origin='lower',extent=[ox,ox+width*res,oy,oy+height*res],cmap='Greys',alpha=.4,vmin=0,vmax=1)
    for name,color,s in zip(names,colors,stage['stages']):
        i=(185+s['raw_row_offset'])*30
        x=[native['pose'][0],*s['blocks']['x']['values'][i:i+30]];y=[native['pose'][1],*s['blocks']['y']['values'][i:i+30]]
        ax.plot(x,y,color=color,label=name,lw=2)
    first=scalar['epochs'][EPOCHS[0]]['rows'][2]['first_contact']
    polygon=rotate(np.asarray(first['pose']),np.asarray(native['padded_footprint']));polygon=np.vstack([polygon,polygon[0]])
    ax.plot(polygon[:,0],polygon[:,1],color=colors[2],ls='--',lw=1.5)
    cell=first['cell'];rect=plt.Rectangle(cell['lo'],res,res,fc='#b32137',ec='black',alpha=.8);ax.add_patch(rect)
    ax.annotate('SG +2.914 s\nraw210 cell (76,37)',xy=(first['pose'][0],first['pose'][1]),xytext=(3.1,.28),arrowprops={'arrowstyle':'->'},fontsize=9)
    ax.set(xlim=(2.9,4.8),ylim=(.1,1.3),xlabel='Frozen odom x (m)',ylabel='Frozen odom y (m)',title='Raw185: model paths on frozen map')
    ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=8,loc='upper right');ax.grid(alpha=.2)
    values=[r['scalar_exact_raw_interval_margin_m']*1000 for r in scalar['epochs'][EPOCHS[0]]['rows']]
    ax=axes[1];bars=ax.bar(range(3),values,color=colors);ax.axhline(0,c='black',lw=1)
    for b,value in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,value+(1 if value>0 else -1),f'{value:.3f}',ha='center',va='bottom' if value>0 else 'top',fontsize=10)
    ax.set(xticks=range(3),xticklabels=['Raw','Bounded','SG'],ylim=(-10,30),ylabel='Exact interval lower (mm)',title='Raw185: full three-second raw203')
    ax.grid(axis='y',alpha=.2)
    ax=axes[2];x=np.arange(3);w=.34
    for shift,epoch,color,label in [(-w/2,EPOCHS[0],'#147d92','Actual score'),(w/2,EPOCHS[1],'#858ba0','Source pose')]:
        count=[report['stage_summaries'][s][epoch]['map_vector_pass_rows'] for s in STAGES]
        bars=ax.bar(x+shift,count,w,color=color,label=label)
        for b,v in zip(bars,count):ax.text(b.get_x()+b.get_width()/2,v+1,str(v),ha='center',fontsize=10)
    ax.set(xticks=x,xticklabels=['Raw','Bounded','SG'],ylim=(0,110),ylabel='Conservative vector pass paths',title='All300: map predicates, each stage')
    ax.legend(fontsize=9);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Fixed sampled controls and original SDK history: constraints and SG have separate effects',fontsize=13)
    fig.text(.5,.02,'Exact cell scope: three raw185 paths. All300 map labels are conservative. Model hypotheses are not published controls.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.92))
    fig.savefig(output/'native_individual_stages.png',dpi=160)
    fig.savefig(output/'native_individual_stages.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','stage_file','report','scalar','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();plot(args.source,args.stage_file,json.loads(args.report.read_text()),json.loads(args.scalar.read_text()),args.output)
