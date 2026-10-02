#!/usr/bin/env python3
"""Recorded native speed input and source-aligned canonical measurements."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_native_velocity_contract_v1'
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('audit',type=Path);parser.add_argument('output_prefix',type=Path);args=parser.parse_args()
    data=json.loads(args.audit.read_text());records=data['records'];times=[r['source_pose_stamp'] for r in records]
    fig,axes=plt.subplots(3,1,figsize=(10.5,7.0),sharex=True)
    for index,(axis,label,color) in enumerate(zip(axes,['vx (m/s)','vy (m/s)','wz (rad/s)'],['#286e9d','#157a56','#a64b30'])):
        axis.plot(times,[r['canonical_velocity'][index] for r in records],color=color,linewidth=1.1,label='Canonical odometry at source pose time')
        axis.plot(times,[r['native_speed'][[0,1,5][index]] for r in records],color='#333333',linestyle='--',linewidth=1.4,label='Captured native state.speed')
        axis.axvline(40.26,color='#7f48ac',linewidth=.9,alpha=.8)
        axis.set_ylabel(label);axis.grid(alpha=.2)
    axes[0].set_title('Native velocity input contract FAILED: all 349 inputs zero during actual motion',fontsize=11)
    axes[0].legend(loc='upper right',fontsize=8)
    axes[-1].set_xlabel('Source pose time (simulation seconds)')
    fig.text(.5,.018,'Canonical motion above controller thresholds in 295/349 samples. Native averaging need not equal instantaneous twist; all-zero trace requires route validation.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.04,1,1))
    fig.savefig(str(args.output_prefix)+'.png',dpi=160)
    fig.savefig(str(args.output_prefix)+'.svg',metadata={'Date':None})
    plt.close(fig)


if __name__=='__main__':main()
