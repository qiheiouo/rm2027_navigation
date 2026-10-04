#!/usr/bin/env python3
"""Independent physical trajectories and clearance diagnostics, no planner truth."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_run import physical_projection, polygon_distance

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    colors={'b0':'#386cb0','candidate':'#dc6b20'}
    for row,(scenario,left,right) in enumerate((('Crossing','shadow06','mpc01'),('Head-on','head_on02','head_on_mpc01'))):
        for label,name in (('b0',left),('candidate',right)):
            es=[json.loads(line) for line in (root/name/'events.jsonl').open()]
            summary=json.loads((root/name/'run_summary.json').read_text());begin=summary['goal_epoch_s']
            robot={};actor={}
            for e in es:
                if e['topic'] not in ('/simulation/oracle/rm_sentry_2027','/simulation/oracle/moving_obstacle'):continue
                m=e['topic'].rsplit('/',1)[-1]
                ns,shape,p,r=physical_projection(e['data'],m,'base_link' if m=='rm_sentry_2027' else 'obstacle_link')
                if ns/1e9>=begin:(robot if m=='rm_sentry_2027' else actor)[ns]=(shape,p)
            pairs=sorted(set(robot)&set(actor))
            xy=[robot[n][1] for n in pairs];at=[actor[n][1] for n in pairs]
            axes[row,0].plot([p[0] for p in xy],[p[1] for p in xy],color=colors[label],label='MPPI' if label=='b0' else 'MPC + fallback')
            if label=='b0':axes[row,0].plot([p[0] for p in at],[p[1] for p in at],'--',color='#626262',label='actual obstacle')
            clearance=[polygon_distance(robot[n][0],actor[n][0]) for n in pairs]
            axes[row,1].plot([n/1e9-begin for n in pairs],clearance,color=colors[label],label=label)
            if label=='candidate':
                selected=[e for e in es if e['topic']=='/controller_selector']
                mpc=[e['receipt_sim_ns']/1e9-begin for e in selected if e['data']['data']=='FollowPathTemporalMPC']
                if mpc:
                    end=next((e['receipt_sim_ns']/1e9-begin for e in selected if e['data']['data']=='FollowPathMPPI' and e['receipt_sim_ns']/1e9-begin>mpc[0]),pairs[-1]/1e9-begin)
                    axes[row,1].axvspan(mpc[0],end,color=colors[label],alpha=.15,label='MPC selected')
        axes[row,0].plot(5.6,0.,'*',ms=12,color='#36904c',label='goal')
        axes[row,0].set(title=scenario+' actual physical path',xlabel='world x (m)',ylabel='world y (m)')
        axes[row,0].set_aspect('equal',adjustable='datalim')
        axes[row,0].legend(fontsize=8);axes[row,0].grid(alpha=.2)
        axes[row,1].axhline(.05,color='#a63636',ls=':',label='0.05 m gate')
        axes[row,1].set(title='Sampled full mechanical envelope clearance',xlabel='simulation time after goal (s)',ylabel='distance (m)')
        axes[row,1].legend(fontsize=8);axes[row,1].grid(alpha=.2)
    fig.suptitle('Actual Gazebo paired evidence — neither scenario passes dynamic acceptance\nSampled clearance is diagnostic; continuous plant error bounds remain unverified',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(root/'paired_physical_paths.png',dpi=160);plt.close(fig)
