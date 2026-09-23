"""Plot observed geometry, CostCritic flags and command chain around first gate failure."""
from pathlib import Path
import json
import math
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = ROOT / 'build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2'
sys.path.insert(0,str(HERE.parent/'dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport, placed, padded, obstacle_polygon, polygon_distance
from read_trace import read_cycle,last


def lines(path):
    return [json.loads(s) for s in path.open()]

analysis = json.loads((HERE/'cycle_analysis.json').read_text())
inputs = json.loads((RUNS/'inputs.json').read_text())
fig, axes = plt.subplots(2,2,figsize=(12,7),sharex='col')
for col, result in enumerate(analysis['trials']):
    name=result['planner']; d=RUNS/(name+'_1');t0=result['first_sample_body_below_005_sim_s']
    rows=[r for r in rows_from_transport(d/'gazebo_poses.jsonl') if t0-.8<=r['t']<=t0+.8]
    body=inputs['body_polygon_m']; pad=padded(body,.03);box=obstacle_polygon()
    x=[r['t']-t0 for r in rows]
    b=[polygon_distance(placed(body,r['robot']),placed(box,r['obstacle'])) for r in rows]
    p=[polygon_distance(placed(pad,r['robot']),placed(box,r['obstacle'])) for r in rows]
    ax=axes[0,col]; ax.plot(x,b,label='body to actual box',color='tab:blue');ax.plot(x,p,label='padded to actual box',color='tab:orange')
    ax.axhline(.05,color='tab:blue',ls=':',lw=1,label='body 0.05 m gate');ax.axhline(0,color='black',lw=.6)
    ax.axvline(0,color='black',ls='--',lw=.8);ax.set_title(name);ax.set_ylabel('actual moving gap (m)');ax.grid(alpha=.2)
    twin=ax.twinx(); tt=[];mask=[]
    for f in (d/'mppi_cycles').glob('cycle_*.json'):
        m,a=read_cycle(f); t=m['map_sim_ns']*1e-9-t0
        if -.8<=t<=.8:tt.append(t);mask.append(int(last(a,'cost_critic.collisions').sum()))
    twin.scatter(tt,mask,s=11,color='tab:red',label='CostCritic flagged rollouts / 300');twin.set_ylim(0,300);twin.set_ylabel('flagged rollouts')
    cmd=lines(d/'observation/command_chain.jsonl');ax=axes[1,col]
    for topic,label,color in [('/cmd_vel_nav','MPPI output','tab:blue'),('/cmd_vel','shared output','tab:orange'),('/simulation/chassis/cmd_vel','chassis forwarded','tab:green'),('/simulation/ground_truth/odom','actual odom','tab:purple')]:
        selected=[r for r in cmd if r['topic']==topic and t0-.8<=r['t']<=t0+.8]
        ax.plot([r['t']-t0 for r in selected],[math.hypot(r['vx'],r['vy']) for r in selected],label=label,color=color,alpha=.85,lw=1)
    ax.axvline(0,color='black',ls='--',lw=.8);ax.set_ylabel('translational speed (m/s)');ax.set_xlabel('time from first body <0.05 m (sim s)');ax.grid(alpha=.2)
axes[0,0].legend(loc='upper left',fontsize=8);axes[1,0].legend(loc='upper left',fontsize=8)
fig.suptitle('Fixed-phase first cases: actual gap, MPPI collision flags and command chain')
fig.tight_layout()
fig.savefig(HERE/'cycle_window.png',dpi=160)
fig.savefig(HERE/'cycle_window.svg')
