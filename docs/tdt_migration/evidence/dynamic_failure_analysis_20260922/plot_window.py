"""Plot frozen time-window analysis with observation and timestamp limits explicit."""
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
data=json.loads((HERE/'window_analysis.json').read_text())
fig,axes=plt.subplots(2,3,figsize=(16,8))
for row,t in enumerate(data['trials']):
    start=t['points']['gate_violation']['state']['t']
    overlap=t['points']['first_overlap']['state']['t']-start
    poses=t['actual_pose_series'];scans=t['scans']
    ax=axes[row,0]
    ax.plot([p['t']-start for p in poses],[p['body_gap_m'] for p in poses],label='Actual moving-box gap')
    points=list(t['points'].values())
    ax.plot([p['state']['t']-start for p in points],
        [p['local_map']['body_to_published_lethal_cells_m'] for p in points],'o--',label='Last published map: body to lethal cells')
    ax.axhline(.05,color='red',ls=':',label='0.05 m body gate')
    ax.set(title=t['planner']+': geometry and published-map gap',ylabel='Gap (m)');ax.legend(fontsize=7)
    ax=axes[row,1]
    ax.plot([p['t']-start for p in scans],[p['expected_box_rays'] for p in scans],label='Expected box rays')
    ax.plot([p['t']-start for p in scans],[p['matching_rays'] for p in scans],label='Matching measured ranges')
    ax.set(title='Scan correspondence (not receipt latency)',ylabel='Ray count');ax.legend(fontsize=7)
    ax=axes[row,2]
    for k,label in [('commands','Observed /cmd_vel'),('trajectory','Ground-truth twist')]:
        values=t[k]
        ax.plot([p['t']-start for p in values],[math.hypot(p['vx'],p['vy']) for p in values],label=label)
    ax.set(title='Translation speed; raw MPPI command unavailable',ylabel='Speed (m/s)');ax.legend(fontsize=7)
    for ax in axes[row]:
        ax.axvline(0,color='red',ls='--',lw=.8)
        ax.axvline(overlap,color='black',ls=':',lw=.8)
        ax.set(xlabel='Seconds from first sampled clearance violation')
        ax.grid(alpha=.2)
fig.suptitle('Offline failure-window review: red = clearance violation; black = first sampled overlap')
fig.tight_layout()
for suffix in ('png','svg'):
    target=HERE/f'failure_window.{suffix}'
    if target.exists():raise FileExistsError(target)
    fig.savefig(target,dpi=150)
p=HERE/'failure_window.svg'
p.write_text('\n'.join(line.rstrip() for line in p.read_text().splitlines())+'\n')
