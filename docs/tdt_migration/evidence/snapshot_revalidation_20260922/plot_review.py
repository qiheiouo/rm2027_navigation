#!/usr/bin/env python3
"""Review actual target-series trajectories and post-navigation raw costmap."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, Circle
from reference_experiment import ROOT, HERE, read_rows, verify
from audit_geometry import placed, OBSTACLES

m=verify();report=json.loads((ROOT/'aggregate.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(15,5))
for ax,name in zip(axes[:2],('tdt_astar','tdt_qp')):
    for x0,y0,x1,y1 in OBSTACLES:
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='.3'))
    for t in [t for t in report['trials'] if t['planner']==name]:
        rows=read_rows(ROOT/f"{name}_{t['trial']}"/'observation/trajectory.jsonl')
        line,=ax.plot([s['x'] for s in rows],[s['y'] for s in rows],label=f"Trial {t['trial']}, recoveries {t['recoveries']}")
        last=rows[-1];poly=placed(m['body_polygon_m'],(last['x'],last['y'],last['yaw']))
        ax.add_patch(Polygon(poly,fill=False,edgecolor=line.get_color(),alpha=.6))
    ax.plot(4.3,0,'k*',ms=10);ax.add_patch(Circle((4.3,0),.15,fill=False,ls=':',edgecolor='black'))
    ax.set(title=name,aspect='equal',xlim=(-.6,5.3),ylim=(-2.6,1.1),xlabel='x (m)',ylabel='y (m)')
    ax.legend(loc='lower left',fontsize=8);ax.grid(alpha=.2)
last=report['trials'][-1];raw=json.loads((ROOT/f"{last['planner']}_{last['trial']}"/'runtime_geometry.json').read_text())['map']
ax=axes[2];width=raw['width'];res=raw['resolution'];ox,oy=raw['origin']
for cost,color,label in [(254,'black','Lethal'),(255,'.7','Unknown')]:
    points=[(ox+(i%width+.5)*res,oy+(i//width+.5)*res) for i,v in enumerate(raw['data']) if v==cost]
    if points:ax.scatter(*zip(*points),s=5,c=color,marker='s',label=label)
ax.add_patch(Polygon(placed(m['body_polygon_m'],(4.3,0,0)),fill=False,color='tab:blue',label='Nominal polygon'))
ax.add_patch(Circle((4.3,0),last['nominal_after_navigation']['derived_spin_required_m'],fill=False,color='tab:red',label='Derived Spin + margins'))
ax.plot(4.3,0,'k*',ms=10)
ax.set(title='Post-navigation raw map (last trial)',aspect='equal',xlim=(3.1,5.2),ylim=(-1.1,1.1),xlabel='x (m)',ylabel='y (m)')
ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('Snapshot revalidation | 382/126 mm reference geometry | static fixture only')
fig.tight_layout();fig.savefig(HERE/'trajectory_review.png',dpi=150);fig.savefig(HERE/'trajectory_review.svg')
