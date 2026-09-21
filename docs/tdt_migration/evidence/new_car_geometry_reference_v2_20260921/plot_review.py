#!/usr/bin/env python3
"""Export geometry and measured-trajectory figures; no change to source evidence."""
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from reference_experiment import HERE,ROOT,read_rows
from audit_geometry import OBSTACLES,placed

audit=json.loads((HERE.parent/'new_car_geometry_audit_20260917/audit.json').read_text())
models=audit['models'];new=models['existing_new_car_126mm'];old=models['p2b_simulation_rectangle']
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
ax=axes[0]
for key,color,label in [('p2a_old_car_rectangle','#777777','Old car 0.64 x 0.54'),('p2b_simulation_rectangle','#d68022','P2B 0.60 x 0.50'),('existing_new_car_126mm','#087e8b','New reference 382/126 mm')]:
 ax.add_patch(Polygon(models[key]['vertices_m'],fill=False,edgecolor=color,lw=2,label=label))
ax.plot(0,0,'k+');ax.set(xlim=(-.5,.5),ylim=(-.5,.5),title='Existing footprint sources',xlabel='x (m)',ylabel='y (m)');ax.legend(loc='upper center',fontsize=8)
ax=axes[1]
for m,color,label in [(old,'#d68022','Old P2B sweep + margins'),(new,'#087e8b','New sweep + same margins')]:
 ax.add_patch(Circle((0,0),m['R_spin_nav2_padded_plus_planner_clearance_m'],fill=False,edgecolor=color,lw=2,label=label))
 ax.add_patch(Polygon(m['vertices_m'],fill=False,edgecolor=color,lw=1,linestyle=':'))
ax.add_patch(Rectangle((-.3,.35),.05,.05,color='#b82635',label='Original rejecting raw cell'))
ax.plot([0,-.25],[0,.35],'k--',lw=1);ax.plot(0,0,'k+')
ax.set(xlim=(-.52,.52),ylim=(-.52,.52),title='Nominal goal: original cell no longer blocks',xlabel='x relative to goal (m)',ylabel='y relative to goal (m)');ax.legend(loc='lower right',fontsize=7)
ax=axes[2]
for x0,y0,x1,y1 in OBSTACLES:ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#444444'))
for name,color in [('tdt_astar','#2274a5'),('tdt_qp','#cd5c35')]:
 rows=read_rows(ROOT/(name+'_1')/'observation/trajectory.jsonl')
 ax.plot([r['x'] for r in rows],[r['y'] for r in rows],color=color,lw=1.8,label=name+' (1 recovery)')
 for r in rows[::45]:ax.add_patch(Polygon(placed(new['vertices_m'],(r['x'],r['y'],r['yaw'])),fill=False,edgecolor=color,lw=.5,alpha=.45))
ax.plot(4.3,0,'k*',markersize=10,label='Nominal (4.3, 0)')
ax.set(xlim=(-.5,5.4),ylim=(-2.6,1.2),title='Measured reference v2 trajectories',xlabel='x (m)',ylabel='y (m)');ax.legend(fontsize=7)
for ax in axes:ax.set_aspect('equal');ax.grid(alpha=.2)
fig.suptitle('Reference geometry only: no heading tuning, no terminal selector; static gate still fails no_recovery',fontsize=11)
for suffix in ('svg','png'):
 path=HERE/('geometry_review_full.'+suffix)
 if path.exists():raise FileExistsError(path)
 fig.savefig(path,dpi=150)
