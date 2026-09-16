import json, math, os
from pathlib import Path
root=Path('/home/wpie/worktrees/rm2027_tdt_phase2')
os.environ['MPLCONFIGDIR']=str(root/'build/tdt_p2b/heading_ab_logs/mpl_cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import numpy as np
series=root/'build/tdt_p2b/runs/heading_follow_v1'
fig,axes=plt.subplots(2,3,figsize=(17,9))
def rows(path): return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
for variant,color in [('baseline','#1966b3'),('path_heading_follow','#cf4b27')]:
 p=series/variant/'tdt_qp_1'; tr=rows(p/'observation/trajectory.jsonl'); plans=rows(p/'observation/plans.jsonl')
 ts=[x['t']-tr[0]['t'] for x in tr]
 axes[0,0].plot([x['x'] for x in tr],[x['y'] for x in tr],color=color,label=variant)
 for x in tr[::max(1,len(tr)//12)]:
  c,s=math.cos(x['yaw']),math.sin(x['yaw'])
  poly=[(x['x']+c*a-s*b,x['y']+s*a+c*b) for a,b in [(-.33,-.28),(.33,-.28),(.33,.28),(-.33,.28)]]
  axes[0,0].add_patch(Polygon(poly,fill=False,edgecolor=color,alpha=.3))
 axes[0,1].plot(ts,np.unwrap([x['yaw'] for x in tr]),color=color,label=variant)
 axes[0,2].plot(ts,[x['wz'] for x in tr],color=color)
 axes[1,0].plot(ts,[x['body_clearance_m'] for x in tr],color=color)
 axes[1,1].plot(ts,[math.hypot(x['x']-4.3,x['y']) for x in tr],color=color)
 # Last published plan, terminal yaw is visible; not necessarily accepted by controller at every instant.
 last=plans[-1]; arc=[0.]
 for a,b in zip(last['xy'],last['xy'][1:]): arc.append(arc[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))
 axes[1,2].plot(arc,np.unwrap(last['yaw']),color=color,label=f'{variant} last plan {last["id"]}')
for bounds in [(1.225,-.55,1.575,.55),(2,.4,4,.65),(2,-.65,4,-.4)]:
 x,y,xx,yy=bounds; axes[0,0].add_patch(Rectangle((x,y),xx-x,yy-y,color='.3'))
axes[0,0].scatter([4.3],[0],marker='*',s=100,color='black');axes[0,0].set_aspect('equal');axes[0,0].legend(fontsize=8)
axes[0,0].set(title='Ground truth + padded footprints',xlabel='x (m)',ylabel='y (m)')
axes[0,1].set(title='Unwrapped base yaw (includes recovery)',xlabel='Navigation time (s)',ylabel='rad')
axes[0,2].set(title='Ground-truth yaw rate',xlabel='Navigation time (s)',ylabel='rad/s')
axes[1,0].axhline(.05,color='black',ls='--');axes[1,0].set(title='Sampled body clearance',xlabel='Navigation time (s)',ylabel='m')
axes[1,1].axhline(.15,color='black',ls='--');axes[1,1].set(title='Distance to unchanged goal',xlabel='Navigation time (s)',ylabel='m')
axes[1,2].set(title='Last published plan yaw',xlabel='Plan arc length (m)',ylabel='rad');axes[1,2].legend(fontsize=7)
for ax in axes.flat: ax.grid(alpha=.25)
fig.suptitle('Heading follow v1: one trial per variant; not deployment acceptance')
fig.tight_layout()
for ext in ('png','svg'):fig.savefig(series/f'heading_review.{ext}',dpi=160)
