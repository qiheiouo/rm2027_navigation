"""Scientific plots of complete outcomes, including contact and wall failures."""
import os;os.environ.setdefault('MPLCONFIGDIR','/tmp/stvl_mpl')
import pathlib,json,sys,math
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(pathlib.Path(__file__).parent));from analyze import CACHE,EVIDENCE,jsonlines
from matplotlib.patches import Polygon
fig,axs=plt.subplots(2,3,figsize=(13,7),constrained_layout=True)
scenes=['crossing','head_on','narrow','wait_release','sudden_stop','stale'];colors=dict(A='#777777',B='#cc8a00',C='#067caf',D='#9858a3')
for ax,scene in zip(axs.flat,scenes):
 seed=13 if scene=='sudden_stop' else 29
 for group in 'ABCD':
  name=f'{scene}_{group}_{seed:02d}_aligned';p=CACHE/'runs'/name;e=json.loads((p/'events.json').read_text());rows=[r for r in jsonlines(p/'truth.jsonl') if e['goal_ns']<=r['source_ns']<=e['end_ns']];robot=[r for r in rows if r['model']=='rm_sentry_2027'];m=json.loads((p/'metrics.json').read_text())
  ax.plot([r['position'][0] for r in robot],[r['position'][1] for r in robot],color=colors[group],label=group+(' contact' if e['contacts'] else ' envelope fail' if not m['safe_success'] else ' safe'))
  if e['contacts'] and robot:ax.plot(*robot[-1]['position'][:2],marker='x',color=colors[group],markersize=9)
  if group=='C':
   actor=[r for r in rows if r['model']=='moving_obstacle'];ax.plot([r['position'][0] for r in actor],[r['position'][1] for r in actor],':',color='black',linewidth=1,label='actor actual')
   if not m['safe_success'] and robot:
    # Mechanical failure envelope at the greatest lateral excursion.
    r=max(robot,key=lambda x:max(abs(y) for _,y in x['polygon']));ax.add_patch(Polygon(r['polygon'],facecolor='none',edgecolor='#ed4b22',linewidth=1.5))
 if scene in ('narrow','wait_release','stale'):
  for y in [-.8,.8]:
   for a,b in [(-1.5,1.65),(2.35,5.5)]:ax.add_patch(Polygon([[a,y-.1],[b,y-.1],[b,y+.1],[a,y+.1]],color='#cccccc'))
 ax.plot(4,0,'*',color='green',markersize=9);ax.set(xlabel='x (m)',ylabel='y (m)',title=f'{scene}, seed {seed}, input-pose epoch',xlim=(-.5,4.5),ylim=(-1.7,1.7));ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='lower left')
fig.savefig(EVIDENCE/'closed_loop_examples.png',dpi=155);fig.savefig(EVIDENCE/'closed_loop_examples.svg');plt.close(fig)
s=json.loads((EVIDENCE/'summary.json').read_text());fig,ax=plt.subplots(figsize=(11,4),constrained_layout=True)
for group in 'ABCD':
 for shift,variant,marker in [(-.08,'consume','o'),(.08,'input_pose','^')]:
  for i,scene in enumerate(scenes):
   rs=[r for r in s['trials'] if r['scene']==scene and r['group']==group and r['variant']==variant and r['seed'] in (13,29) and r['cohort']=='final_fixture' and r['phase']=='finite']
   for seed in [13,29]:
    r=next(r for r in rs if r['seed']==seed)
    if r['success']:ax.scatter(i+shift+(seed-21)/180,r['arrival_s'],color=colors[group],marker=marker,facecolors=colors[group] if r['safe_success'] else 'none',s=45)
ax.set_xticks(range(len(scenes)),scenes);ax.set_ylabel('Arrival time (s), goals only');ax.grid(axis='y',alpha=.2);ax.set_title('Original circles / input-pose triangles; hollow = envelope safety fail; contact failures in tables')
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([],[],color=c,marker='o',linestyle='',label=g) for g,c in colors.items()],ncol=4)
fig.savefig(EVIDENCE/'paired_arrival.png',dpi=155);plt.close(fig)
