"""Four focused proxy trajectories; no video or simulated new-car CAD claim."""
import os,pathlib,json,sys
os.environ['MPLCONFIGDIR']='/tmp/rm2027_roots_mpl'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from data import jsonlines
from inventory import OLD,walls
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_temporal_roots/evidence'
cases=['wait_release_C_29_aligned','stale_C_29_aligned','narrow_D_47_aligned','narrow_C_29_aligned'];fig,axs=plt.subplots(2,2,figsize=(11,6.5))
for ax,name in zip(axs.flat,cases):
 original=OLD/'build/stvl_cache/runs'/name
 for wall in walls(original):ax.add_patch(Polygon(wall,color='.80'))
 for label,path,color in [('frozen original',original,'#bd3740'),('geometry + storage',ROOT/'build/roots_cache/repair_abi/runs'/name,'#397db8'),('+ measured velocity',ROOT/'build/roots_cache/repair_odom/runs'/name,'#208055')]:
  e=json.loads((path/'events.json').read_text());rows=[r for r in jsonlines(path/'truth.jsonl') if r['model']=='rm_sentry_2027' and e['goal_ns']<=r['source_ns']<=e['end_ns']];ax.plot([r['position'][0] for r in rows],[r['position'][1] for r in rows],color=color,label=label,lw=1.5)
 actor=[r for r in jsonlines(original/'truth.jsonl') if r['model']=='moving_obstacle'];ax.plot([r['position'][0] for r in actor],[r['position'][1] for r in actor],color='.4',ls=':',lw=1,label='actor (original truth)')
 ax.set_title(name,fontsize=10);ax.set_xlim(-.2,4.1);ax.set_ylim(-1.1,1.1);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map x [m]');ax.set_ylabel('map y [m]')
axs[0,0].legend(loc='upper left',fontsize=7);fig.suptitle('Gazebo software proxy: paired paths, unchanged footprint / gates',fontsize=12);fig.tight_layout();fig.savefig(OUT/'focused_paths.png',dpi=160);plt.close(fig)
from analyze import polygon_distance
fig,axs=plt.subplots(1,2,figsize=(11,3.8))
for cohort,color,label in [('heldout_abi','#bd3740','geometry, native speed=0'),('heldout_odom','#208055','geometry + measured speed')]:
 p=ROOT/'build/roots_cache'/cohort/'runs/narrow_D_61_aligned';e=json.loads((p/'events.json').read_text());by={}
 for r in jsonlines(p/'truth.jsonl'):
  if e['goal_ns']<=r['source_ns']<=e['end_ns']:by.setdefault(r['source_ns'],{})[r['model']]=r
 common=[(t,v) for t,v in sorted(by.items()) if 'rm_sentry_2027' in v and 'moving_obstacle' in v];times=[(t-e['goal_ns'])/1e9 for t,v in common];gaps=[polygon_distance(v['rm_sentry_2027']['polygon'],v['moving_obstacle']['polygon']) for t,v in common];axs[1].plot(times,gaps,color=color,label=label)
 rr=[v['rm_sentry_2027'] for t,v in common];axs[0].plot([r['position'][0] for r in rr],[r['position'][1] for r in rr],color=color,label=label)
 for w in walls(p):
  if cohort=='heldout_abi':axs[0].add_patch(Polygon(w,color='.8'))
 if cohort=='heldout_abi':
  first=next((item for item,g in zip(common,gaps) if g<.047),None)
  if first:
   t,v=first;cr=[c for c in jsonlines(p/'control.jsonl') if c['epoch_ns']<=t];(OUT/'heldout_D61_negative.json').write_text(json.dumps({'case':p.name,'cohort':cohort,'first_gap_below_frozen_047_ns':t,'relative_s':(t-e['goal_ns'])/1e9,'robot':v['rm_sentry_2027'],'actor':v['moving_obstacle'],'last_control':{k:v for k,v in cr[-1].items() if k not in ('raw','root_diagnostics','critic_deltas')},'sensor_contact_count':len(e['contacts']),'unchanged_gate':'failed despite no sensor contact; retain full-envelope metric'},indent=2)+'\n')
axs[0].set_xlim(-.2,4.1);axs[0].set_ylim(-1.1,1.1);axs[0].set_aspect('equal');axs[0].set_xlabel('map x [m]');axs[0].set_ylabel('map y [m]');axs[1].axhline(.047,color='black',ls='--',label='frozen .047 m gate');axs[1].set_xlabel('time since accepted goal [s]');axs[1].set_ylabel('sampled full-bbox dynamic gap [m]');axs[1].legend(fontsize=7);fig.suptitle('Oracle seed 61: reached goal is insufficient for safety acceptance');fig.tight_layout();fig.savefig(OUT/'heldout_oracle_61.png',dpi=160);plt.close(fig)
