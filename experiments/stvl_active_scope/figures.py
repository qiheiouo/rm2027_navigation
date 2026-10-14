"""Four actual executed paths; frozen proxy geometry, no synthetic trajectories."""
import pathlib,json,gzip,csv,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/active_scope_cache';OUT=ROOT/'experiments/stvl_active_scope/evidence'
fig,axes=plt.subplots(1,2,figsize=(10,4.5))
for ax,scene,seed in zip(axes,('crossing','narrow'),(13,83)):
 for kind in ('full','active'):
  p=CACHE/kind/'runs'/f'{scene}_C_{seed:02d}_aligned';e=json.loads((p/'events.json').read_text());od=[x for x in csv.DictReader(gzip.open(p/'odometry.csv.gz','rt')) if e['goal_ns']<=int(x['source_ns'])<=e['end_ns']];ax.plot([float(x['x']) for x in od],[float(x['y']) for x in od],label=kind)
  if kind=='full':
   actor=[json.loads(l) for l in gzip.open(p/'truth.jsonl.gz','rt')];actor=[x for x in actor if x['model']=='moving_obstacle' and e['goal_ns']<=x['source_ns']<=e['end_ns']];ax.plot([x['position'][0] for x in actor],[x['position'][1] for x in actor],'--',color='gray',label='actor center')
 ax.set_title(scene+' (one paired noise seed)');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend()
fig.suptitle('Executed proxy paths: active input scope does not establish navigation improvement');fig.tight_layout();fig.savefig(OUT/'executed_paths.png',dpi=130)
