"""Static scientific view of two actual trajectories, no CAD/digital twin."""
import gzip,json,pathlib,csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/coasting_live_cache';OUT=ROOT/'experiments/stvl_coasting_live/evidence'
fig,axes=plt.subplots(1,2,figsize=(9,3.7),sharex=True,sharey=True)
for ax,kind in zip(axes,('strict','allow_equal')):
 p=CACHE/kind/'runs/narrow_C_83_aligned';e=json.loads((p/'events.json').read_text());r=json.loads((OUT/(kind+'.json')).read_text())
 odom=list(csv.DictReader(gzip.open(p/'odometry.csv.gz','rt')));odom=[x for x in odom if e['goal_ns']<=int(x['source_ns'])<=e['end_ns']]
 truth=[json.loads(x) for x in gzip.open(p/'truth.jsonl.gz','rt')];actor=[x for x in truth if x['model']=='moving_obstacle' and e['goal_ns']<=x['source_ns']<=e['end_ns']]
 ax.plot([float(x['x']) for x in odom],[float(x['y']) for x in odom],label='Robot (measured odometry)',lw=1.8)
 ax.plot([x['position'][0] for x in actor],[x['position'][1] for x in actor],label='Obstacle center (truth)',color='tab:orange',ls='--')
 ax.scatter([float(odom[0]['x']),float(odom[-1]['x'])],[float(odom[0]['y']),float(odom[-1]['y'])],s=24,c=['green','red'],zorder=3)
 if kind=='allow_equal':
  witness=json.loads((OUT/(kind+'_latency_witness.json')).read_text());xy=witness['controller_record']['pose'];ax.scatter([xy[0]],[xy[1]],s=64,facecolor='none',edgecolor='purple',label='101.60 ms output cycle',zorder=4)
 ax.set_title(('Strict' if kind=='strict' else 'Allow equal')+f"\narrival {r['arrival_s']:.3f} s; min gap {r['body_clearance_m']:.3f} m")
 ax.set_xlabel('Map x [m]');ax.grid(alpha=.25);ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=7,loc='lower left')
axes[0].set_ylabel('Map y [m]');fig.suptitle('C83 pilot: no fresh-coasting equality consumed in either run',fontsize=10);fig.tight_layout();fig.savefig(OUT/'paired_paths.png',dpi=140);plt.close(fig)
