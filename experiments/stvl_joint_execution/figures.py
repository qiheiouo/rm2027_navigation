"""Small scientific figures from recorded execution; no controller input."""
import pathlib,json,csv,sys,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_joint_execution/evidence';CACHE=ROOT/'build/joint_cache/fresh/runs'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
def odometry(p):return list(csv.DictReader((p/'odometry.csv').open()))
fig,axes=plt.subplots(1,3,figsize=(12,3.7))
for ax,names,title in zip(axes,[['crossing_A_29_aligned','crossing_B_29_aligned','crossing_C_29_aligned','crossing_D_29_aligned'],['wait_release_C_13_aligned','narrow_C_29_aligned'],['crossing_C_83_aligned','narrow_C_83_aligned']],['Paired crossing / seed 29','Waiting and narrow passage','Held-out seed 83']):
    for name in names:
        p=CACHE/name;e=json.loads((p/'events.json').read_text());r=[x for x in odometry(p) if e['goal_ns']<=int(x['source_ns'])<=e['end_ns']]
        ax.plot([float(x['x']) for x in r],[float(x['y']) for x in r],label=name.replace('_aligned',''))
        if e['contacts']:ax.scatter(float(r[-1]['x']),float(r[-1]['y']),s=55,marker='x',color='red')
    ax.set_title(title);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.25);ax.legend(fontsize=7);ax.set_aspect('equal',adjustable='datalim')
fig.suptitle('Recorded proxy robot paths; red cross = actual actor contact; no C-prime intervention');fig.tight_layout();fig.savefig(OUT/'fresh_paths.png',dpi=160);plt.close(fig)
p=CACHE/'crossing_D_29_aligned';e=json.loads((p/'events.json').read_text());start=e['goal_ns'];od=[r for r in odometry(p) if start<=int(r['source_ns'])<=start+1200000000];commands=list(csv.DictReader((p/'commands.csv').open()))
fig,axes=plt.subplots(2,1,figsize=(8,5),sharex=True)
for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel'):
    r=[r for r in commands if r['topic']==topic and start<=int(r['receipt_ns'])<=start+1200000000];axes[0].step([(int(x['receipt_ns'])-start)/1e9 for x in r],[float(x['vx']) for x in r],where='post',label=topic,alpha=.75)
axes[0].plot([(int(r['source_ns'])-start)/1e9 for r in od],[float(r['vx']) for r in od],label='measured body vx',color='black',linewidth=1);axes[0].set_ylabel('vx (m/s)');axes[0].legend(fontsize=8);axes[0].grid(alpha=.25)
a=[abs(float(b['vx'])-float(x['vx']))/((int(b['source_ns'])-int(x['source_ns']))/1e9) for x,b in zip(od,od[1:])];axes[1].plot([(int(x['source_ns'])-start)/1e9 for x in od[1:]],a,label='20ms feedback difference');axes[1].axhline(.8,color='red',linestyle='--',label='smoother 0.8');axes[1].axhline(2,color='gray',linestyle=':',label='proxy SDF 2.0');axes[1].set_ylabel('|dvx/dt| (m/s²)');axes[1].set_xlabel('seconds after goal');axes[1].legend(fontsize=8);axes[1].grid(alpha=.25)
fig.suptitle('Early execution before any recovery; receipts are not actuator timestamps');fig.tight_layout();fig.savefig(OUT/'execution_steps.png',dpi=160);plt.close(fig)
