import pathlib,json,csv,os,numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/rm-realtime-matplotlib')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_execution_realtime/evidence';CACHE=ROOT/'build/realtime_cache'
full={r['case']:r for r in json.loads((OUT/'performance_summary.json').read_text())['runs']};simple={r['case']:r for r in json.loads((OUT/'classification_summary.json').read_text())['runs']}
cases=['diag__narrow_D_29_aligned','repair_odom__wait_release_C_13_aligned','fresh__crossing_C_83_aligned__6627000000'];x=np.arange(3)
fig,ax=plt.subplots(figsize=(8,4))
for off,key,label in [(-.24,'reference','Reference full telemetry'),(0,'optimized','Prepared full telemetry'),(.24,'classification','Same predicate / witnesses')]:
    values=[simple[k]['time_ms']['p99'] if key=='classification' else full[k]['timing'][key]['p99'] for k in cases];ax.bar(x+off,values,.23,label=label)
ax.axhline(100,color='red',linestyle='--',label='Entire controller cycle 100ms');ax.set_xticks(x,['Oracle wall failure','Waiting C13','Held-out C83']);ax.set_ylabel('Isolated checker p99 (ms)');ax.set_title('Same coverage/thresholds; excluded I/O and native controller cost are separate');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(OUT/'performance.png',dpi=150);plt.close(fig)
from mapping import rows
p=CACHE/'corrected_scene/runs/narrow_C_83_aligned';e=json.loads((p/'events.json').read_text());start=e['goal_ns'];end=e['end_ns'];native=[r for r in rows(p/'native_boundary.csv') if start<=int(r['sim_ns'])<=end];od=[r for r in rows(p/'odometry.csv') if start<=int(r['source_ns'])<=end]
fig,axes=plt.subplots(2,1,figsize=(8,5),sharex=True)
for ax,key,label in [(axes[0],'vx','Body forward velocity / target (m/s)'),(axes[1],'wz','Yaw rate / target (rad/s)')]:
    ax.plot([(int(r['sim_ns'])-start)/1e9 for r in native],[float(r['applied_'+key]) for r in native],label='native limited wheel target',alpha=.7)
    ax.plot([(int(r['source_ns'])-start)/1e9 for r in od],[float(r[key]) for r in od],label='body pose-derived feedback (window 10)',linewidth=1)
    ax.set_ylabel(label);ax.legend(fontsize=8);ax.grid(alpha=.25)
axes[1].set_xlabel('seconds after goal');fig.suptitle('C83 original-physics replay: filtered body estimate vs native wheel target');fig.tight_layout();fig.savefig(OUT/'execution_feedback.png',dpi=150);plt.close(fig)
