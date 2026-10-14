"""Static research plots; all traces retained in linked evidence."""
import pathlib,json,gzip
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=pathlib.Path(__file__).resolve().parent/'evidence';o=json.loads((p/'original_forensics.json').read_text());c=o['cycles'];g=o['actual_gaps'];fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax[0].plot([x['relative_s'] for x in c if x['command']],[x['command'][0] for x in c if x['command']],'.-',label='native output vx')
ax[0].plot([x['relative_s'] for x in c],[x['speed'][0] for x in c],label='input filtered body vx')
ax[0].set_ylabel('m/s');ax[0].legend();ax[1].plot([x['relative_s'] for x in g],[x['gap_m'] for x in g],label='recorded mechanical bbox clearance');ax[1].axhline(.047,color='r',linestyle='--',label='frozen gate .047m');ax[1].set_ylabel('m');ax[1].set_xlabel('seconds after original C83 goal');ax[1].legend()
for a in ax:
 for t,label in [(2.514,'zero-safe'),(3.514,'first native fail'),(5.308,'contact')]:a.axvline(t,color='grey',alpha=.5)
 a.grid(alpha=.2)
fig.suptitle('Original C83: risky motion precedes late stop and actor contact');fig.tight_layout();fig.savefig(p/'original_c83.png',dpi=140);plt.close(fig)
fig,ax=plt.subplots(figsize=(9,5))
for cohort,label in [('shadow','C83 initial shadow'),('shadow_durable','C83 durable shadow'),('normal','D29 shadow')]:
 q=p/f'{cohort}_paired_cycles.json';a=json.loads(q.read_text() if q.exists() else gzip.open(str(q)+'.gz','rt').read());ax.scatter([x['T_mppi_disjoint_ms'] for x in a],[x['T_total_necessary_boundary_ms'] for x in a],s=10,alpha=.6,label=label)
ax.axhline(100,color='r',linestyle='--',label='100ms whole-cycle budget');ax.set_xlabel('same-cycle MPPI ms (added raw read separated)');ax.set_ylabel('entry to completed all-candidate shadow ms');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(p/'same_cycle.png',dpi=140)
