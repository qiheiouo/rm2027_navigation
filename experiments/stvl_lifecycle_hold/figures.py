"""Publication-style static plots; physical data and bounds remain explicit."""
import pathlib,json,gzip
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2];P=ROOT/'experiments/stvl_lifecycle_hold/evidence'
def read(n):
 p=P/n
 return json.loads(p.read_text() if p.exists() else gzip.open(str(p)+'.gz','rt').read())
a=read('fixture_summary.json');tim=next(x for x in a if x['label']=='timing_retry');names=['normal','single_100ms','three_period_300ms','missing_cycle','silence_1500ms'];v={x['case']:x for x in tim['profiles']};fig,ax=plt.subplots(figsize=(9,4));ax.bar(names,[v[n]['displacement_request_to_stop_m']*100 for n in names]);ax.set_ylabel('actual distance from logical stop to body stop (cm)');ax.tick_params(axis='x',rotation=15);ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(P/'delay_distance.png',dpi=130);plt.close(fig)
fig,ax=plt.subplots(figsize=(9,4))
for n in ('reference','brake','brake_delayed'):
 x=read(n+'_metrics.json');ax.plot([g['relative_s'] for g in x['gaps']],[g['gap_m'] for g in x['gaps']],label=n)
ax.axhline(.047,color='r',linestyle='--',label='original gate .047m');ax.axvline(2.522493,color='gray',linestyle=':');ax.set_xlabel('seconds after frozen execution epoch');ax.set_ylabel('mechanical bbox dynamic gap (m)');ax.set_title('Execution-only fixture: early braking vs fixed upstream continuation');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(P/'brake_gap.png',dpi=130)
