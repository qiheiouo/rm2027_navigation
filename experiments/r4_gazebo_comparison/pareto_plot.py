"""A25 finite-sample clearance/efficiency chart from recorded outcomes only."""
import json,pathlib,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=pathlib.Path(sys.argv[1]);summary=json.loads((root/'summary.json').read_text());audit=json.loads((root/'pareto_audit.json').read_text())
colors={'B0':'#2368a0','R4':'#d87520'}
fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
axes[0].axvspan(.28,.32,color='#bbc5be',alpha=.3,label='Target median band')
for mode in ('B0','R4'):
    group=[r for r in summary['runs'] if r['phase']=='finite' and not r['startup'] and r['mode']==mode]
    successful=[r for r in group if r['arrival_s'] is not None]
    axes[0].scatter([r['min_dynamic_clearance_m'] for r in successful],[r['arrival_s'] for r in successful],color=colors[mode],label=mode,alpha=.7)
    g=next(g for g in summary['groups'] if g['mode']==mode)
    if g['arrival_s']: axes[0].scatter(g['min_clearance_m']['p50'],g['arrival_s']['p50'],marker='*',s=150,color=colors[mode],edgecolors='black',linewidths=.5)
    indices=range(1,len(audit['pairs'])+1);prefix='baseline' if mode=='B0' else 'r4'
    axes[1].plot(list(indices),[r[prefix+'_arrival_s'] for r in audit['pairs']],'-o',color=colors[mode],label=mode)
    axes[2].plot(list(indices),[r[prefix+'_wait_s'] for r in audit['pairs']],'-o',color=colors[mode],label=mode)
axes[0].set(xlabel='Sampled mechanical minimum clearance (m)',ylabel='Arrival time (s)')
axes[1].set(xlabel='Independent paired trial',ylabel='Arrival time (s)')
axes[2].set(xlabel='Independent paired trial',ylabel='WAIT below 0.02 m/s (s)',ylim=(0,None))
for ax in axes: ax.grid(alpha=.2);ax.legend()
fig.suptitle(f"A25 S2 clearance / efficiency: {audit['verdict']} — stars mark group medians")
fig.savefig(root/'pareto.png',dpi=160);plt.close(fig)
