"""Recorded A26 geometry, trajectories and finite paired outcomes."""
import json,pathlib,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

root=pathlib.Path(sys.argv[1]);s=json.loads((root/'summary.json').read_text());spec=json.loads((root/'assets/scenario.json').read_text());audit=json.loads((root/'corridor_audit.json').read_text())
colors={'B0':'#2368a0','R4':'#d87520'}
replacements=json.loads((root/'replacement_runs.json').read_text()) if (root/'replacement_runs.json').exists() else {}
display_repeat={v.replace(':','_'):int(k.split(':')[-1])-100 for k,v in replacements.items()}
fig,axes=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
geo=axes[0,0]
for a,b,c,d in spec['walls']:geo.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#737d86'))
sx,sy,sz=spec['actor_size'];geo.add_patch(Rectangle((spec['gate_x']-sx/2,-sy/2),sx,sy,facecolor='#34393f',alpha=.25,label='Actor at centre'))
geo.plot([2,2],[-1.4,1.4],'k--',alpha=.4)
selected=[r for r in s['runs'] if r['phase']=='finite' and not r['startup']]
for mode in ('B0','R4'):
    runs=sorted([r for r in selected if r['mode']==mode],key=lambda r:display_repeat.get(r['run'],int(r['run'].split('_')[-1])-100))
    for j,r in enumerate(runs):
        samples=[]
        for line in (root/'runs'/r['run']/'truth.jsonl').open():
            v=json.loads(line)
            if v['model']=='rm_sentry_2027' and r['goal_ns']<=v['source_ns']<=r['goal_ns']+r['elapsed_s']*1e9:samples.append(((v['source_ns']-r['goal_ns'])/1e9,v['position']))
        geo.plot([p[0] for t,p in samples],[p[1] for t,p in samples],color=colors[mode],alpha=.6,label=mode if j==0 else None)
        if j==0:axes[1,2].plot([t for t,p in samples],[p[0] for t,p in samples],color=colors[mode],label=mode+' trial 101')
    repeat=[display_repeat.get(r['run'],int(r['run'].split('_')[-1])-100) for r in runs]
    if any(r['arrival_s'] is not None for r in runs):axes[0,1].plot(repeat,[r['arrival_s'] for r in runs],'-o',color=colors[mode],label=mode)
    axes[0,2].plot(repeat,[r['min_dynamic_clearance_m'] for r in runs],'-o',color=colors[mode],label=mode)
    axes[1,0].plot(repeat,[r['wait_s'] for r in runs],'-o',color=colors[mode],label=mode)
    if any(r['signed_clear_to_pass_s'] is not None for r in runs):axes[1,1].plot(repeat,[r['signed_clear_to_pass_s'] for r in runs],'-o',color=colors[mode],label=mode)
geo.set(xlabel='World x (m)',ylabel='World y (m)',xlim=(-.5,4.5),ylim=(-1.8,1.8),aspect='equal',title='Same closed channel; all finite trajectories')
axes[0,1].set(xlabel='Paired trial',ylabel='Arrival time (s)')
axes[0,2].set(xlabel='Paired trial',ylabel='Minimum dynamic mechanical clearance (m)')
axes[1,0].set(xlabel='Paired trial',ylabel='WAIT below 0.02 m/s (s)',ylim=(0,None))
axes[1,1].set(xlabel='Paired trial',ylabel='Full-circle gate pass minus actual clear (s)')
axes[1,1].axhline(0,color='black',lw=.6)
axes[1,2].set(xlabel='Time since goal (s)',ylabel='Actual world-forward position (m)')
axes[1,2].axhline(spec['gate_x']+.225+.43869,color='black',lw=.6,ls='--')
for ax in axes.flat:
    ax.grid(alpha=.2)
    if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8)
    elif ax in (axes[0,1],axes[1,1]):ax.text(.5,.5,'Not observed: trials terminated',ha='center',va='center',transform=ax.transAxes)
fig.suptitle('A26 first-error-abort cohort: WAIT/GO outcome not observed')
fig.savefig(root/'corridor.png',dpi=160);plt.close(fig)
