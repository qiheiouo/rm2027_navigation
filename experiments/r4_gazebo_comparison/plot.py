"""Small scientific figure from recorded physical trajectories and outcomes."""
import json,pathlib,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze import polygon_distance
root=pathlib.Path(sys.argv[1]);summary=json.loads((root/'summary.json').read_text())
colors={'B0':'#2368a0','R4':'#d87520'}
fig,axes=plt.subplots(2,3,figsize=(12,7),constrained_layout=True)
for j,scene in enumerate(('S0','S1','S2')):
    for mode in ('B0','R4'):
        group=[r for r in summary['runs'] if r['phase']=='finite' and not r['startup'] and r['scene']==scene and r['mode']==mode]
        index=0 if mode=='B0' else 1
        for k,r in enumerate(group):
            if r['arrival_s'] is not None: axes[0,j].plot(index+(k-(len(group)-1)/2)*.025,r['arrival_s'],'o',color=colors[mode],alpha=.7)
        fails=sum(not r['success'] for r in group)
        axes[0,j].text(index,0.02,f'{len(group)-fails}/{len(group)} success',transform=axes[0,j].get_xaxis_transform(),ha='center',color=colors[mode],fontsize=9)
        if scene!='S0':
            for k,r in enumerate(group):
                if r['min_dynamic_clearance_m'] is not None:
                    axes[1,j].plot(index+(k-(len(group)-1)/2)*.025,r['min_dynamic_clearance_m'],'o' if r['success'] else 'x',color=colors[mode])
        else:
            # All successful empty-scene progress traces. Oracle remains observation only.
            for r in group:
                t=[];x=[]
                for line in (root/'runs'/r['run']/'truth.jsonl').open():
                    v=json.loads(line)
                    if v['model']=='rm_sentry_2027' and v['source_ns']>=r['goal_ns']:
                        t.append((v['source_ns']-r['goal_ns'])/1e9);x.append(v['position'][0])
                axes[1,j].plot(t,x,color=colors[mode],alpha=.35,lw=1)
    axes[0,j].set_title(scene);axes[0,j].set_xticks([0,1],['STVL + MPPI','R4 XY + native wz']);axes[0,j].set_xlim(-.4,1.4);axes[0,j].set_ylim(bottom=0);axes[0,j].set_ylabel('Arrival time, successful runs (s)')
    if scene=='S0': axes[1,j].set_xlabel('Time from accepted goal (s)');axes[1,j].set_ylabel('Physical world x (m)')
    else:
        axes[1,j].set_xticks([0,1],['STVL + MPPI','R4 XY + native wz']);axes[1,j].set_xlim(-.4,1.4);axes[1,j].set_ylabel('Sampled mechanical clearance (m)');axes[1,j].set_ylim(bottom=-.015)
    for ax in axes[:,j]: ax.grid(alpha=.2)
fig.suptitle('A23 finite Gazebo comparison — frozen A22 consumption; x = failed/censored run',fontsize=12)
fig.savefig(root/'comparison.png',dpi=160);plt.close(fig)
# First physical failure, with actual ego timing visible instead of spatial-only claims.
paths={'B0':root/'runs/S1_B0_104','R4':root/'runs/S1_R4_104'}
if all((p/'events.json').exists() for p in paths.values()):
    fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
    for mode,p in paths.items():
        e=json.loads((p/'events.json').read_text());truth={}
        for line in (p/'truth.jsonl').open():
            v=json.loads(line)
            if e['goal_ns']<=v['source_ns']<=e['end_ns']: truth.setdefault(v['source_ns'],{})[v['model']]=v
        robot=[(ns,v['rm_sentry_2027']['position']) for ns,v in sorted(truth.items()) if 'rm_sentry_2027' in v]
        axes[0].plot([v[0] for ns,v in robot],[v[1] for ns,v in robot],label=mode,color=colors[mode])
        common=[(ns,v) for ns,v in sorted(truth.items()) if len(v)==2]
        axes[1].plot([(ns-e['goal_ns'])/1e9 for ns,v in common],[polygon_distance(v['rm_sentry_2027']['polygon'],v['moving_obstacle']['polygon']) for ns,v in common],label=mode,color=colors[mode])
    axes[0].plot([2,2],[-.9,.9],'k--',label='actor center trajectory');axes[0].set(xlabel='world x (m)',ylabel='world y (m)');axes[0].axis('equal')
    axes[1].axhline(0,color='black',lw=.7);axes[1].set(xlabel='time from accepted goal (s)',ylabel='sampled mechanical clearance (m)',ylim=(-.03,1.6))
    for ax in axes: ax.grid(alpha=.2);ax.legend()
    fig.suptitle('S1 pair 104: baseline actor / front-left-wheel contact at t=4.876s')
    fig.savefig(root/'first_collision.png',dpi=160);plt.close(fig)
