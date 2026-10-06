"""Independent paired timing audit of the actual actor trajectory."""
import bisect,csv,json,math,pathlib,sys
from analyze import stats
root=pathlib.Path(sys.argv[1]);runs={}
replacement_file=root/'replacement_runs.json'
replacements=json.loads(replacement_file.read_text()) if replacement_file.exists() else {}
for p in (root/'runs').iterdir():
    if (p/'metrics.json').exists():
        r=json.loads((p/'metrics.json').read_text())
        if r['phase']=='finite' and not r['startup']: runs[p.name]=r
def interpolate(samples,t):
    times=[r[0] for r in samples];i=bisect.bisect_left(times,t)
    if i==0 or i==len(samples): return None
    a,b=samples[i-1:i+1];f=(t-a[0])/(b[0]-a[0])
    return [x+f*(y-x) for x,y in zip(a[1],b[1])]
pairs=[]
for scene,count in [('S0',5),('S1',10),('S2',10)]:
    for repeat in range(101,101+count):
        b_name=replacements.get(f'{scene}:B0:{repeat}',f'{scene}:B0:{repeat}').replace(':','_')
        r_name=replacements.get(f'{scene}:R4:{repeat}',f'{scene}:R4:{repeat}').replace(':','_')
        b=runs.get(b_name);r=runs.get(r_name)
        if not b or not r: continue
        delta=[];velocity_delta=[]
        horizon=min(b['elapsed_s'],r['elapsed_s'],9.)
        for k in range(1,int(horizon*10)):
            bp=interpolate(b['actor_timed_samples'],k/10);rp=interpolate(r['actor_timed_samples'],k/10)
            if bp is not None and rp is not None: delta.append(math.dist(bp,rp))
            b0=interpolate(b['actor_timed_samples'],k/10-.05);b1=interpolate(b['actor_timed_samples'],k/10+.05)
            r0=interpolate(r['actor_timed_samples'],k/10-.05);r1=interpolate(r['actor_timed_samples'],k/10+.05)
            if all(v is not None for v in (b0,b1,r0,r1)):
                velocity_delta.append(math.dist([(y-x)/.1 for x,y in zip(b0,b1)],[(y-x)/.1 for x,y in zip(r0,r1)]))
        pairs.append(dict(scene=scene,repeat=repeat,b0_run=b_name,r4_run=r_name,actor_delta_m=stats(delta),actor_velocity_delta_mps=stats(velocity_delta),common_trajectory_s=horizon,
            robot_start_delta_m=math.dist(b['start_position'],r['start_position']) if b['start_position'] and r['start_position'] else None,
            baseline_success=b['success'],r4_success=r['success'],
            b0_goal_ns=b['goal_ns'],r4_goal_ns=r['goal_ns']))
(root/'fairness.json').write_text(json.dumps(pairs,indent=2)+'\n')
print(json.dumps(pairs,indent=2))
