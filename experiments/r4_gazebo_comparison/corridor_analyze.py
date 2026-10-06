"""A26 observer metrics and preregistered decision; never feeds control."""
import bisect,json,math,pathlib,subprocess,sys
import yaml
from analyze import aggregate,rows,stats,polygon_distance

root=pathlib.Path(sys.argv[1]);aggregate(root)
spec=json.loads((root/'assets/scenario.json').read_text())
summary=json.loads((root/'summary.json').read_text());radius=math.hypot(.32,.27)+.02
wall_polygons=[[(a,c),(b,c),(b,d),(a,d)] for a,b,c,d in spec['walls']]

for r in summary['runs']:
    if r['startup']: continue
    directory=root/'runs'/r['run'];start=r['goal_ns'];end=start+int(r['elapsed_s']*1e9)
    run_spec=json.loads((directory/'scenario.json').read_text()) if (directory/'scenario.json').exists() else spec
    wall_polygons=[[(a,c),(b,c),(b,d),(a,d)] for a,b,c,d in run_spec['walls']]
    truth={}
    for line in (directory/'truth.jsonl').open():
        v=json.loads(line);ns=v['source_ns']
        if start<=ns<=end: truth.setdefault(ns,{})[v['model']]=v
    ordered=sorted(truth.items());robot=[(ns,v['rm_sentry_2027']) for ns,v in ordered if 'rm_sentry_2027' in v]
    actor=[((ns-start)/1e9,v['moving_obstacle']['position']) for ns,v in ordered if 'moving_obstacle' in v]
    clear=None;entered=False
    for ns,v in ordered:
        if 'moving_obstacle' not in v: continue
        ys=[p[1] for p in v['moving_obstacle']['polygon']]
        if min(ys)<=radius and max(ys)>=-radius: entered=True
        if entered and min(ys)>radius: clear=ns;break
    passed=next((ns for ns,v in robot if v['position'][0]>spec['gate_x']+.225+radius),None)
    wall_clearance=min((polygon_distance(v['polygon'],w) for ns,v in robot for w in wall_polygons),default=None)
    xy=[v['position'][:2] for ns,v in robot]
    distance=sum(math.dist(a,b) for a,b in zip(xy,xy[1:]));lateral=sum(abs(a[1]-b[1]) for a,b in zip(xy,xy[1:]))
    excursions=0;inside=False
    for ns,v in robot:
        active=(passed is None or ns<passed) and abs(v['position'][1])>.4
        if active and not inside: excursions+=1
        inside=active
    plans=[json.loads(line) for line in (directory/'plans.jsonl').open()]
    controls=[v for v in rows(directory/'control.csv') if start<=int(v['epoch_ns'])<=end]
    valid_controls=[v for v in controls if v['valid']=='1']
    predictions=0;moving_predictions=0
    # Public observed CV near the gate. This geometric selection is analysis only.
    for line in (directory/'prediction.jsonl').open():
        m=json.loads(line)['message']['prediction']
        near=[t for t in m['tracks'] if abs(t['position']['x']-spec['gate_x'])<.45 and abs(t['position']['y'])<1.4]
        predictions+=bool(near)
        moving_predictions+=any(t['state']>=2 and t['velocity']['y']>.15 for t in near)
    resume=None;consecutive=0;first=None
    if clear is not None:
        for v in rows(directory/'odometry.csv'):
            ns=int(v['source_ns'])
            if ns<clear or ns>end: continue
            yaw=float(v['yaw']);forward=math.cos(yaw)*float(v['vx'])-math.sin(yaw)*float(v['vy'])
            if forward>.05:
                if consecutive==0:first=ns
                consecutive+=1
                if consecutive>=3:resume=max(0.,(first-clear)/1e9);break
            else:consecutive=0;first=None
    r.update(actual_clear_s=(clear-start)/1e9 if clear else None,actual_clear_resume_s=resume,gate_pass_s=(passed-start)/1e9 if passed else None,
             signed_clear_to_pass_s=(passed-clear)/1e9 if clear and passed else None,already_passed_at_clear=passed<clear if clear and passed else None,
             min_static_wall_clearance_m=wall_clearance,travel_distance_m=distance,lateral_distance_m=lateral,max_lateral_excursion_m=max((abs(p[1]) for p in xy),default=None),
             lateral_escape_attempts=excursions,plan_messages=len(plans),plan_pocket_messages=sum(any(abs(p[1])>.4 for p in v['xy']) for v in plans),
             gate_prediction_messages=predictions,gate_nonzero_cv_messages=moving_predictions,
             native_unavailable_cycles=sum(v['reason']=='Optimizer fail to compute path' for v in controls),
             solver_ms=stats([float(v['solver_ms']) for v in valid_controls if r['mode']=='R4']),
             native_ms=stats([float(v['native_ms']) for v in valid_controls]),
             r4_call_ms=stats([float(v['total_ms']) for v in valid_controls if r['mode']=='R4']),
             combined_call_ms=stats([float(v['native_ms'])+float(v['total_ms']) for v in valid_controls]))
    metrics=json.loads((directory/'metrics.json').read_text());metrics.update(r);metrics['actor_timed_samples']=actor
    (directory/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')

extra_fields=['gate_pass_s','signed_clear_to_pass_s','actual_clear_resume_s','min_static_wall_clearance_m','travel_distance_m','lateral_distance_m','max_lateral_excursion_m','lateral_escape_attempts','plan_messages','plan_pocket_messages','gate_nonzero_cv_messages']
for g in summary['groups']:
    selected=[r for r in summary['runs'] if r['scene']==g['scene'] and r['mode']==g['mode'] and r['phase']=='finite' and not r['startup']]
    for key in extra_fields:g[key]=stats([r[key] for r in selected if r[key] is not None])
    controls=[]
    for r in selected:
        controls.extend(v for v in rows(root/'runs'/r['run']/'control.csv') if r['goal_ns']<=int(v['epoch_ns'])<=r['goal_ns']+r['elapsed_s']*1e9 and v['valid']=='1')
    g['r4_solver_ms']=stats([float(v['solver_ms']) for v in controls if g['mode']=='R4'])
    g['native_call_ms']=stats([float(v['native_ms']) for v in controls])
    g['r4_consumption_ms']=stats([float(v['total_ms']) for v in controls if g['mode']=='R4'])
    g['combined_call_ms']=stats([float(v['native_ms'])+float(v['total_ms']) for v in controls])
    g['valid_call_samples']=len(controls)
    g['native_unavailable_cycles']=sum(r['native_unavailable_cycles'] for r in selected)
    g['solver_p95_ms']=stats([r['solver_ms']['p95'] for r in selected if r['solver_ms']])
    g['native_p95_ms']=stats([r['native_ms']['p95'] for r in selected if r['native_ms']])
(root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

def interpolate(samples,t):
    i=bisect.bisect_left([s[0] for s in samples],t)
    if i==0 or i==len(samples):return None
    a,b=samples[i-1:i+1];f=(t-a[0])/(b[0]-a[0]);return [x+f*(y-x) for x,y in zip(a[1],b[1])]
finite={r['run']:r for r in summary['runs'] if r['phase']=='finite' and not r['startup']}
replacements=json.loads((root/'replacement_runs.json').read_text()) if (root/'replacement_runs.json').exists() else {}
pairs=[]
for i in range(101,111):
    b=finite.get(replacements.get(f'S3:B0:{i}',f'S3:B0:{i}').replace(':','_'));r=finite.get(replacements.get(f'S3:R4:{i}',f'S3:R4:{i}').replace(':','_'))
    if not b or not r:continue
    bp=json.loads((root/'runs'/b['run']/'metrics.json').read_text())['actor_timed_samples'];rp=json.loads((root/'runs'/r['run']/'metrics.json').read_text())['actor_timed_samples']
    horizon=min(b['elapsed_s'],r['elapsed_s'],10.3333333333);delta=[];velocity=[]
    for k in range(1,int(horizon*10)):
        t=k/10;ba=interpolate(bp,t);ra=interpolate(rp,t)
        if ba and ra:delta.append(math.dist(ba,ra))
        a0=interpolate(bp,t-.05);a1=interpolate(bp,t+.05);b0=interpolate(rp,t-.05);b1=interpolate(rp,t+.05)
        if all(v is not None for v in (a0,a1,b0,b1)):velocity.append(math.dist([(y-x)/.1 for x,y in zip(a0,a1)],[(y-x)/.1 for x,y in zip(b0,b1)]))
    pairs.append(dict(repeat=i,baseline=b['run'],r4=r['run'],baseline_arrival_s=b['arrival_s'],r4_arrival_s=r['arrival_s'],actor_position_delta_m=stats(delta),actor_velocity_delta_mps=stats(velocity),robot_start_delta_m=math.dist(b['start_position'],r['start_position'])))
(root/'fairness.json').write_text(json.dumps(pairs,indent=2)+'\n')

def flatten(v,p=''):
    if isinstance(v,dict):return {k:child for key,value in v.items() for k,child in flatten(value,p+'.'+key if p else key).items()}
    return {p:v}
config={m:yaml.safe_load((root/f'assets/corridor_{m}_nav2.yaml').read_text()) for m in ('B0','R4')}
assert (root/'assets/corridor_R4_nav2.yaml').read_bytes()==(root.parent/'r4_matched_comparison_20261006/assets/matched_R4_nav2.yaml').read_bytes()
bc,rc=map(flatten,(config['B0'],config['R4']));config_delta={k:[rc.get(k),bc.get(k)] for k in rc.keys()|bc.keys() if rc.get(k)!=bc.get(k)}
assert set(config_delta)=={'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius'},config_delta
errors=[];endpoints=[]
for directory in (root/'runs').iterdir():
    if not (directory/'manifest.json').exists():continue
    manifest=json.loads((directory/'manifest.json').read_text());expected=flatten(config[manifest['mode']]);actual=flatten(yaml.safe_load((directory/'nav2.yaml').read_text()))
    unexpected=[k for k in actual.keys()|expected.keys() if actual.get(k)!=expected.get(k) and k not in ('controller_server.ros__parameters.FollowPath.research_mode','controller_server.ros__parameters.FollowPath.research_log')]
    if unexpected:errors.append(dict(run=directory.name,keys=unexpected))
    text=(directory/'output_endpoints.txt').read_text() if (directory/'output_endpoints.txt').exists() else ''
    endpoints.append(dict(run=directory.name,unique_output='Publisher count: 1' in text and 'Node name: chassis_interface_stub' in text,cleanup_minus11='exit code -11' in (directory/'launch.log').read_text()))
assert not errors,errors
groups={g['mode']:g for g in summary['groups'] if g['scene']=='S3'}
decision=dict(verdict='Pending',r4_a24_byte_equal=True,shared_config_delta=config_delta,trial_config_errors=errors,endpoints=endpoints,pairs=pairs,methodology='First-native-error abort cohort: original wrapper permanently latches exceptions and recorder ends on first error. Existing Nav2 retry/recovery and passage after release are not observed; do not infer a general prediction-theory verdict.',scope='Current frozen Research runtime only; final recovery-enabled comparison is separately subject to user approval.')
if all(m in groups for m in ('B0','R4')) and groups['B0']['n']==groups['R4']['n'] and groups['B0']['n'] in (5,10):
    b,r=groups['B0'],groups['R4'];n=b['n'];required=4 if n==5 else 8
    ratio=r['arrival_s']['p50']/b['arrival_s']['p50'] if r['arrival_s'] and b['arrival_s'] else None
    wins=sum(p['r4_arrival_s'] is not None and p['baseline_arrival_s'] is not None and p['r4_arrival_s']<p['baseline_arrival_s'] for p in pairs)
    safety_ok=r['success']==n and r['contacts']==0 and r['success']>=b['success'] and r['min_clearance_m']['p50']>=b['min_clearance_m']['p50']-.05 and r['min_clearance_m']['min']>=b['min_clearance_m']['min']-.05
    possible_go=safety_ok and ratio is not None and ratio<=.9 and wins>=required
    repeated_b_failure=n-b['success']>=(2 if n==5 else 4)
    safety_go=r['success']==n and r['contacts']==0 and repeated_b_failure and ratio is not None and ratio<=1.1
    extend=n==5 and not (possible_go or safety_go) and (ratio is not None and abs(ratio-1)<.1 or wins not in (0,n) or 0<n-b['success']<2 or 0<n-r['success']<2)
    decision.update(verdict='Candidate Go — causal/reactive challenge required' if possible_go or safety_go else 'Modify — fixed samples may change decision' if extend else 'Stop',n_per_mode=n,r4_over_baseline_arrival_ratio=ratio,r4_faster_pairs=wins,efficiency_go_candidate=possible_go,safety_go_candidate=safety_go,extension_informative=extend,r4_safety_comparable=safety_ok,reason='No independently valuable advantage under the final Research criteria.' if not(possible_go or safety_go or extend) else 'Additional preregistered information required; no parameter tuning.')
    decision['frozen_runtime_verdict']=decision['verdict']
    if b['success']==0 and r['success']==0 and r['native_unavailable_cycles']==n and not r['gate_pass_s']:
        decision.update(verdict='Modify — WAIT/GO experiment truncated by first-error abort',reason='No R4 Go established. All R4 trials end on native delegation before release; reuse of existing Nav2 recovery needs user approval. More samples of this abort behavior cannot resolve WAIT/GO value.',extension_informative=False)
(root/'corridor_audit.json').write_text(json.dumps(decision,indent=2)+'\n');print(json.dumps(decision,indent=2))
