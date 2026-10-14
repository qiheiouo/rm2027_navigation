"""Read-only candidate accounting and retrospective execution audit.

Observed future command receipts are used ONLY to explain past execution.
They are not prediction inputs, a deployable rollout, or actuator timestamps.
"""
import pathlib,json,csv,sys,math,bisect,collections
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_joint_execution/evidence'
OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013/build/roots_cache')
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
FIELDS=('vx','vy','wz')
def readcsv(p):return list(csv.DictReader(p.open()))
def wrap(v):return math.atan2(math.sin(v),math.cos(v))
def interpolate(rows,ts,t):
    i=bisect.bisect_left(ts,t)
    if i==0 or i==len(ts):return None
    a,b=rows[i-1],rows[i];q=(t-ts[i-1])/(ts[i]-ts[i-1]);r=[]
    for k in ('x','y','yaw',*FIELDS):
        d=float(b[k])-float(a[k]);r.append(float(a[k])+q*(wrap(d) if k=='yaw' else d))
    return r
def ramps(commands):
    pairs=[]
    for a,b in zip(commands,commands[1:]):
        dt=(int(b['receipt_ns'])-int(a['receipt_ns']))/1e9
        if 0<dt<=.2:
            pairs.append([(float(b[k])-float(a[k]))/dt for k in FIELDS])
    return {'component_abs_accel': [analyze.stats([abs(r[j]) for r in pairs]) for j in range(3)],
            'period_ms':analyze.stats([(int(b['receipt_ns'])-int(a['receipt_ns']))/1e6 for a,b in zip(commands,commands[1:]) if int(b['receipt_ns'])>int(a['receipt_ns'])])}
def acceleration(od):
    results=[]
    for a,b in zip(od,od[1:]):
        dt=(int(b['source_ns'])-int(a['source_ns']))/1e9
        if 0<dt<=.1:results.append([abs(float(b[k])-float(a[k]))/dt for k in FIELDS])
    return {'absolute':[analyze.stats([r[j] for r in results]) for j in range(3)],
            'exceed_with_tolerance':[sum(r[j]>(.81,.81,2.01)[j] for r in results) for j in range(3)],'intervals':len(results)}
def observed_rollout(state,epoch,commands,ts,delay_ns):
    state=state[:];target=[0.,0.,0.];i=bisect.bisect_right(ts,epoch-delay_ns)-1
    if i>=0:target=[float(commands[i][k]) for k in FIELDS]
    # Exact SDF acceleration upper bound 2 per body component, 1ms proxy.
    for step in range(100):
        now=epoch+step*1000000;i=bisect.bisect_right(ts,now-delay_ns)-1
        if i>=0:target=[float(commands[i][k]) for k in FIELDS]
        for j in range(3):state[j+3]+=max(-.002,min(.002,target[j]-state[j+3]))
        x,y,yaw,vx,vy,w=state;state[0]+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*.001;state[1]+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*.001;state[2]+=w*.001
    return state
def execution(p):
    event=json.loads((p/'events.json').read_text());start,end=event['goal_ns'],event['end_ns']
    if start is None:return {'run':p.name,'not_started':True}
    od=[r for r in readcsv(p/'odometry.csv') if start<=int(r['source_ns'])<=end];ts=[int(r['source_ns']) for r in od]
    commands=readcsv(p/'commands.csv');sim=[r for r in commands if r['topic']=='/simulation/chassis/cmd_vel'];cts=[int(r['receipt_ns']) for r in sim]
    controls=[r for r in jsonlines(p/'control.jsonl') if 'command' in r and start<=r['epoch_ns']<=end];models=[];feedback_errors=[]
    for c in controls:
        epoch=c['pose_epoch_ns'];initial=interpolate(od,ts,epoch);actual=interpolate(od,ts,epoch+100000000)
        if initial is None or actual is None or not c.get('aggregate'):continue
        sdk=c['aggregate'][0];dead=[initial[0]+(math.cos(initial[2])*initial[3]-math.sin(initial[2])*initial[4])*.1,initial[1]+(math.sin(initial[2])*initial[3]+math.cos(initial[2])*initial[4])*.1,initial[2]+initial[5]*.1]
        projections=[observed_rollout(initial,epoch,sim,cts,d) for d in (0,20000000)]
        item={'relative_s':(epoch-start)/1e9,'epoch_ns':epoch,'actual_next_pose':actual[:3],'sdk_next_pose':sdk,'returned_twist':c['command'],'sdk_position_error_m':math.dist(sdk[:2],actual[:2]),'sdk_yaw_error_rad':abs(wrap(sdk[2]-actual[2])),
              'feedback_hold_position_error_m':math.dist(dead[:2],actual[:2]),'observed_target_2_ramp_error_m':[math.dist(s[:2],actual[:2]) for s in projections],
              'observed_target_2_ramp_velocity_error':[math.dist(s[3:],actual[3:]) for s in projections],
              'sdk_vs_actual_next_velocity_error':math.dist(c.get('selected_post_sg_shift_controls',[c['command']])[0],actual[3:])}
        models.append(item);feedback_errors.append(max(abs(a-b) for a,b in zip(c['speed'],initial[3:])))
    recovery=[e['ROS_ns'] for e in event.get('native_mppi_errors',[])];boundary=min(recovery) if recovery else end
    pre=[r for r in od if int(r['source_ns'])<boundary]
    result={'run':p.name,'native_recovery_first_s':(boundary-start)/1e9 if recovery else None,'feedback_max_error':max(feedback_errors,default=None),
            'actual_acceleration':acceleration(od),'pre_first_recovery_actual_acceleration':acceleration(pre),
            'commands_by_topic':{topic:ramps([r for r in commands if r['topic']==topic and start<=int(r['receipt_ns'])<=end]) for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel')},
            'controller_epoch_period_ms':analyze.stats([(b['epoch_ns']-a['epoch_ns'])/1e6 for a,b in zip(controls,controls[1:])]),
            'state_source_to_record_receipt_ms':analyze.stats([(int(r['receipt_ns'])-int(r['source_ns']))/1e6 for r in od]),
            'next_state_pairs':len(models),'sdk_position_error_m':analyze.stats([r['sdk_position_error_m'] for r in models]),'sdk_yaw_error_rad':analyze.stats([r['sdk_yaw_error_rad'] for r in models]),
            'feedback_hold_position_error_m':analyze.stats([r['feedback_hold_position_error_m'] for r in models]),
            'observed_target_2_ramp_error_m':[analyze.stats([r['observed_target_2_ramp_error_m'][i] for r in models]) for i in (0,1)],
            'sdk_velocity_error':analyze.stats([r['sdk_vs_actual_next_velocity_error'] for r in models]),
            'worst_sdk_pairs':sorted(models,key=lambda r:r['sdk_position_error_m'],reverse=True)[:3],
            'first_pairs':models[:3], 'actual_application_time':'not recorded; ROS receipt projection is retrospective, not causal/deployable certification'}
    return result
def history():
    index=json.loads((OUT/'history_index.json').read_text());rows=[]
    for meta in index:
        case=pathlib.Path(meta['input']).stem;r=json.loads((ROOT/'build/joint_cache/results'/f'{case}.json').read_text());raw=r['results'][:300];selected=r['results'][300]
        costs=meta['native_costs_after_gamma'];weights=[math.exp(-(x-min(costs))/meta['native_temperature']) for x in costs];z=sum(weights)
        native_dyn=meta['native_dynamic_min_clearance'] or []
        errors=[abs(d['dynamic_discrete_min_m']-native_dyn[i]) for i,d in enumerate(raw) if i<len(native_dyn) and d['dynamic_discrete_min_m'] is not None]
        rows.append({'case':case,'relative_s':meta['relative_s'],'native_feedback_matches_recorded':meta['feedback_consistent'],'total_ms':r['total_ms'],
            'flags':{k:dict(collections.Counter(d[k] for d in raw)) for k in ('static','dynamic','nominal_rate','joint_geometry','joint_complete')},
            'native_same_static_perimeter_clear_but_filled_swept_unsafe':sum(d['static']=='Unsafe' and d['native_same_static_grid_discrete_hits']==0 for d in raw),
            'native_dynamic_discrete_clear_but_dense_unsafe':sum(i<len(native_dyn) and native_dyn[i]>.02 and d['dynamic']=='Unsafe' for i,d in enumerate(raw)),
            'same_static_native_hit_but_new_static_safe':sum(d['static']=='Safe' and d['native_same_static_grid_discrete_hits']>0 for d in raw),
            'master_hit_but_separate_static_dynamic_safe':sum(d['joint_geometry']=='Safe' and meta['native_master_footprint_hits'][i] for i,d in enumerate(raw)),
            'geometry_unsafe_softmax_mass':sum(weights[i] for i,d in enumerate(raw) if d['joint_geometry']=='Unsafe')/z,
            'dynamic_discrete_alignment_max_difference':max(errors,default=None),'selected_sdk':selected})
    (OUT/'history_summary.json').write_text(json.dumps(rows,indent=2)+'\n')
def main():
    history();rows=[]
    for cohort in ('repair_odom','heldout_odom'):
        for p in sorted((OLD/cohort/'runs').glob('*')):
            if (p/'events.json').exists():r=execution(p);r['cohort']=cohort;rows.append(r)
    result={'scope':'old raw read-only; fixed retrospective 0/20ms receipt-delay sensitivity and 1ms per-component 2.0 SDF rate; future observed commands solely evaluation, never group inputs', 'runs':rows}
    (OUT/'execution_history.json').write_text(json.dumps(result,indent=2)+'\n');print('audited',len(rows),'historical runs')
if __name__=='__main__':main()
