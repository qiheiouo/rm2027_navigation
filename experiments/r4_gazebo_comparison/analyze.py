"""Offline finite-run metrics. Oracle geometry never feeds either controller."""
import csv,json,math,pathlib,statistics,sys
frozen=pathlib.Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/experiments/temporal_mpc')
sys.path.insert(0,str(frozen))
from temporal_mpc.oracle import polygon_distance

def rows(path):
    return list(csv.DictReader(path.open())) if path.exists() else []
def stats(values):
    if not values: return None
    values=sorted(values)
    def q(p):
        x=(len(values)-1)*p;i=int(x)
        return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(x-i)
    return dict(n=len(values),min=values[0],p50=q(.5),p95=q(.95),max=values[-1])
def trial(path):
    e=json.loads((path/'events.json').read_text());start=e['goal_ns'];end=e['end_ns']
    manifest=json.loads((path/'manifest.json').read_text())
    phase=manifest.get('phase','finite' if manifest['repeat']>=100 else 'pilot')
    result=dict(run=path.name,scene=e['scene'],mode=e['mode'],phase=phase,goal_ns=start,
                finished=e['finished'],status=e['result_status'],reason=e['controller_failure'],
                contact_messages=len(e['contacts']),startup=start is None)
    if start is None: return result
    result['success']=e['result_status']==4 and not e['contacts'] and not e['controller_failure']
    result['elapsed_s']=(end-start)/1e9
    result['arrival_s']=result['elapsed_s'] if result['success'] else None
    od=[r for r in rows(path/'odometry.csv') if start<=int(r['source_ns'])<=end]
    ct=[r for r in rows(path/'control.csv') if start<=int(r['epoch_ns'])<=end]
    result['native_ms']=stats([float(r['native_ms']) for r in ct])
    result['solver_ms']=stats([float(r['solver_ms']) for r in ct if r['mode']=='R4'])
    result['r4_call_ms']=stats([float(r['total_ms']) for r in ct if r['mode']=='R4'])
    result['control_samples']=len(ct);result['valid_samples']=sum(r['valid']=='1' for r in ct)
    result['combined_call_ms']=stats([float(r['native_ms'])+float(r['total_ms']) for r in ct])
    stamps=[int(r['epoch_ns']) for r in ct]
    result['control_gap_ms']=stats([(b-a)/1e6 for a,b in zip(stamps,stamps[1:])])
    # B0 logs entry epoch while R4 logs post-native acquisition. Compare actual
    # packets at the common observable output instead of those distinct seams.
    packets=[int(r['receipt_ns']) for r in rows(path/'commands.csv') if r['topic']=='/simulation/chassis/cmd_vel' and start<=int(r['receipt_ns'])<=end]
    result['actual_output_gap_ms']=stats([(b-a)/1e6 for a,b in zip(packets,packets[1:])])
    wait=longest=current=0.;resume=None;forward_count=0;first_forward=None;last_sign=None;reversals=0
    for i,r in enumerate(od):
        ns=int(r['source_ns']);speed=math.hypot(float(r['vx']),float(r['vy']))
        dt=0 if i+1==len(od) else (int(od[i+1]['source_ns'])-ns)/1e9
        if 0<=dt<=.1:
            if speed<.02: wait+=dt;current+=dt;longest=max(longest,current)
            else: current=0
        else: current=0
        yaw=float(r['yaw']);vx=math.cos(yaw)*float(r['vx'])-math.sin(yaw)*float(r['vy'])
        sign=1 if vx>.02 else -1 if vx<-.02 else None
        if sign is not None:
            if last_sign is not None and sign!=last_sign: reversals+=1
            last_sign=sign
        if e['scene']=='S2' and ns>=start+9_000_000_000 and resume is None:
            if vx>.05:
                if forward_count==0: first_forward=ns
                forward_count+=1
                if forward_count==3: resume=(first_forward-start-9_000_000_000)/1e9
            else: forward_count=0;first_forward=None
    result.update(wait_s=wait if od else None,longest_stall_s=longest if od else None,
                  clear_resume_s=resume,forward_reversals=reversals if od else None)
    truth={}
    for line in (path/'truth.jsonl').open():
        r=json.loads(line);ns=r['source_ns']
        if start<=ns<=end: truth.setdefault(ns,{})[r['model']]=r
    robot=[(ns,r['rm_sentry_2027']) for ns,r in sorted(truth.items()) if 'rm_sentry_2027' in r]
    common=[(ns,r) for ns,r in sorted(truth.items()) if 'rm_sentry_2027' in r and 'moving_obstacle' in r]
    distances=[polygon_distance(r['rm_sentry_2027']['polygon'],r['moving_obstacle']['polygon']) for ns,r in common]
    result['min_dynamic_clearance_m']=min(distances) if distances else None
    result['envelope_overlap_samples']=sum(d<=1e-10 for d in distances)
    result['oracle_robot_samples']=len(robot);result['oracle_common_samples']=len(common)
    result['oracle_common_gap_ms']=stats([(b[0]-a[0])/1e6 for a,b in zip(common,common[1:])])
    result['start_position']=robot[0][1]['position'] if robot else None
    result['end_position']=robot[-1][1]['position'] if robot else None
    x=[v['position'][0] for ns,v in robot]
    result['backward_distance_m']=sum(max(0.,a-b) for a,b in zip(x,x[1:])) if x else None
    maximum=-math.inf;backtrack=0.
    for value in x: maximum=max(maximum,value);backtrack=max(backtrack,maximum-value)
    result['maximum_backtrack_m']=backtrack if x else None
    result['contacts_publishers']=e.get('contacts_publishers')
    result['contact_seen']=e.get('contact_seen')
    result['state_age_ms']=stats([(int(r['epoch_ns'])-int(r['state_stamp_ns']))/1e6 for r in ct if int(r['state_stamp_ns'])>0])
    result['history_age_ms']=stats([(int(r['epoch_ns'])-int(r['history_stamp_ns']))/1e6 for r in ct if int(r['history_stamp_ns'])>0])
    # Timed actual obstacle samples permit a paired scene audit, including PID lag.
    actor=[((ns-start)/1e9,r['moving_obstacle']['position']) for ns,r in sorted(truth.items()) if 'moving_obstacle' in r]
    result['actor_timed_samples']=actor
    # Shared straight-route circle support, independent of either actual ego
    # timing. The actor is clear once its full projection leaves this band.
    radius=math.hypot(.32,.27)+.02
    clear=[ns for ns,r in sorted(truth.items()) if ns>=start+7_000_000_000 and 'moving_obstacle' in r and min(y for x,y in r['moving_obstacle']['polygon'])>radius]
    clear_ns=clear[0] if clear and e['scene']=='S2' else None
    result['actual_clear_s']=(clear_ns-start)/1e9 if clear_ns is not None else None
    resume_actual=None;first=None;consecutive=0
    if clear_ns is not None:
        for r in od:
            ns=int(r['source_ns'])
            if ns<clear_ns: continue
            yaw=float(r['yaw']);forward=math.cos(yaw)*float(r['vx'])-math.sin(yaw)*float(r['vy'])
            if forward>.05:
                if consecutive==0: first=ns
                consecutive+=1
                if consecutive==3: resume_actual=max(0.,(first-clear_ns)/1e9);break
            else: first=None;consecutive=0
    result['actual_clear_resume_s']=resume_actual
    result['already_forward_at_clear']=resume_actual==0. if resume_actual is not None else None
    result['nominal_dynamic_positive_cycles']=sum(float(r['nominal_cost'])>0 for r in ct) if e['mode']=='R4' else None
    result['costmap_actor_occupied_samples']=sum(int(r['actor_window_lethal'])>0 for r in rows(path/'costmap.csv') if r['actor_window_lethal'] and start<=int(r['source_ns'])<=end) if (path/'costmap.csv').exists() else None
    (path/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
def aggregate(root):
    runs=[trial(p) for p in sorted((root/'runs').iterdir()) if (p/'events.json').exists()]
    summary=[]
    for scene in ('S0','S1','S2'):
        for mode in ('B0','R4'):
            group=[r for r in runs if r['scene']==scene and r['mode']==mode and r['phase']=='finite' and not r['startup']]
            if not group: continue
            controls=[r for run in group for r in rows(root/'runs'/run['run']/'control.csv') if run['goal_ns']<=int(r['epoch_ns'])<=run['goal_ns']+int(run['elapsed_s']*1e9)]
            summary.append(dict(scene=scene,mode=mode,n=len(group),success=sum(r['success'] for r in group),
                contacts=sum(r['contact_messages']>0 for r in group),
                failures=[dict(run=r['run'],finished=r['finished'],reason=r['reason']) for r in group if not r['success']],
                min_clearance_m=stats([r['min_dynamic_clearance_m'] for r in group if r['min_dynamic_clearance_m'] is not None]),
                arrival_s=stats([r['arrival_s'] for r in group if r['arrival_s'] is not None]),
                wait_s=stats([r['wait_s'] for r in group if r['wait_s'] is not None]),
                longest_stall_s=stats([r['longest_stall_s'] for r in group if r['longest_stall_s'] is not None]),
                clear_resume_s=stats([r['clear_resume_s'] for r in group if r['clear_resume_s'] is not None]),
                actual_clear_resume_s=stats([r['actual_clear_resume_s'] for r in group if r['actual_clear_resume_s'] is not None]),
                forward_reversals=stats([r['forward_reversals'] for r in group if r['forward_reversals'] is not None]),
                maximum_backtrack_m=stats([r['maximum_backtrack_m'] for r in group if r['maximum_backtrack_m'] is not None]),
                solver_p95_ms=stats([r['solver_ms']['p95'] for r in group if r['solver_ms']]),
                native_p95_ms=stats([r['native_ms']['p95'] for r in group if r['native_ms']]),
                native_call_ms=stats([float(r['native_ms']) for r in controls]),
                r4_solver_ms=stats([float(r['solver_ms']) for r in controls if mode=='R4']),
                r4_consumption_ms=stats([float(r['total_ms']) for r in controls if mode=='R4']),
                combined_call_ms=stats([float(r['native_ms'])+float(r['total_ms']) for r in controls])))
    compact=[{k:v for k,v in r.items() if k!='actor_timed_samples'} for r in runs]
    (root/'summary.json').write_text(json.dumps(dict(groups=summary,runs=compact),indent=2)+'\n')
    fields=['run','scene','mode','phase','startup','success','finished','reason','contact_messages','min_dynamic_clearance_m','arrival_s','elapsed_s','wait_s','longest_stall_s','clear_resume_s','actual_clear_s','actual_clear_resume_s','forward_reversals','control_samples','valid_samples','oracle_common_samples']
    with (root/'trials.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(compact)
    print(json.dumps(summary,indent=2))
if __name__=='__main__': aggregate(pathlib.Path(sys.argv[1]))
