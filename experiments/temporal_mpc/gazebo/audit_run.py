#!/usr/bin/env python3
"""Read-only audit of complete actual ROS inputs and independent Gazebo geometry.

Public observations never provide oracle geometry/poses. Relative model/link
poses are composed at their integer source epochs; missing samples fail gates.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from temporal_mpc.contracts import PublicAdapter, ContractError, V2
from temporal_mpc.oracle import polygon_distance


def namespace(value):
    if isinstance(value,dict): return SimpleNamespace(**{k:namespace(v) for k,v in value.items()})
    if isinstance(value,list): return [namespace(v) for v in value]
    return value


def epoch(stamp):
    if (type(stamp['sec']) is not int or type(stamp['nanosec']) is not int
            or stamp['sec']<0 or not 0<=stamp['nanosec']<10**9): raise ValueError('integer stamp')
    return stamp['sec']*10**9+stamp['nanosec']


def stats(values):
    if not values: return None
    return dict(count=len(values),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
                min=float(min(values)),max=float(max(values)))


def rotation(q):
    v=np.array([q[k] for k in ('x','y','z','w')],float)
    if not np.isfinite(v).all() or abs(v@v-1)>1e-5: raise ValueError('oracle quaternion')
    x,y,z,w=v
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def transform(t):
    return np.array([t['translation'][k] for k in ('x','y','z')]),rotation(t['rotation'])


def hull(points):
    points=sorted(set(tuple(map(float,p)) for p in points))
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[]; upper=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0: lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0: upper.pop()
        upper.append(p)
    if len(lower[:-1]+upper[:-1])<3: raise ValueError('degenerate oracle projection')
    return lower[:-1]+upper[:-1]


def physical_projection(message,model,link):
    by_child={t['child_frame_id']:t for t in message['transforms']}
    world=by_child[model]; relative=by_child[model+'/'+link]
    ns=epoch(world['header']['stamp'])
    if (world['header']['frame_id']!='phase1_omni' or relative['header']['frame_id']!=model
            or epoch(relative['header']['stamp'])!=ns): raise ValueError('oracle hierarchy/epoch')
    wp,wr=transform(world['transform']); lp,lr=transform(relative['transform'])
    p,r=wp+wr@lp,wr@lr
    if model=='rm_sentry_2027':
        # Complete mechanical 3D envelope of main SDF: body + four r=.075 spheres.
        # Neither visual size nor tracker observed extent is used.
        extents=(.325,.300); heights=(0.,.26)
        for name,x,y in (('front_left_wheel',.25,.225),('front_right_wheel',.25,-.225),
                         ('rear_left_wheel',-.25,.225),('rear_right_wheel',-.25,-.225)):
            wheel=by_child[model+'/'+name]
            if epoch(wheel['header']['stamp'])!=ns: raise ValueError('wheel epoch')
            center,_=transform(wheel['transform'])
            if np.max(np.abs(center-np.array([x,y,.075])))>.001: raise ValueError('wheel offset outside envelope')
    else: extents=(.225,.275); heights=(-.4,.4)
    corners=[p+r@np.array([x,y,z]) for x in (-extents[0],extents[0])
             for y in (-extents[1],extents[1]) for z in heights]
    return ns,hull([point[:2] for point in corners]),p,r


def audit(directory):
    root=Path(directory)
    events=[]
    with (root/'events.jsonl').open() as stream:
        for line in stream: events.append(json.loads(line))
    summary=json.loads((root/'run_summary.json').read_text())
    scene=json.loads((root/'scene/scene.json').read_text())
    adapter=PublicAdapter(geometry_mode='nominal_diameter')
    rejected=Counter(); sources=[]; observations=Counter(); complete=0; confirmed=0
    receipt_age=[]; processing=[]; display_error=[]; accepted=0
    scans={epoch(e['data']['header']['stamp']):e for e in events if e['topic']=='/scan'}
    source_tfs={e['data']['source_ns']:e['data'] for e in events if e['topic']=='source_tf'}
    tf_ok=0; schema_failures=0; source_scan_ok=0; process_invalid=0
    for e in events:
        if e['topic']!='/dynamic_obstacle_predictions': continue
        d=e['data']; ns=epoch(d['header']['stamp']); sources.append(ns)
        complete+=d['complete'] is True
        receipt_age.append((e['receipt_sim_ns']-ns)/1e9)
        delay=(epoch(d['processing_stamp'])-ns)/1e9; processing.append(delay)
        if delay<0 or epoch(d['processing_stamp'])>e['receipt_sim_ns']: process_invalid+=1
        if d['schema']!=V2 or d['authority']!='shadow_only' or d['header']['frame_id']!='map': schema_failures+=1
        tf=source_tfs.get(ns)
        if tf and tf['available'] and epoch(tf['transform']['header']['stamp'])==ns: tf_ok+=1
        if ns in scans and scans[ns]['data']['header']['frame_id']=='sim_lidar_link': source_scan_ok+=1
        try: adapter.consume(namespace(d),e['receipt_sim_ns']); accepted+=1
        except ContractError as error: rejected[str(error)]+=1
        for t in d['tracks']:
            observations[str(t['state'])]+=1
            confirmed+=t['state']==2
            for i,p in enumerate(t['prediction']):
                expected=[t['position'][k]+(i+1)*d['prediction_dt']*t['velocity'][k] for k in ('x','y','z')]
                display_error.append(max(abs(p[k]-v) for k,v in zip(('x','y','z'),expected)))
    odom=[e for e in events if e['topic']=='/odometry/lio']
    model_invalid=Counter(); wz=[]; velocity=[]; odom_ages=[]
    for e in odom:
        d=e['data']; q=d['pose']['pose']['orientation']; v=d['twist']['twist']; w=abs(v['angular']['z'])
        wz.append(w); velocity.append(math.hypot(v['linear']['x'],v['linear']['y']))
        odom_ages.append((e['receipt_sim_ns']-epoch(d['header']['stamp']))/1e9)
        if w>1e-8: model_invalid['fixed_yaw_qp_wz']+=1
        if abs(q['x'])>1e-6 or abs(q['y'])>1e-6: model_invalid['nonplanar_worker_pose']+=1
        if v['linear']['x']<-.5 or v['linear']['x']>.8 or abs(v['linear']['y'])>.5: model_invalid['body_speed_bounds']+=1
    health=Counter(); feasible=0; solves=[]; solve_reasons=Counter(); compute=[]; executed_ready=0
    for e in events:
        if e['topic']=='/temporal_mpc/health':
            d=json.loads(e['data']['data']);health[d['reason']]+=1
            if d['executed']: compute.append(d['elapsed_s']);executed_ready+=d['ready']
        if e['topic']=='/temporal_mpc/solver_diagnostic':
            d=json.loads(e['data']['data']);solves.append(d['elapsed_s']);feasible+=d['feasible'];solve_reasons[d['reason']]+=1
    robot={}; actor={}; oracle_errors=Counter()
    for e in events:
        if e['topic'] not in ('/simulation/oracle/rm_sentry_2027','/simulation/oracle/moving_obstacle'): continue
        model=e['topic'].rsplit('/',1)[-1];link='base_link' if model=='rm_sentry_2027' else 'obstacle_link'
        try:
            ns,polygon,p,r=physical_projection(e['data'],model,link)
            (robot if model=='rm_sentry_2027' else actor)[ns]=(polygon,p,r)
        except (KeyError,ValueError) as error: oracle_errors[str(error)]+=1
    common=sorted(set(robot)&set(actor))
    distances=[polygon_distance(robot[ns][0],actor[ns][0]) for ns in common]
    gaps=np.diff(common)/1e9 if len(common)>1 else []
    # Declared diagnostic point-speed bound 4m/s robot + 1.5m/s actor. This is
    # not a plant certificate; samples cannot certify unseen physical motion.
    reserve=max(gaps,default=0.)*.5*5.5
    lower=min(distances,default=math.inf)-reserve
    sampled_intersection=any(d<=1e-10 for d in distances)
    contact_messages=[e for e in events if e['topic']=='/simulation/oracle/contacts']
    contacts=[c for e in contact_messages for c in e['data'].get('contacts',[])
              if 'rm_sentry_2027' in json.dumps(c)]
    sensor_gate=(len(sources)>=100 and schema_failures==0 and confirmed>0 and complete==len(sources)
                 and accepted==len(sources) and tf_ok==len(sources) and source_scan_ok==len(sources)
                 and process_invalid==0 and max(display_error,default=math.inf)<1e-8)
    state_gate=bool(odom) and not model_invalid
    action_success=summary['result'] is not None and summary['result'].get('status')==4
    # Diagnostic clearance reserve does not turn sampling into a physical proof.
    physical_gate=bool(common and not oracle_errors and len(common)>=100 and max(gaps,default=math.inf)<=.025)
    replay_file=root/"ros_capture_audit.json"
    replay=json.loads(replay_file.read_text()) if replay_file.exists() else None
    replay_gate=bool(replay and not replay["serialized_json_mismatches"] and
                     replay["source_tf_replay_available"]==len(sources))
    sensor_gate=bool(sensor_gate or (len(sources)>=100 and schema_failures==0 and confirmed>0
        and complete==len(sources) and accepted==len(sources) and source_scan_ok==len(sources)
        and process_invalid==0 and max(display_error,default=math.inf)<1e-8 and replay_gate))
    result=dict(source_tf_replay_gate=replay_gate,event_file_sha256=hashlib.sha256((root/'events.jsonl').read_bytes()).hexdigest(),
                scenario=scene['scenario'],mode=summary['mode'],source_counts=len(sources),public_accepted=accepted,
                public_rejections=dict(rejected),source_epoch_monotonic=all(b>a for a,b in zip(sources,sources[1:])),
                schema_failures=schema_failures,complete_count=complete,track_states=dict(observations),
                confirmed_observations=confirmed,source_scan_pairs=source_scan_ok,source_tf_pairs=tf_ok,
                source_receipt_sim_s=stats(receipt_age),source_processing_sim_s=stats(processing),
                processing_order_failures=process_invalid,display_cv_error_max_m=max(display_error,default=None),
                odom_count=len(odom),odom_receipt_sim_s=stats(odom_ages),measured_wz_abs=stats(wz),
                measured_translation_speed=stats(velocity),model_domain_rejections=dict(model_invalid),
                solver_s=stats(solves),solver_feasible_count=feasible,solver_reasons=dict(solve_reasons),
                native_health_reasons=dict(health),native_executed_s=stats(compute),native_executed_ready_count=executed_ready,
                oracle_same_epoch_pairs=len(common),oracle_errors=dict(oracle_errors),oracle_gap_s=stats(list(gaps)),
                sampled_full_envelope_clearance_m=min(distances,default=None),
                diagnostic_sweep_lower_m=lower if math.isfinite(lower) else None,
                diagnostic_point_speed_bound=5.5,physical_speed_bound_certified=False,
                sampled_full_envelope_intersection=sampled_intersection,actual_robot_contacts=len(contacts),
                actual_contact_message_count=len(contact_messages),action_success=action_success,
                sensor_contract_gate=sensor_gate,measured_model_gate=state_gate,mpc_entry_gate=sensor_gate and state_gate,
                oracle_sample_gate=physical_gate,physical_acceptance=False,false_block=None,
                blocked_pair_reason=None if sensor_gate and state_gate else 'real input/model entry gate failed; MPC comparison not authorized by registration')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    result=audit(args.directory)
    (Path(args.directory)/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))
