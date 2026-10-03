#!/usr/bin/env python3
"""Manifest-backed source-ray reveal labels; no runtime or stopping certificate."""
import argparse
import bisect
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path
import yaml
from analyze_trial import compose,distance,load_truth_rows,rotation,segment
from audit_mechanical_footprint import components
from audit_scan_geometry import fixture,first_box,interpolate,stats
from dynamic_consumption_io import read_consumption
from trial_io import rows

POLICY={'material_bin_m':.05,'residual_sigma_multiplier':3.,'minimum_face_returns':3,
        'confirmation_scans':2,'maximum_confirmation_gap_s':.15,'maximum_pose_bracket_s':.04,
        'margin_m':.05,'tau_s':[0.,.05,.1,.2,.4],'deceleration_mps2':[.5,1.,2.]}
FACES=('x_minus','x_plus','y_minus','y_plus')


def validate(root):
    root=root.resolve()
    manifest=json.loads((root/'manifest.json').read_text())
    actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name!='manifest.json'}
    if actual!=set(manifest['files']):raise ValueError('archive file set differs')
    for name,sha in manifest['files'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root):raise ValueError('source escaped archive')
        h=hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
        if h.hexdigest()!=sha:raise ValueError('archive identity mismatch '+name)
    return hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()


def face_bin(point,dimensions,tolerance):
    """Classify an ideal box intersection; corners stay ambiguous."""
    x,y=point;hx,hy=(v/2 for v in dimensions)
    onx=abs(abs(x)-hx)<=1e-7;ony=abs(abs(y)-hy)<=1e-7
    if onx and hy-abs(y)>tolerance:face='x_minus' if x<0 else 'x_plus';along=y;length=2*hy
    elif ony and hx-abs(x)>tolerance:face='y_minus' if y<0 else 'y_plus';along=x;length=2*hx
    else:return None
    index=min(math.ceil(length/POLICY['material_bin_m'])-1,max(0,math.floor((along+length/2)/POLICY['material_bin_m'])))
    return face,index


def material_segment(face,index,dimensions):
    hx,hy=(v/2 for v in dimensions);length=2*(hy if face.startswith('x') else hx)
    lo=-length/2+index*POLICY['material_bin_m'];hi=min(length/2,lo+POLICY['material_bin_m'])
    if face.startswith('x'):
        x=-hx if face=='x_minus' else hx;return [(x,lo),(x,hi)]
    y=-hy if face=='y_minus' else hy;return [(lo,y),(hi,y)]


def closing_sensitivity(gap,speed,actor_speed):
    """Explicit hypothetical translation-only model, actor speed is not a bound."""
    if any(not math.isfinite(v) or v<0 for v in (gap,speed,actor_speed)):raise ValueError('finite nonnegative sensitivity inputs required')
    return [{'tau_s':tau,'assumed_deceleration_mps2':a,
             'self_reaction_and_stop_m':speed*tau+speed*speed/(2*a),
             'actor_translation_until_self_stop_m':actor_speed*(tau+speed/a),
             'static_target_reveal_margin_m':gap-POLICY['margin_m']-speed*tau-speed*speed/(2*a),
             'instant_actor_speed_sensitivity_margin_m':gap-POLICY['margin_m']-speed*tau-speed*speed/(2*a)-actor_speed*(tau+speed/a)}
            for tau in POLICY['tau_s'] for a in POLICY['deceleration_mps2']]


def reliable_keys(counts,previous,stamp,previous_stamp):
    face_counts=Counter()
    for (face,index),count in counts.items():face_counts[face]+=count
    previous_faces=Counter()
    for (face,index),count in previous.items():previous_faces[face]+=count
    if previous_stamp is None or not 0<stamp-previous_stamp<=POLICY['maximum_confirmation_gap_s']:return set()
    return {key for key in counts if previous.get(key,0)>0 and face_counts[key[0]]>=POLICY['minimum_face_returns']
            and previous_faces[key[0]]>=POLICY['minimum_face_returns']}


def linear_values(times,values,stamp,max_gap):
    """Interpolate twist components as values, without angle wrapping."""
    i=bisect.bisect_left(times,stamp)
    if i<len(times) and times[i]==stamp:return tuple(values[i])
    if i==0 or i==len(times) or times[i]-times[i-1]>max_gap:return None
    u=(stamp-times[i-1])/(times[i]-times[i-1])
    return tuple(a+u*(b-a) for a,b in zip(values[i-1],values[i]))


def audit(root):
    root=root.resolve();manifest_sha=validate(root)
    execution=json.loads((root/'execution.json').read_text());trial_policy=json.loads((root/'policy.json').read_text())
    if execution['execution']!='PASS' or trial_policy['body_clearance']!=POLICY['margin_m']:raise ValueError('original executed fixed-margin trial required')
    source=list(rows(root,'observations.jsonl'));scene=fixture(root/'scene_inputs');sigma=scene['noise_stddev']
    if not math.isfinite(sigma) or sigma<=0:raise ValueError('recorded positive sensor sigma required')
    physical=load_truth_rows(root,execution);truth_times=[r['t'] for r in physical]
    robots=[r['robot'] for r in physical];actors=[r['obstacle'] for r in physical]
    odom=[]
    for row in sorted((r for r in source if r['kind']=='canonical_odom'),key=lambda r:r['stamp']):
        if odom and row['stamp']<=odom[-1]['stamp']:continue
        x,y,qx,qy,qz,qw=row['pose']
        if (row['frame']!='odom' or row['child_frame']!='base_link' or abs(qx)>1e-3 or abs(qy)>1e-3
                or abs(qz*qz+qw*qw-1)>1e-3
                or not all(math.isfinite(v) for v in row['pose']+row['velocity'])):raise ValueError('unsupported canonical odometry')
        odom.append(row)
    odom_times=[r['stamp'] for r in odom]
    odom_poses=[(r['pose'][0],r['pose'][1],math.atan2(2*r['pose'][5]*r['pose'][4],1-2*r['pose'][4]**2)) for r in odom]
    velocities=[r['velocity'] for r in odom]
    guards=sorted((r for r in source if r['kind']=='guard'),key=lambda r:r['receive_sim']);guard_times=[r['receive_sim'] for r in guards]
    commands=sorted((r for r in source if r['kind']=='final_cmd'),key=lambda r:r['receive_sim']);command_times=[r['receive_sim'] for r in commands]
    publications=defaultdict(list)
    for row in source:
        if row['kind']=='obstacles':publications[row['source_stamp_ns']].append(row)
    consumed=defaultdict(list)
    for path in sorted((root/'dynamic_scores').glob('score_*.json'),key=lambda p:int(p.stem.split('_')[-1])):
        meta,_=read_consumption(path);used=meta['input_used'];epoch=used['stamp_sec']*1_000_000_000+used['stamp_nanosec']
        consumed[epoch].append({'ordinal':meta['ordinal'],'score_stamp':meta['score_stamp'],'source_age_used':meta['source_age_used'],
                                'tracks':len(used['tracks']),'input_schema':used['schema']})
    shapes=components(root/'scene_inputs');body=next(s['points'] for s in shapes if s['name']=='base_link/base_collision')
    config=yaml.safe_load((root/'profile.yaml').read_text());cm=config['local_costmap']['local_costmap']['ros__parameters']
    footprint=yaml.safe_load(cm['footprint']);pad=cm['footprint_padding']
    padded=[tuple(v+(pad if v>0 else -pad if v<0 else 0) for v in p) for p in footprint]
    dimensions=scene['actor_dimensions'];tolerance=POLICY['residual_sigma_multiplier']*sigma
    all_keys={(face,i) for face in FACES for i in range(math.ceil((dimensions[1] if face.startswith('x') else dimensions[0])/POLICY['material_bin_m']))}
    first_raw={};first_ideal={};first_reliable={};previous={};previous_stamp=None;last_reliable=set()
    accepted=[];events=[];skips=Counter();failures=Counter();first_stamp=None
    def latest(records,times,stamp):
        i=bisect.bisect_right(times,stamp)-1
        return None if i<0 else {'observer_receipt_age_s':stamp-times[i],'record':records[i]}
    def event(key,kind,first_scan,confirm,robot,actor):
        face,index=key;surface=rotation(actor,material_segment(face,index,dimensions))
        body_gap=distance(rotation(robot,body),surface);padded_gap=distance(rotation(robot,padded),surface)
        gaps=[body_gap]
        for shape in shapes:
            if shape['kind']=='circle':
                center=rotation(robot,[shape['center']])[0];gaps.append(max(0.,segment(*surface,center)-shape['radius']))
        measured=linear_values(odom_times,velocities,confirm['stamp'],POLICY['maximum_pose_bracket_s'])
        k=bisect.bisect_right(truth_times,confirm['stamp'])
        actor_speed=None;actor_yaw_rate=None
        if 0<k<len(truth_times):
            dt=truth_times[k]-truth_times[k-1];actor_speed=math.dist(actors[k][:2],actors[k-1][:2])/dt
            actor_yaw_rate=math.remainder(actors[k][2]-actors[k-1][2],2*math.pi)/dt
        speed=math.hypot(*measured[:2]) if measured is not None else None
        epoch=round(confirm['stamp']*1e9);publication=publications.get(epoch,[]);contexts=consumed.get(epoch,[])
        result={'face':face,'material_bin':index,'event_kind':kind,'first_ray_source_stamp':first_scan,
                'confirmation_source_stamp':confirm['stamp'],'confirmation_observer_receive_sim':confirm['receive_sim'],
                'observer_scan_receipt_delay_s':confirm['receive_sim']-confirm['stamp'],'robot_physical_pose':robot,
                'actor_physical_pose':actor,'material_segment_world':surface,'base_gap_m':body_gap,
                'full_mechanical_gap_m':min(gaps),'padded_gap_m':padded_gap,'measured_twist_interpolated':measured,
                'robot_translation_speed_mps':speed,'offline_instant_actor_translation_speed_mps':actor_speed,
                'offline_instant_actor_yaw_rate':actor_yaw_rate,'latest_guard_by_observer_receipt':latest(guards,guard_times,confirm['stamp']),
                'latest_final_cmd_by_observer_receipt':latest(commands,command_times,confirm['stamp']),
                'same_scan_epoch_public_receipts':[{'receive_sim':r['receive_sim'],'stamp':r['stamp'],'tracks':len(r['tracks'])} for r in publication],
                'same_scan_epoch_actual_consumptions':contexts,'actual_surface_membership_in_tracker_or_critic':'UNKNOWN',
                'actual_total_response_latency_s':None,'actual_brake_deceleration_mps2':None,
                'actual_braking_or_replanning_margin':'NOT ESTABLISHED',
                'hypothetical_translation_stop_sensitivity':closing_sensitivity(min(gaps),speed,actor_speed) if speed is not None and actor_speed is not None else []}
        if kind=='first_reliable':result['left_censored_at_first_accepted_scan']=first_raw.get(key)==first_stamp
        return result
    for scan in sorted((r for r in source if r['kind']=='scan' and execution['start_sim']<=r['stamp']<=execution['last_sim']),key=lambda r:r['stamp']):
        stamp=scan['stamp']
        if scan['frame']!='sim_lidar_link' or scan['time_increment']!=0:skips['unsupported_frame_or_scan_motion']+=1;continue
        canonical=interpolate(odom_times,odom_poses,stamp,.04);robot=interpolate(truth_times,robots,stamp,.04);actor=interpolate(truth_times,actors,stamp,.04)
        if canonical is None or robot is None or actor is None:skips['missing_source_pose_bracket']+=1;continue
        if math.dist(canonical[:2],robot[:2])>1e-4 or abs(math.remainder(canonical[2]-robot[2],2*math.pi))>1e-4:skips['odom_world_alignment_mismatch']+=1;continue
        if first_stamp is None:first_stamp=stamp
        sensor=compose(canonical,scene['sensor_xy_yaw']);counts=Counter();ideal=Counter();indices=defaultdict(list)
        for i,measured in enumerate(scan['ranges']):
            angle=sensor[2]+scan['angle_min']+i*scan['angle_increment']
            hit=first_box(sensor,angle,actor,scene,scan['range_min'],scan['range_max'])
            if hit is None or hit[1]!='actor':continue
            expected=hit[0];world=(sensor[0]+expected*math.cos(angle),sensor[1]+expected*math.sin(angle))
            dx,dy=world[0]-actor[0],world[1]-actor[1];c,s=math.cos(actor[2]),math.sin(actor[2]);key=face_bin((c*dx+s*dy,-s*dx+c*dy),dimensions,tolerance)
            if key is None:failures['ambiguous_corner_rays']+=1;continue
            ideal[key]+=1;first_ideal.setdefault(key,stamp)
            if not isinstance(measured,(int,float)) or not math.isfinite(measured) or not scan['range_min']<=measured<=scan['range_max']:failures['expected_actor_nonfinite_or_invalid_return']+=1;continue
            if abs(measured-expected)>tolerance:failures['expected_actor_residual_outside_label_gate']+=1;continue
            counts[key]+=1;indices[key].append(i);first_raw.setdefault(key,stamp)
        reliable=reliable_keys(counts,previous,stamp,previous_stamp)
        for key in sorted(reliable):
            if key not in first_reliable:
                first_reliable[key]=stamp;events.append(event(key,'first_reliable',previous_stamp,scan,robot,actor))
            elif key not in last_reliable:events.append(event(key,'reappearance',previous_stamp,scan,robot,actor))
        accepted.append({'source_stamp':stamp,'receive_sim':scan['receive_sim'],'ideal_actor_bins':len(ideal),'observed_actor_bins':len(counts),
                         'reliable_actor_bins':len(reliable),'actor_ray_counts':{f'{face}:{index}':value for (face,index),value in sorted(counts.items())},
                         'reliable_material_keys':[f'{face}:{index}' for face,index in sorted(reliable)],
                         'source_ray_indices':{f'{face}:{index}':value for (face,index),value in sorted(indices.items())}})
        previous=counts;previous_stamp=stamp;last_reliable=reliable
    material=[]
    for key in sorted(all_keys):
        material.append({'face':key[0],'bin':key[1],'local_segment':material_segment(*key,dimensions),
                         'first_geometric_recorded_ray_source_stamp':first_ideal.get(key),'first_actual_valid_label_source_stamp':first_raw.get(key),
                         'first_reliable_confirmation_source_stamp':first_reliable.get(key),
                         'state':'reliable_proxy_observed' if key in first_reliable else 'NOT REVEALED BY REGISTERED PROXY'})
    contacts=json.loads((root/'mechanical_contact_audit.json').read_text())
    contact_surfaces={}
    for name,witness in contacts['witnesses'].items():
        stamp=witness['time'];robot=witness['physical_pose'];actor=witness['physical_actor_pose'];labels=[]
        for key in sorted(all_keys):
            surface=rotation(actor,material_segment(*key,dimensions));gap=distance(rotation(robot,body),surface)
            for shape in shapes:
                if shape['kind']=='circle':gap=min(gap,max(0.,segment(*surface,rotation(robot,[shape['center']])[0])-shape['radius']))
            if gap>POLICY['margin_m']+1e-9:continue
            token=f'{key[0]}:{key[1]}'
            seen=[r for r in accepted if r['source_stamp']<=stamp and token in r['reliable_material_keys']]
            labels.append({'face':key[0],'material_bin':key[1],'gap_at_contact_event_m':gap,
                           'first_reliable_source_stamp':first_reliable.get(key),
                           'latest_prior_reliable_source_stamp':seen[-1]['source_stamp'] if seen else None,
                           'event_minus_latest_prior_reliable_s':stamp-seen[-1]['source_stamp'] if seen else None,
                           'actual_surface_membership_downstream':'UNKNOWN'})
        contact_surfaces[name]={'event_stamp':stamp,'near_trigger_material_bins':labels}
    first_events=[r for r in events if r['event_kind']=='first_reliable'];not_censored=[r for r in first_events if not r['left_censored_at_first_accepted_scan']]
    margins=[v['static_target_reveal_margin_m'] for r in not_censored for v in r['hypothetical_translation_stop_sensitivity']]
    delays=[r['receive_sim']-r['source_stamp_ns']/1e9 for r in source if r['kind']=='obstacles']
    tracker_wall=[float(s['values']['latency_ms']) for r in source if r['kind']=='tracker' for s in r['statuses'] if s['name']=='dynamic_obstacle_tracking_shadow' and 'latency_ms' in s['values']]
    return {'schema':'surface_reveal_audit/v1','verdict':'OFFLINE REVEAL LABELS; ACTUAL STOP/REPLAN SAFETY NOT ESTABLISHED',
            'source_manifest_sha256':manifest_sha,'registered_policy':POLICY,'fixture':scene,'accepted_scans':len(accepted),
            'skipped_scans':dict(skips),'label_failures':dict(failures),'material_bins':len(all_keys),'reliable_bins':len(first_reliable),
            'never_reliably_revealed_bins':len(all_keys-set(first_reliable)),'first_events':len(first_events),
            'non_left_censored_first_events':len(not_censored),'reappearances':len(events)-len(first_events),
            'non_left_censored_first_full_mechanical_gap_m':stats([r['full_mechanical_gap_m'] for r in not_censored]),
            'hypothetical_static_target_margin_m':stats(margins),'observer_scan_receipt_delay_s':stats([r['receive_sim']-r['source_stamp'] for r in accepted]),
            'observer_publication_receipt_minus_source_s':stats(delays),'tracker_wall_compute_ms':stats(tracker_wall),
            'actual_brake_deceleration_mps2':None,'actual_end_to_end_response_latency_s':None,'actual_safe_reveal_verdict':'NOT ESTABLISHED',
            'original_contact_witnesses':contacts['witnesses'],'material_inventory':material,'events':events,'scan_records':accepted,
            'contact_surface_observability':contact_surfaces,
            'limits':['Box/material/first-hit identities and interpolation use offline truth only; not a MID360 or all-ground-robot result.',
                      'Residual gate and consecutive scan rule are an audit proxy, not certified sensing or actual surface membership downstream.',
                      'Initial observations are left-censored; unsampled, occluded, missing and never-revealed surfaces remain unknown.',
                      'Observer receipt, source age and wall compute clocks cannot supply total perception/controller/serial/chassis response latency.',
                      'Sensitivity assumes translation-only deceleration and optionally frozen instantaneous actor speed; neither is a measured guaranteed bound.',
                      'A stopped robot can still be hit by an approaching actor; self stopping is not a controlled invariant or an escape proof.',
                      'Existing A/B trajectories are not moving-surface C trials; original physical/raw203/3s/task gates remain unchanged.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    report=audit(args.archive)
    with args.output.open('x') as stream:stream.write(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('material_inventory','events','scan_records','original_contact_witnesses')}))
