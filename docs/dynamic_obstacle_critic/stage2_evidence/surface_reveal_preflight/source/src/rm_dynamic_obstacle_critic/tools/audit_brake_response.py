#!/usr/bin/env python3
"""Empirical frozen-trace stop episodes, never a guaranteed brake bound."""
import argparse,bisect,json,math
from pathlib import Path
from analyze_trial import load_truth_rows
from audit_surface_reveal import validate
from trial_io import rows

POLICY={'maximum_command_receipt_gap_s':.1,'maximum_prior_odom_age_s':.04,
        'minimum_initial_translation_speed_mps':.05,'stop_translation_speed_mps':.005,
        'stop_yaw_rate_radps':.01,'sustained_stop_s':.2,'maximum_observation_s':2.,'maximum_odom_gap_s':.04}


def zero(row):return all(v==0 for v in row['velocity'])


def zero_episodes(commands):
    episodes=[];start=None;previous=None
    for i,row in enumerate(commands):
        gap=previous is None or row['receive_sim']-previous['receive_sim']>POLICY['maximum_command_receipt_gap_s']
        if start is not None and (gap or not zero(row)):
            episodes.append({'first':commands[start],'last':previous,'previous':commands[start-1] if start else None,
                             'end_reason':'command_receipt_gap' if gap else 'nonzero_command_resumed'})
            start=None
        if zero(row) and start is None:start=i
        previous=row
    if start is not None:episodes.append({'first':commands[start],'last':previous,'previous':commands[start-1] if start else None,'end_reason':'trace_end'})
    return episodes


def sustained_stop(odom,start,end):
    candidate=None;previous=None
    for row in odom:
        if not start<=row['stamp']<=end:continue
        if previous is not None and row['stamp']-previous>POLICY['maximum_odom_gap_s']:candidate=None
        vx,vy,wz=row['velocity'];stopped=math.hypot(vx,vy)<=POLICY['stop_translation_speed_mps'] and abs(wz)<=POLICY['stop_yaw_rate_radps']
        if not stopped:candidate=None
        elif candidate is None:candidate=row['stamp']
        if candidate is not None and row['stamp']-candidate>=POLICY['sustained_stop_s']:
            return {'stop_source_stamp':candidate,'confirmation_source_stamp':row['stamp']}
        previous=row['stamp']
    return None


def audit(root):
    manifest=validate(root);execution=json.loads((root/'execution.json').read_text())
    if execution['graph']['/cmd_vel']!=['dynamic_safety_guard']:raise ValueError('single final guard publisher required')
    log=list(rows(root,'observations.jsonl'))
    commands=[r for r in log if r['kind']=='final_cmd' and execution['start_sim']<=r['receive_sim']<=execution['last_sim']]
    odom=sorted((r for r in log if r['kind']=='canonical_odom'),key=lambda r:r['stamp']);unique=[]
    for r in odom:
        if r['frame']!='odom' or r['child_frame']!='base_link' or not all(math.isfinite(v) for v in r['velocity']):raise ValueError('canonical finite velocity required')
        if not unique or r['stamp']>unique[-1]['stamp']:unique.append(r)
    odom=unique;times=[r['stamp'] for r in odom];truth=load_truth_rows(root,execution)
    results=[]
    for ordinal,episode in enumerate(zero_episodes(commands)):
        stamp=episode['first']['receive_sim'];previous=episode['previous']
        row={'ordinal':ordinal,'zero_command_first_observer_stamp':stamp,'zero_command_last_observer_stamp':episode['last']['receive_sim'],
             'zero_episode_end_reason':episode['end_reason'],'command_emission_or_actuator_stamp':None,'actual_end_to_end_delay_s':None}
        if previous is None or zero(previous):row['verdict']='UNKNOWN START / COMMAND GAP';results.append(row);continue
        j=bisect.bisect_right(times,stamp)-1
        if j<0 or stamp-times[j]>POLICY['maximum_prior_odom_age_s']:row['verdict']='MISSING PRIOR SOURCE ODOM';results.append(row);continue
        measured=odom[j];vx,vy,wz=measured['velocity'];speed=math.hypot(vx,vy)
        row.update(initial_odom_source_stamp=measured['stamp'],initial_measured_twist=measured['velocity'],initial_translation_speed_mps=speed,
                   observer_minus_initial_odom_source_s=stamp-measured['stamp'])
        if speed<POLICY['minimum_initial_translation_speed_mps']:row['verdict']='LOW INITIAL TRANSLATION SPEED; NOT A BRAKE CAPABILITY SAMPLE';results.append(row);continue
        end=min(episode['last']['receive_sim'],stamp+POLICY['maximum_observation_s']);stop=sustained_stop(odom,stamp,end)
        if stop is None:row['verdict']='NO SUSTAINED STOP WITHIN AVAILABLE ZERO EPISODE';results.append(row);continue
        samples=[r for r in truth if measured['stamp']<=r['t']<=stop['stop_source_stamp']]
        if len(samples)<2 or samples[0]['t']-measured['stamp']>.04 or stop['stop_source_stamp']-samples[-1]['t']>.04:
            row['verdict']='MISSING PHYSICAL PATH BRACKET';results.append(row);continue
        displacement=sum(math.dist(a['robot'][:2],b['robot'][:2]) for a,b in zip(samples,samples[1:]))
        angular_travel=sum(abs(math.remainder(b['robot'][2]-a['robot'][2],2*math.pi)) for a,b in zip(samples,samples[1:]))
        elapsed=stop['stop_source_stamp']-measured['stamp']
        row.update(verdict='OBSERVED SIMULATED STOP',**stop,source_elapsed_s=elapsed,physical_translation_path_m=displacement,
                   physical_yaw_travel_rad=angular_travel,physical_path_first_stamp=samples[0]['t'],physical_path_last_stamp=samples[-1]['t'],
                   equivalent_time_deceleration_mps2=speed/elapsed if elapsed>0 else None,
                   equivalent_distance_deceleration_mps2=speed*speed/(2*displacement) if displacement>0 else None,
                   odom_speed_samples=[{'source_stamp':r['stamp'],'twist':r['velocity']} for r in odom if measured['stamp']<=r['stamp']<=stop['confirmation_source_stamp']])
        results.append(row)
    usable=[r for r in results if r['verdict']=='OBSERVED SIMULATED STOP']
    return {'schema':'empirical_brake_response/v1','verdict':'TRACE OBSERVATIONS ONLY; GUARANTEED BRAKE BOUND NOT ESTABLISHED',
            'source_manifest_sha256':manifest,'registered_policy':POLICY,'zero_episodes':len(results),'observed_stop_episodes':len(usable),
            'minimum_observed_equivalent_time_deceleration_mps2':min((r['equivalent_time_deceleration_mps2'] for r in usable),default=None),
            'maximum_observed_source_elapsed_s':max((r['source_elapsed_s'] for r in usable),default=None),
            'actual_end_to_end_response_delay_s':None,'guaranteed_brake_deceleration_lower_mps2':None,'episodes':results,
            'limits':['Command zero epochs are observer receipts, not emission or actuator epochs; cannot measure total reaction latency.',
                      'Equivalent decelerations include selected trace history and sampling/bracket error; not a measured guaranteed limit or real chassis capability.',
                      'Initial yaw and physical yaw travel are recorded; scalar translation stopping does not certify a rotating footprint.',
                      'Self stopping does not guarantee escape from an approaching target. Original physical/raw203/3s/task gates remain separate.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    result=audit(args.archive)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='episodes'}))
