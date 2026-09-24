#!/usr/bin/env python3
"""Read-only first-trial A/B audit using the pre-existing independent polygon oracle."""
import bisect,hashlib,json,math,re,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
SERIES=ROOT/'build/tdt_p2b/runs/dynamic_prediction_mppi_ab_v2'
sys.path.insert(0,str(ROOT/'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport,obstacle_polygon,geometry_metrics
POLYGON=json.loads((ROOT/'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json').read_text())['body_polygon_m']
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read_rows(path):return [json.loads(line) for line in path.open()]
def stall_metrics(trajectory,start,end):
    # Query only; no new runtime safety threshold. Stop: planar odom speed <=.03
    # while still >.3 m from nominal goal for >=.3 s.
    seq=[r for r in trajectory if start<=r['t']<=end]
    episodes=[];active=None;total=0.
    for a,b in zip(seq,seq[1:]):
        dt=b['t']-a['t']
        stopped=math.hypot(a['vx'],a['vy'])<=.03 and math.hypot(a['x']-5.6,a['y'])>.3
        if stopped:
            total+=dt
            if active is None:active=a['t']
        elif active is not None:
            if a['t']-active>=.3:episodes.append([active,a['t']])
            active=None
    if active is not None and seq[-1]['t']-active>=.3:episodes.append([active,seq[-1]['t']])
    return {'low_speed_time_s':total,'episodes_at_least_030s':episodes,
            'rule':'odom planar speed <=0.03 m/s while >0.3m from goal; episodes >=0.3s'}
def trial(mode,planner):
    p=SERIES/f'{mode}_{planner}_1';metadata=json.loads((p/'metadata.json').read_text())
    assert sha(p/'profile.yaml')==metadata['profile_sha256']
    for name,digest in metadata['file_sha256'].items():assert sha(ROOT/name)==digest,name
    s=json.loads((p/'observation/summary.json').read_text())
    assert s['evidence_valid'] and s['goal']==[5.6,0.,0.]
    events=read_rows(p/'observation/events.jsonl')
    pre=[e for e in events if e.get('event')=='preflight'];nav=[e for e in events if e.get('event')=='navigation_result']
    assert len(pre)==len(nav)==1 and pre[0]['result']==s['preflight']
    assert nav[0]['status']==s['action_status'] and nav[0]['recoveries']==s['recoveries']
    start,end=pre[0]['t'],nav[0]['t']
    poses=rows_from_transport(p/'gazebo_poses.jsonl');times=[r['t'] for r in poses]
    assert times[0]<=start and times[-1]>=end
    lo=max(0,bisect.bisect_right(times,start)-1);hi=bisect.bisect_left(times,end)
    sample=poses[lo:hi+1]
    assert max(b['t']-a['t'] for a,b in zip(sample,sample[1:]))<.2
    geom=geometry_metrics(sample,POLYGON,.03,obstacle_polygon())
    checks=dict(s['checks'])
    checks['body_clearance_at_least_005m']=geom['body_clearance_at_least_005m']
    checks['padded_footprint_no_contact']=geom['padded_footprint_no_contact']
    launch=(p/'launch.log').read_text(errors='replace')
    consumption=[]
    for line in launch.splitlines():
        m=re.search(r'PredictionV1Critic consumed source age ([0-9.]+) s, tracks (\d+), predicted-collision trajectories (\d+)/(\d+), near samples (\d+), cycles (\d+)',line)
        if m:consumption.append({'age_s':float(m[1]),'tracks':int(m[2]),'collision_trajectories':int(m[3]),'trajectory_count':int(m[4]),'near_samples':int(m[5]),'accepted_cycles_total':int(m[6])})
    if mode=='candidate':
        assert 'Critic loaded : mppi::critics::PredictionV1Critic' in launch
        assert consumption and (p/'predictions.jsonl').exists()
        predictions=read_rows(p/'predictions.jsonl')
        assert predictions and any(row['complete'] and row['frame']=='odom' for row in predictions)
    else:
        assert not consumption and 'Critic loaded : mppi::critics::PredictionV1Critic' not in launch
        predictions=[]
    return {'mode':mode,'planner':planner,'source_commit':metadata['source_commit'],'profile_sha256':metadata['profile_sha256'],
      'raw_summary_matches_events':True,'evidence_valid':True,'preflight_status':s['preflight']['status'],
      'action_status':s['action_status'],'recoveries':s['recoveries'],
      'final_xy_error_m':s['final_xy_error_m'],'final_yaw_error_rad':s['final_yaw_error_rad'],
      'navigation_interval_sim_s':[start,end],'navigation_duration_sim_s':end-start,
      'traveled_m':s['traveled_m'],'cross_track_rms_m':s['cross_track_rms_m'],
      'stall':stall_metrics(read_rows(p/'observation/trajectory.jsonl'),start,end),
      'geometry':geom,'checks':checks,'limited_dynamic_gate_pass':all(checks.values()),
      'prediction_messages':len(predictions),'prediction_consumption_log_samples':consumption,
      'prediction_skips_logged':launch.count('PredictionV1Critic skipped prediction'),
      'source_sha256':{str(path.relative_to(ROOT)):sha(path) for path in
        [p/'metadata.json',p/'profile.yaml',p/'observation/summary.json',p/'observation/events.jsonl',
         p/'observation/trajectory.jsonl',p/'gazebo_poses.jsonl',p/'launch.log']}}
def main():
    data={'schema':'rm_dynamic_prediction_v1_mppi_first_ab/v1',
      'scope':'Fixed phase QP first trials only; different simulated runs, not paired-frame A/B or a multi-phase acceptance matrix.',
      'accepted_for_deployment':False,'trials':[trial(mode,planner) for planner in ('tdt_qp','tdt_astar') for mode in ('baseline','candidate')]}
    target=HERE/'ab_audit_v2.json'
    with target.open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n')
    for t in data['trials']:
        g=t['geometry'];print(json.dumps({'mode':t['mode'],'planner':t['planner'],'action':t['action_status'],
          'recoveries':t['recoveries'],'duration':t['navigation_duration_sim_s'],
          'moving_body_bound':g['body']['moving_interpolation_bound_m'],
          'moving_padded_bound':g['padded']['moving_interpolation_bound_m'],
          'pass':t['limited_dynamic_gate_pass'],'prediction_samples':len(t['prediction_consumption_log_samples'])}))
if __name__=='__main__':main()
