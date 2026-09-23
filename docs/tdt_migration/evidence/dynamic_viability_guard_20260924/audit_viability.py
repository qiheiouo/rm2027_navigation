#!/usr/bin/env python3
"""Audit the frozen viability pilot against raw decisions, maps and geometry."""
import collections
import hashlib
import json
import math
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUNS=ROOT/'build/tdt_p2b/runs'
SERIES=RUNS/'dynamic_viability_guard_pilot_v1'
TRIAL=SERIES/'tdt_qp_1'
sys.path.insert(0,str(HERE.parent/'dynamic_guard_pilot_20260923'))
from guard_core import BODY_GATE_M,HOLD_S,certificate,fixture_sweep,padded,radius


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(x) for x in p.open()]
def cell_at(g,x,y):
    ox,oy=g['origin'];r=g['resolution']
    ix,iy=math.floor((x-ox)/r),math.floor((y-oy)/r)
    if not (0<=ix<g['width'] and 0<=iy<g['height']):return None
    return g['data'][iy*g['width']+ix]


def main():
    m=json.loads((SERIES/'inputs.json').read_text())
    old=RUNS/'dynamic_guard_horizon_pilot_v2/profile.yaml'
    assert sha(SERIES/'profile.yaml')==sha(old)==m['profile_sha256']
    assert sha(TRIAL/'profile.yaml')==m['profile_sha256']
    for p,d in m['files'].items():assert sha(ROOT/p)==d,p
    s=json.loads((TRIAL/'observation/summary.json').read_text())
    d=json.loads((TRIAL/'dynamic_summary.json').read_text())
    assert s['evidence_valid'] and d['evidence_valid'] and d['original_static_summary_matches']
    a=rows(TRIAL/'guard_decisions.jsonl')
    assert a[0]['kind']=='configuration' and a[1]['kind']=='policy'
    body=a[0]['body'];sweep,fixture=fixture_sweep()
    rb=radius(body);rp=radius(padded(body,.03))
    decision=[x for x in a if x['kind']=='decision']
    counts=collections.Counter(x['reason'] for x in decision)
    assert sum(counts.values())==len(decision)
    current_stop_bad=[];next_zero_bad=[]
    for x in decision:
        v=max(math.hypot(*x['odom_speed'][:2]),math.hypot(*x['emitted'][:2]))
        w=max(abs(x['odom_speed'][2]),abs(x['emitted'][2]))
        current=certificate(x['pose'],x['odom_speed'],x['emitted'],body,sweep,
                            hold_s=HOLD_S+.05+max(0,x['odom_age_s']))
        assert current['safe_under_model']==x['certificate']['current']['safe_under_model']
        for key in ('body_gap_m','padded_gap_m','body_lower_m','padded_lower_m'):
            assert abs(current[key]-x['certificate']['current'][key])<1e-9,(x['sim_s'],key)
        reach=v*(.12+.30)+v*v/2
        angle=w*(.12+.30)+w*w/4
        bl=current['body_gap_m']-reach-rb*angle
        pl=current['padded_gap_m']-reach-rp*angle
        assert abs(bl-x['certificate']['next_body_lower_m'])<1e-9
        assert abs(pl-x['certificate']['next_padded_lower_m'])<1e-9
        combined=current['safe_under_model'] and bl>=BODY_GATE_M and pl>0
        assert combined==x['certificate']['safe_under_model']
        if x['reason']=='admit':
            assert x['scale']==1 and x['emitted']==x['proposed'] and combined
        elif x['reason']=='scale_viability':
            assert 0<x['scale']<1 and combined and not x['full_command_certificate']['safe_under_model']
            assert all(abs(y-x['scale']*z)<1e-11 for y,z in zip(x['emitted'],x['proposed']))
        elif x['reason']=='reject_next_unstoppable':
            assert x['scale']==0 and x['emitted']==[0,0,0] and not combined
            next_zero_bad.append(x)
            if not current['safe_under_model']:current_stop_bad.append(x)
        else:raise AssertionError(x['reason'])
    chain=rows(TRIAL/'observation/command_chain.jsonl')
    chain_counts=collections.Counter(x['topic'] for x in chain)
    assert chain_counts['/cmd_vel']==chain_counts['/cmd_vel_guarded']==chain_counts['/simulation/chassis/cmd_vel']
    maps={}
    for name,file,lethal in (('global','costmap.jsonl',100),('local','raw_local_maps.jsonl',254)):
        all_maps=rows(TRIAL/'observation'/file)
        checked=[g for g in all_maps if g['t']>=24 and cell_at(g,4.9,0)is not None]
        assert checked and all(cell_at(g,4.9,0)==lethal for g in checked)
        maps[name]={'recorded':len(all_maps),'sweep_center_checked':len(checked),'lethal':len(checked)}
    geom=d['geometry']['new_car_reference']
    gaps=[y['sim_s']-x['sim_s'] for x,y in zip(decision,decision[1:])]
    result={
      'schema':'tdt_dynamic_viability_guard_audit/v1',
      'profile_sha256':m['profile_sha256'],'same_profile_as_horizon_v2':True,
      'fixture_source':fixture,
      'docker_exit':int((TRIAL/'docker_exit.txt').read_text()),
      'runtime_geometry_capture_exit':int((TRIAL/'geometry_capture_exit.txt').read_text()),
      'preflight_status':s['preflight']['status'],'action_status':s['action_status'],
      'timed_out':s['timed_out'],'recoveries':s['recoveries'],
      'final_xy_error_m':s['final_xy_error_m'],'final_yaw_error_rad':s['final_yaw_error_rad'],
      'cross_track_rms_m':s['cross_track_rms_m'],
      'checks':d['checks'],'limited_dynamic_geometry_and_goal_pass':d['limited_dynamic_geometry_and_goal_pass'],
      'body_moving_interpolation_bound_m':geom['body']['moving_interpolation_bound_m'],
      'padded_moving_interpolation_bound_m':geom['padded']['moving_interpolation_bound_m'],
      'max_gazebo_gap_s':d['continuity']['max_gazebo_gap_s'],
      'guard_decisions_recomputed':len(decision),'guard_decisions_by_reason':dict(counts),
      'current_stop_model_failure_on_zero':len(current_stop_bad),
      'next_decision_model_failure_on_zero':len(next_zero_bad),
      'next_decision_failure_time_range_s':[min(x['sim_s'] for x in next_zero_bad),
                                             max(x['sim_s'] for x in next_zero_bad)] if next_zero_bad else None,
      'min_current_body_lower_m':min(x['certificate']['current']['body_lower_m'] for x in decision),
      'min_next_body_lower_m':min(x['certificate']['next_body_lower_m'] for x in decision),
      'min_next_padded_lower_m':min(x['certificate']['next_padded_lower_m'] for x in decision),
      'max_odom_age_s':max(x['odom_age_s'] for x in decision),
      'max_guard_decision_gap_s':max(gaps),
      'command_chain_counts':dict(chain_counts),
      'costmap_marking':maps,
      'qp_adopted':d['qp_adopted'],'validated_astar_fallbacks':d['validated_astar_fallbacks'],
      'accepted_for_deployment':False}
    with (HERE/'viability_audit.json').open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')

if __name__=='__main__':main()
