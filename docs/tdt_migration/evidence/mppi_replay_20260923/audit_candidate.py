#!/usr/bin/env python3
"""Cross-check one-variable pilot against frozen raw evidence without rewriting trials."""
import bisect, hashlib, json, math, sys
from pathlib import Path
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASE=ROOT/'build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2'
SERIES=ROOT/'build/tdt_p2b/runs/dynamic_odom_routing_pilot_v1'
PREV=ROOT/'docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922'
sys.path.insert(0,str(PREV))
sys.path.insert(0,str(ROOT/'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from read_trace import read_cycle
from audit_dynamic import inspect_dynamic_trial
from dynamic_metrics import rows_from_transport,obstacle_polygon,geometry_metrics

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    inputs=json.loads((SERIES/'inputs.json').read_text())
    previous=json.loads((PREV/'manifest.json').read_text())
    missing=[];modified=[]
    for rel,digest in previous['files'].items():
        path=ROOT/rel
        if not path.is_file():missing.append(rel)
        elif sha(path)!=digest:modified.append(rel)
    assert not missing and not modified,(missing[:3],modified[:3])
    for block in ('files','diagnostic_tools','diagnostic_libraries'):
        for rel,digest in inputs[block].items():assert sha(ROOT/rel)==digest,rel
    assert sha(HERE/'run_candidate.py')==inputs['candidate_tool_sha256']
    result={'schema':'tdt_mppi_odom_pilot_audit/v1','preserved_previous_manifest_entries':len(previous['files']),
            'previous_missing':missing,'previous_modified':modified,'trials':{}}
    for name in ('tdt_astar','tdt_qp'):
        base=BASE/f'{name}_1';d=SERIES/f'{name}_1'
        source=BASE/'profiles'/f'{name}.yaml';candidate=SERIES/'profiles'/f'{name}.yaml'
        assert sha(source)==inputs['source_profiles'][name]
        assert sha(candidate)==inputs['profiles'][name]==sha(d/'profile.yaml')
        a=yaml.safe_load(source.read_text());b=yaml.safe_load(candidate.read_text())
        assert 'odom_topic' not in a['controller_server']['ros__parameters']
        assert b['controller_server']['ros__parameters'].pop('odom_topic')=='/odometry/lio'
        assert a==b
        meta=json.loads((d/'metadata.json').read_text())
        assert meta['profile_sha256']==sha(candidate)
        raw=inspect_dynamic_trial(d)
        summary=json.loads((d/'dynamic_summary.json').read_text())
        assert raw['raw_summary_matches'] and summary['original_static_summary_matches']
        assert summary['evidence_valid']
        for key in ('action_status','recoveries','final_xy_error_m','final_yaw_error_rad','cross_track_rms_m'):
            assert summary[key]==raw['summary'][key],key
        rows=rows_from_transport(d/'gazebo_poses.jsonl');times=[r['t'] for r in rows]
        start,end=summary['navigation_interval_sim_s']
        lo=max(0,bisect.bisect_right(times,start)-1);hi=bisect.bisect_left(times,end)
        geom=geometry_metrics(rows[lo:hi+1],inputs['body_polygon_m'],.03,obstacle_polygon())
        assert json.dumps(geom,sort_keys=True)==json.dumps(summary['geometry']['new_car_reference'],sort_keys=True)
        cycles=[]
        for path in (d/'mppi_cycles').glob('cycle_*.json'):
            m,arrays=read_cycle(path)
            cycles.append(m)
        assert cycles
        old_summary=json.loads((base/'dynamic_summary.json').read_text())
        old_cycles=[json.loads(path.read_text()) for path in (base/'mppi_cycles').glob('cycle_*.json')]
        item={'profile_sha256':sha(candidate),'source_profile_sha256':sha(source),
              'raw_summary_matches':True,'geometry_recomputed':True,
              'runtime_geometry_preflight_present':(d/'runtime_geometry_preflight.json').is_file(),
              'runtime_geometry_after_present':(d/'runtime_geometry.json').is_file(),
              'candidate_cycles':len(cycles),'candidate_exception_cycles':sum(c['unwinding_exception'] for c in cycles),
              'candidate_output_cycles':sum(any(e['kind']=='output' for e in c['events']) for c in cycles),
              'candidate_nonzero_speed_cycles':sum(any(abs(v)>1e-4 for v in c['speed']) for c in cycles),
              'baseline_cycles':len(old_cycles),'baseline_nonzero_speed_cycles':sum(any(abs(v)>1e-4 for v in c['speed']) for c in old_cycles),
              'candidate':{key:summary[key] for key in ('action_status','recoveries','final_xy_error_m','final_yaw_error_rad','cross_track_rms_m','checks','limited_dynamic_geometry_and_goal_pass')},
              'candidate_new_car_body':geom['body'],'candidate_new_car_padded':geom['padded'],
              'baseline':{key:old_summary[key] for key in ('action_status','recoveries','final_xy_error_m','final_yaw_error_rad','cross_track_rms_m','checks','limited_dynamic_geometry_and_goal_pass')},
              'baseline_new_car_body':old_summary['geometry']['new_car_reference']['body'],
              'baseline_new_car_padded':old_summary['geometry']['new_car_reference']['padded']}
        assert item['runtime_geometry_preflight_present'] and item['runtime_geometry_after_present']
        result['trials'][name]=item
    out=HERE/'candidate_audit.json'
    with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({name:{'nonzero_speed':r['candidate_nonzero_speed_cycles'],'cycles':r['candidate_cycles'],
        'action':r['candidate']['action_status'],'recoveries':r['candidate']['recoveries'],
        'body_bound':r['candidate_new_car_body']['moving_interpolation_bound_m'],
        'padded_overlap':r['candidate_new_car_padded']['sampled_moving_overlap']}
        for name,r in result['trials'].items()},indent=2))

if __name__=='__main__':main()
