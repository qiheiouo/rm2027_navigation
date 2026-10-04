#!/usr/bin/env python3
"""Explicit experimental entry gate: actual sensor replay + pre-goal measured state.

Post-impact MPPI state failures remain in the full audit. They do not establish
that an unimpacted, calibrated plant cannot start a separately guarded MPC run.
"""
import argparse
import hashlib
import json
from pathlib import Path
from audit_run import epoch


def model_ok(d):
    q=d['pose']['pose']['orientation'];v=d['twist']['twist']
    return (d['header']['frame_id']=='odom' and d['child_frame_id']=='base_link'
            and abs(q['x'])<=1e-6 and abs(q['y'])<=1e-6
            and abs(v['angular']['z'])<=1e-8 and -.5<=v['linear']['x']<=.8
            and abs(v['linear']['y'])<=.5)


def evaluate(shadow,calibration):
    shadow,calibration=Path(shadow),Path(calibration)
    a=json.loads((shadow/'audit.json').read_text());c=json.loads((calibration/'calibration_audit.json').read_text())
    s=json.loads((shadow/'run_summary.json').read_text());end=s['goal_epoch_s']
    readings=[json.loads(line) for line in (shadow/'events.jsonl').open()]
    prefix=[e['data'] for e in readings if e['topic']=='/odometry/lio' and end-1<=epoch(e['data']['header']['stamp'])/1e9<end-.02]
    pregoal=bool(len(prefix)>=30 and all(model_ok(d) for d in prefix))
    responses=c['pulses']; errors=[]
    for response in responses:
        measured=response['measured_body_velocity_median'];target=response['target_body_velocity']
        if measured is None: errors.append(float('inf'))
        else: errors.append(max(abs(m-t) for m,t in zip(measured,target)))
    calibration_gate=bool(c['body_model_gate'] and len(errors)==4 and max(errors)<1e-4)
    return dict(mpc_entry_gate=bool(a['sensor_contract_gate'] and pregoal and calibration_gate),
                sensor_contract_gate=a['sensor_contract_gate'],pre_goal_model_gate=pregoal,
                pre_goal_measured_samples=len(prefix),calibration_gate=calibration_gate,
                full_shadow_model_gate=a['measured_model_gate'],full_shadow_physical_acceptance=a['physical_acceptance'],
                criterion='Experimental start only; all runtime revalidation/fallback stays active, no deployment acceptance',
                prerequisite_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
                    shadow/'audit.json',shadow/'events.jsonl',shadow/'ros_capture_audit.json',
                    calibration/'events.jsonl',calibration/'calibration_audit.json')})

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('shadow');parser.add_argument('calibration');parser.add_argument('output')
    args=parser.parse_args();result=evaluate(args.shadow,args.calibration)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
