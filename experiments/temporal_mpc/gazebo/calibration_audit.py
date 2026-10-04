#!/usr/bin/env python3
"""Measured pulse response summary; commands are not used as odometry."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from audit_run import epoch, stats

PULSES=((2.,5.,.4,0.),(8.,11.,0.,.3),(14.,17.,.4,.3),(20.,23.,-.3,0.))

def audit(root):
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    odom=[e['data'] for e in events if e['topic']=='/odometry/lio']
    readings=[]
    for d in odom:
        v=d['twist']['twist'];q=d['pose']['pose']['orientation']
        yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
        readings.append([epoch(d['header']['stamp'])/1e9,v['linear']['x'],v['linear']['y'],v['angular']['z'],yaw])
    data=np.array(readings)
    pulses=[]
    for begin,end,x,y in PULSES:
        steady=data[(data[:,0]>=begin+1.5)&(data[:,0]<end)]
        pulses.append(dict(interval=[begin,end],target_body_velocity=[x,y,0.],
                           steady_sample_count=len(steady),
                           measured_body_velocity_median=np.median(steady[:,1:4],axis=0).tolist() if len(steady) else None,
                           measured_wz_abs_max=float(np.max(np.abs(steady[:,3]))) if len(steady) else None))
    dv=np.diff(data[:,1:4],axis=0)/np.diff(data[:,0])[:,None]
    cmds=[e['data'] for e in events if e['topic']=='/cmd_vel']
    return dict(pulses=pulses,canonical_odom_count=len(odom),measured_wz_abs=stats(np.abs(data[:,3]).tolist()),
                measured_yaw_range_rad=[float(np.min(data[:,4])),float(np.max(data[:,4]))],
                measured_accel_abs_max=np.max(np.abs(dv),axis=0).tolist(),
                command_wz_abs_max=max([abs(c['angular']['z']) for c in cmds],default=None),
                body_model_gate=bool(np.max(np.abs(data[:,3]))<=1e-8),
                note='Steady response diagnostics only; no lag/continuous error bound or calibration compensation certified')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    result=audit(root);(root/'calibration_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
