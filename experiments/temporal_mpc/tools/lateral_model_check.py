#!/usr/bin/env python3
"""Illustrative fixed-yaw synthetic observations, never physical acceptance."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from temporal_mpc.contracts import Snapshot, Track, Geometry, NOMINAL_DIAMETER
from temporal_mpc.frontend import prepare_route
from temporal_mpc.realtime_qp import RealtimeMPC
from temporal_mpc.dynamics import rollout_zoh


def run(binary,moving):
    m=RealtimeMPC();x=np.zeros(6);history=[];failures=Counter();modes=Counter()
    route=prepare_route(binary,np.zeros((120,160),np.uint8),.05,(-1.,-3.),(0.,0.),(5.6,0.))
    for k in range(800):
        t=k*.05;ns=k*50_000_000
        center=max(1.175,4.675-.5*t) if moving else 2.5
        speed=-.5 if moving and center>1.175 else 0.
        s=Snapshot(ns,(Track(1,(center,0.),(speed,0.),ns,'confirmed',Geometry('circle',radius=NOMINAL_DIAMETER,source='synthetic full-D observation hypothesis')),))
        r=m.solve(x,ns,s,route.window(x,ns,m.times));modes[m.lateral.mode]+=1
        if not r.model_feasible:failures[r.reason]+=1
        x=rollout_zoh(x,[(r.command-x[3:])/.05],.05)[1]
        history.append(dict(t=t,state=x.tolist(),feasible=r.model_feasible,reason=r.reason,mode=m.lateral.mode,slack=r.constraint_min,elapsed_s=r.elapsed_s))
        if np.linalg.norm(x[:2]-[5.6,0.])<.2:break
    return dict(scope='Synthetic public observation and held-target model only; no plant, tracker, scheduling or physical certificate',
                goal_reached=bool(np.linalg.norm(x[:2]-[5.6,0.])<.2),final=x.tolist(),steps=len(history),
                uncertified=dict(failures),modes=dict(modes),history=history)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('binary');p.add_argument('output');args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise SystemExit('refuse overwrite')
    result={name:run(args.binary,moving) for name,moving in [('stationary',False),('head_on',True)]}
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({name:{k:v for k,v in r.items() if k!='history'} for name,r in result.items()},indent=2))
