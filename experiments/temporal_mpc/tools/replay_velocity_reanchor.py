#!/usr/bin/env python3
"""Compile the real projection helper on a recorded velocity rejection case.

Checks model velocity, dynamics, corridor and all public geometry only. No oracle
or new execution claim; the original rejected v1 trial stays unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from audit_execution import native_states
from audit_run import namespace, epoch
from temporal_mpc.contracts import PublicAdapter, predict
from temporal_mpc.geometry import clearance


def evaluate(root,output):
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    failures=[json.loads(e['data']['data']) for e in events if e['topic']=='/temporal_mpc/health']
    d=next(d for d in failures if d['executed'] and d['constraint']=='velocity_y')
    proposals={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/temporal_mpc/proposal'}
    predictions={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/dynamic_obstacle_predictions'}
    p=proposals[d['proposal_ns']];prediction=predictions[d['prediction_ns']]
    case=dict(health=d,proposal=p,prediction=prediction,event_sha256=hashlib.sha256((root/'events.jsonl').read_bytes()).hexdigest())
    (output/'case.json').write_text(json.dumps(case,indent=2)+'\n')
    age=(d['evaluation_ns']-d['proposal_ns'])*1e-9
    controls=[[a['x'],a['y']] for a in p['accelerations']]
    rows=','.join('{'+','.join(map(repr,r))+'}' for r in controls)
    x=d['initial_state'];source=output/'replay.cpp';binary=output/'replay'
    source.write_text('#include "stopping.hpp"\n#include <iostream>\n#include <iomanip>\n#include <cmath>\nint main(){\n'
        +f'double input[30][2]={{{rows}}}; double x={x[0]!r},y={x[1]!r},vx={x[3]!r},vy={x[4]!r};\n'
        +f'const double c=std::cos({x[2]!r}),s=std::sin({x[2]!r}),age={age!r};\n'
        +'std::cout<<std::setprecision(17)<<x<<" "<<y<<" "<<vx<<" "<<vy<<"\\n";\n'
        +'for(int k=0;k<30;k++){double ax=0.,ay=0.,from=age+k*.05,to=from+.05;\n'
        +'if(k>=27){ax=std::clamp(-vx/.05,-1.,1.);ay=std::clamp(-vy/.05,-1.,1.);}\n'
        +'else for(int i=0;i<30;i++){double w=std::max(0.,std::min(to,(i+1)*.05)-std::max(from,i*.05))/.05;ax+=w*input[i][0];ay+=w*input[i][1];}\n'
        +'ax=rm_temporal_mpc::bounded_stopping_acceleration(vx,ax,(29-k)*.05,-.5,.8);\n'
        +'ay=rm_temporal_mpc::bounded_stopping_acceleration(vy,ay,(29-k)*.05,-.5,.5);\n'
        +'vx+=.05*ax;vy+=.05*ay;x+=.05*(c*vx-s*vy);y+=.05*(s*vx+c*vy);\n'
        +'std::cout<<x<<" "<<y<<" "<<vx<<" "<<vy<<"\\n";} }\n')
    include=Path(__file__).resolve().parents[1]/'ros2/rm_temporal_mpc_controller/include'
    subprocess.run(['g++','-std=c++17','-O2','-I',str(include),str(source),'-o',str(binary)],check=True,capture_output=True)
    text=subprocess.check_output([str(binary)],text=True);(output/'native_states.txt').write_text(text)
    points=np.array([[float(v) for v in line.split()] for line in text.splitlines()])
    v2=dict(d,reanchor_projection='velocity_and_stop/v2');new=native_states(v2,p);old=native_states(d,p)
    error=float(np.max(np.abs(points-new[:,[0,1,3,4]])))
    snapshot=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(prediction),d['evaluation_ns'])
    timeline=predict(snapshot,d['evaluation_ns'],np.arange(31)*.05)
    reserve=.025*np.hypot(.8,.5);b=p['centre_bounds']
    minima=[new[:,0]-b[0]-reserve,b[1]-new[:,0]-reserve,new[:,1]-b[2]-reserve,b[3]-new[:,1]-reserve,
            (new[:,3:5]-[-.5,-.5]).ravel(),([.8,.5]-new[:,3:5]).ravel()]
    for centers,shape,speed in zip(timeline.centers,timeline.geometries,timeline.speeds):
        minima.append(clearance(new,centers,shape,(.355,.330))-.02-reserve-.025*speed)
    minimum=float(np.min(np.concatenate(minima)))
    acceleration=np.diff(new[:,3:5],axis=0)/.05
    result=dict(original_velocity_y_slack=d['slack'],original_vy_min=float(np.min(old[:,4])),
                projected_vy_min=float(np.min(new[:,4])),cpp_python_state_error_max=error,
                all_public_track_count=len(timeline.track_ids),constraint_min=minimum,
                target_change_acceleration_max=float(np.max(np.abs(acceleration))),terminal_speed=float(np.max(np.abs(new[-1,3:]))),
                all_model_checks_pass=bool(error<1e-12 and minimum>=-1e-6 and np.max(np.abs(acceleration))<=1.+1e-9 and np.max(np.abs(new[-1,3:]))<1e-5),
                scope='Recorded v1 rejection replayed with v2 projection only. No new plant/TF/STVL execution or physical acceptance claim.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise SystemExit('refuse overwrite')
    out.mkdir(parents=True)
    print(json.dumps(evaluate(Path(args.run),out),indent=2))
