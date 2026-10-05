#!/usr/bin/env python3
"""Read completed A16 CSVs; geometric sensitivity only, no runtime model/solver."""
import argparse
import csv
import hashlib
import json
import math
import pathlib
import re
import shutil
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    values=sorted(values)
    def percentile(p):
        at=(len(values)-1)*p;lo=int(at);hi=math.ceil(at)
        return values[lo]+(at-lo)*(values[hi]-values[lo])
    return dict(n=len(values),median=percentile(.5),p95=percentile(.95),min=values[0],max=values[-1]) if values else dict(n=0,median=None,p95=None,min=None,max=None)


def extremum(a,b,lo,hi,maximum):
    # a*cos(theta)+b*sin(theta): include endpoints and every interior extremum.
    phase=math.atan2(b,a)+(0 if maximum else math.pi)
    values=[a*math.cos(t)+b*math.sin(t) for t in [lo,hi]]
    first=math.ceil((lo-phase)/(2*math.pi));last=math.floor((hi-phase)/(2*math.pi))
    if first<=last:values.append(math.hypot(a,b)*(1 if maximum else -1))
    return (max if maximum else min)(values)


def support(footprint,padding,lo,hi):
    return [min(extremum(x,-y,lo,hi,False) for x,y in footprint)-padding,
            max(extremum(x,-y,lo,hi,True) for x,y in footprint)+padding,
            min(extremum(y,x,lo,hi,False) for x,y in footprint)-padding,
            max(extremum(y,x,lo,hi,True) for x,y in footprint)+padding]


def sensitivity(yaw,vx,vy,wz,duration,footprint,padding):
    angle=wz*duration
    sinc=1 if angle==0 else math.sin(angle)/angle
    cosc=0 if angle==0 else 2*math.sin(angle/2)**2/angle
    # Norm is invariant under the initial world rotation.
    dx=duration*((sinc-1)*vx-cosc*vy);dy=duration*(cosc*vx+(sinc-1)*vy)
    base=support(footprint,padding,yaw,yaw)
    swept=support(footprint,padding,min(yaw,yaw+angle),max(yaw,yaw+angle))
    expansion=max(0,base[0]-swept[0],swept[1]-base[1],base[2]-swept[2],swept[3]-base[3])
    return abs(angle),math.hypot(dx,dy),expansion


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path);p.add_argument('--evidence',type=pathlib.Path);args=p.parse_args()
    root=pathlib.Path(__file__).resolve().parents[2];out=args.output.resolve()
    target=args.evidence.resolve() if args.evidence else pathlib.Path(__file__).parent/'evidence'
    if out.exists():raise FileExistsError('preserve existing audit output; choose a new directory')
    if target.exists():raise FileExistsError('preserve existing evidence; pass a new --evidence directory')
    out.mkdir(parents=True)
    profile=root/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml';text=profile.read_text()
    footprints=re.findall(r'footprint:\s*"(\[[^\n]+\])"',text)
    footprint=json.loads(footprints[0]);assert all(json.loads(s)==footprint for s in footprints)
    paddings=re.findall(r'footprint_padding:\s*([0-9.]+)',text);padding=float(paddings[0]);assert len(set(paddings))==1
    accel=json.loads(re.findall(r'max_accel:\s*(\[[^\n]+\])',text)[0]);decel=json.loads(re.findall(r'max_decel:\s*(\[[^\n]+\])',text)[0])
    angular_first_step=.05*min(accel[2],-decel[2])
    assert sensitivity(0,.4,.1,0,1.5,footprint,padding)==(0,0,0)
    # Independent endpoint example: quarter-turn constant forward body velocity.
    angle,bias,exp=sensitivity(0,1,0,1,math.pi/2,footprint,padding)
    assert abs(bias-math.hypot(1-math.pi/2,1))<1e-12
    all_rows=[];results={};inputs={}
    evidence=root/'experiments/r4_corrected_runtime_shadow/evidence'
    for scene in ['S0','S1','S2']:
        cp=evidence/(scene+'_cycles.csv');ep=evidence/(scene+'_events.json')
        inputs[str(cp.relative_to(root))]=sha(cp);inputs[str(ep.relative_to(root))]=sha(ep)
        events={v['kind']:v for v in json.loads(ep.read_text())['events']}
        start=events['goal_accepted']['ROS_ns'];end=events['observation_end']['ROS_ns'];goal=events['native_goal_result']['ROS_ns']
        with cp.open() as f:rows=[r for r in csv.DictReader(f) if start<=int(r['acquire_ros_ns'])<=end]
        assert len(rows)==400
        for r in rows:
            v=dict(scene=scene,cycle=int(r['cycle']),acquire_ros_ns=int(r['acquire_ros_ns']),native_navigation=int(r['acquire_ros_ns'])<=goal,
                   dynamic_event=scene!='S0' and start+10**9<=int(r['acquire_ros_ns'])<events['clear_target']['ROS_ns'],
                   absolute_measured_wz=abs(float(r['measured_wz'])))
            for seconds,label in [(.05,'50ms'),(1.5,'1500ms')]:
                angle,bias,exp=sensitivity(float(r['yaw']),float(r['measured_vx']),float(r['measured_vy']),float(r['measured_wz']),seconds,footprint,padding)
                v['held_'+label+'_angle_rad']=angle;v['fixed_yaw_'+label+'_endpoint_bias_m']=bias;v['held_'+label+'_support_face_expansion_m']=exp
            all_rows.append(v)
        scene_rows=[r for r in all_rows if r['scene']==scene];windows={}
        for label,chosen in [('goal',scene_rows),('native_navigation',[r for r in scene_rows if r['native_navigation']]),('dynamic_event',[r for r in scene_rows if r['dynamic_event']])]:
            windows[label]=dict(cycles=len(chosen),metrics={k:stats(r[k] for r in chosen) for k in chosen[0] if k not in ['scene','cycle','acquire_ros_ns','native_navigation','dynamic_event']} if chosen else {},
                               measured_wz_exceeds_first_step_from_zero=sum(r['absolute_measured_wz']>angular_first_step for r in chosen))
        same_geometry=[(a,b) for a,b in zip(rows,rows[1:]) if all(a[k]==b[k] for k in ['path_digest','map_digest','limits_digest'])]
        windows['identity_diagnostics']=dict(same_path_map_limits_adjacent_pairs=len(same_geometry),body_digest_changed=sum(a['body_digest']!=b['body_digest'] for a,b in same_geometry),
                                            exact_yaw_changed=sum(a['yaw']!=b['yaw'] for a,b in same_geometry),corridor_rebuilt_cycles=sum(r['corridor_rebuilt']=='1' for r in rows))
        results[scene]=windows
    with (out/'sensitivity.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(all_rows[0]),lineterminator='\n');w.writeheader();w.writerows(all_rows)
    summary=dict(stage='A17 recorded-input rotation scope sensitivity',profile=dict(footprint=footprint,padding=padding,angular_first_step_from_zero=angular_first_step),scenes=results,
                 meaning='Conditional constant measured body-twist geometry at 50ms/1.5s. Not actual future, obstacle prediction, Follow result, physical safety or runtime behavioral PASS.',
                 runtime_calls=dict(Follow=0,solver=0,prediction=0,ROS=0,current_admission=0))
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    sources=['src/rm_r4_prediction_consumption/include/rm_r4_prediction_consumption/follow.hpp','src/rm_r4_prediction_consumption/include/rm_r4_prediction_consumption/consumption.hpp',
             'src/rm_r4_prediction_consumption/src/follow.cpp','src/rm_r4_prediction_consumption/src/consumption.cpp','src/rm_r4_prediction_consumption/src/corridor.cpp','src/rm_r4_prediction_consumption/CMakeLists.txt',
             'src/rm_navigation_execution_adapters/include/rm_navigation_execution_adapters/current_geometry.hpp','src/rm_navigation_execution_adapters/src/current_geometry.cpp',
             'src/rm_navigation_execution_adapters/include/rm_navigation_execution_adapters/lease_fence.hpp','src/rm_navigation_execution_adapters/src/lease_fence.cpp','src/rm_navigation_execution_adapters/CMakeLists.txt',
             'src/rm_r4_nav2_controller/src/controller.cpp','src/rm_r4_nav2_controller/src/proposal_bridge.cpp',
             'experiments/tdt_planner/rm_tdt_planner/vendor/tdt_nav/MinimumSnapOsqp/sfcSquare.cpp','experiments/tdt_planner/rm_tdt_planner/include/rm_tdt_planner/pose_geometry.hpp',
             'experiments/tdt_planner/rm_tdt_planner/src/pose_geometry.cpp','experiments/r4_runtime_shadow/shadow.cpp','experiments/r4_runtime_shadow/shadow_seed.hpp',
             str(profile.relative_to(root)),'src/rm_competition_interfaces/msg/DynamicObstaclePredictionArray.msg','src/rm_competition_interfaces/msg/DynamicObstaclePrediction.msg']
    provenance=dict(baseline_HEAD=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),script_sha256=sha(pathlib.Path(__file__)),inputs=inputs,
                    audited_sources={name:sha(root/name) for name in sources},outputs={name:sha(out/name) for name in ['summary.json','sensitivity.csv']},
                    formula_checks='zero rotation and independent quarter-turn endpoint identities PASS; analytical extrema include endpoints and interior stationary angles; not runtime validation')
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    target.mkdir(parents=True)
    for name in ['summary.json','sensitivity.csv','provenance.json']:shutil.copyfile(out/name,target/name)
    print(json.dumps(dict(scenes={s:results[s]['native_navigation']['cycles'] for s in results},input_records=len(all_rows),output_directory=str(out))))


if __name__=='__main__':main()
