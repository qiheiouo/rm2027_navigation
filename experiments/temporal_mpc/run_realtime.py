#!/usr/bin/env python3
"""Reproducible M1 T-DT frontend + bounded temporal QP SHADOW experiment."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time
import numpy as np
import scipy
import osqp
from temporal_mpc.fixtures import SCENARIOS, Scenario
from temporal_mpc.frontend import prepare_route
from temporal_mpc.contracts import Snapshot
from temporal_mpc.realtime_qp import RealtimeMPC, QPConfig
from temporal_mpc.oracle import step_and_audit


def setup(s, frontend, static_detour=False):
    r=.05;d=s.domain
    grid=np.zeros((round((d[3]-d[2])/r),round((d[1]-d[0])/r)),np.uint8)
    if static_detour:
        grid[33:47,70:80]=254
    begin=time.perf_counter()
    route=prepare_route(frontend,grid,r,(d[0],d[2]),s.start,s.goal)
    duration=time.perf_counter()-begin
    return route,grid,duration


def simulate(s,mode,cfg,frontend,output,static_detour=False):
    name=s.name+'__'+mode
    route,grid,planning_s=setup(s,frontend,static_detour)
    (output/(name+'__route.json')).write_text(json.dumps({"tdt_output":route.document,
        "certified_merged_bounds":route.bounds.tolist(),"plan_id":route.plan_id,
        "grid_sha256":hashlib.sha256(grid.tobytes()).hexdigest(),"planning_s":planning_s},indent=2)+'\n')
    controller=RealtimeMPC(cfg);x=np.array(s.start);trace=[]
    timing=[];cpu=[];statuses=Counter();reasons=Counter();minimum=10.;contact=False;goal_time=None
    controls=[];commands=[];states=[x.tolist()];waiting=distance=0.;feasible=fallbacks=0
    # Physical map includes the known occupied border-cell strip.
    domain=(s.domain[0]+.05,s.domain[1]-.05,s.domain[2]+.05,s.domain[3]-.05)
    truth=(lambda t:((2.75,0.),(.25,.35))) if static_detour else s.truth
    with (output/(name+'.jsonl')).open('x') as f:
        for k in range(round(s.duration/cfg.period)):
            t=k*cfg.period;epoch=round(t*1e9)
            snap=Snapshot(epoch,()) if static_detour else s.observe(t,mode)
            started,cpu_started=time.perf_counter(),time.process_time()
            w=route.window(x,epoch,controller.times)
            result=controller.solve(x,epoch,snap,w)
            total=time.perf_counter()-started;cpu_time=time.process_time()-cpu_started
            nx,lo,sampled,hit=step_and_audit(x,result.acceleration,t,cfg.period,truth,domain,0. if static_detour else s.speed_bound)
            record={"cycle":k,"epoch_ns":epoch,"source_ns":snap.source_ns,"initial":x.tolist(),
                "command":result.command.tolist(),"acceleration":result.acceleration.tolist(),
                "centre_bounds":w.centre_bounds,"reference":w.reference.tolist(),"plan_id":w.plan_id,
                "status":result.status,"reason":result.reason,"solver_status":result.solver_status,
                "iterations":result.iterations,"selected_track_ids":result.selected_ids,
                "model_feasible":result.model_feasible,"constraint_min":result.constraint_min,
                "fallback_requested":result.fallback_id,"handoff_executed":False,
                "solve_elapsed_s":result.elapsed_s,"control_elapsed_s":total,"cpu_s":cpu_time,
                "physical_clearance_lower_m":lo,"physical_clearance_sampled_m":sampled,"physical_contact":hit}
            f.write(json.dumps(record,allow_nan=False)+'\n');trace.append(record)
            controls.append(result.acceleration);commands.append(result.command);timing.append(total);cpu.append(cpu_time)
            statuses[result.status]+=1;reasons[result.reason]+=1;feasible+=int(result.model_feasible);fallbacks+=int(result.fallback_id is not None)
            distance+=float(np.linalg.norm(np.asarray(nx[:2])-x[:2]));minimum=min(minimum,lo);contact|=hit
            waiting+=cfg.period if np.linalg.norm(nx[3:5])<.05 else 0.
            x=np.array(nx);states.append(x.tolist())
            if np.linalg.norm(x[:2]-s.goal[:2])<=.12 and np.max(np.abs(x[3:]))<=.05:
                goal_time=(k+1)*cfg.period;break
            if contact:break
    acceleration=np.asarray(controls);velocity=np.asarray(states)[:,3:];command=np.asarray(commands)
    report={"scenario":s.name,"geometry_mode":mode,"cycles":len(trace),"model_goal_reached":goal_time is not None,
        "ros_action_success":None,"physical_collision":contact,"minimum_physical_clearance_lower_m":minimum,
        "completion_time_s":goal_time,"observed_duration_s":len(trace)*cfg.period,"stop_wait_duration_s":waiting,
        "travel_distance_m":distance,"average_speed_m_s":distance/(len(trace)*cfg.period),
        "detour_distance_m":max(0.,distance-np.linalg.norm(x[:2]-np.array(s.start[:2]))),
        "control_p50_s":float(np.percentile(timing,50)),"control_p95_s":float(np.percentile(timing,95)),"control_max_s":max(timing),
        "cycle_budget_miss_count":sum(t>cfg.cycle_budget for t in timing),"20hz_deadline_miss_count":sum(t>cfg.period for t in timing),
        "solver_failure_cycles":sum(r['status'] not in ('optimized','feasible_iterate') for r in trace),
        "fallback_request_cycles":fallbacks,"fallback_handoffs_executed":0,"model_feasible_cycles":feasible,
        "max_iterations":max(r['iterations'] for r in trace),"statuses":dict(statuses),"reasons":dict(reasons),
        "cpu_process_s":sum(cpu),"max_abs_velocity_xyz":np.abs(velocity).max(axis=0).tolist(),
        "rms_velocity_xyz":np.sqrt(np.mean(velocity**2,axis=0)).tolist(),
        "max_abs_acceleration_xyz":np.abs(acceleration).max(axis=0).tolist(),
        "max_abs_command_delta_xyz":np.abs(np.diff(command,axis=0)).max(axis=0).tolist() if len(command)>1 else [0.,0.,0.],
        "max_abs_jerk_xyz":(np.abs(np.diff(acceleration,axis=0)).max(axis=0)/cfg.period).tolist() if len(trace)>1 else [0.,0.,0.],
        "recovery_count":None,"false_block_count":None,"false_block_duration_s":None,
        "offline_physical_task_gate":bool(goal_time is not None and not contact and minimum>=.05),
        "offline_physical_and_20hz_gate":bool(goal_time is not None and not contact and minimum>=.05 and max(timing)<=cfg.period),
        "deployment_accepted":False}
    (output/(name+'.json')).write_text(json.dumps({"scenario":asdict(s),"config":asdict(cfg),
        "physical_domain":domain,"static_detour":static_detour,"report":report},indent=2)+'\n')
    return report


def verify(output):
    count=0
    for p in sorted(output.glob('*.jsonl')):
        doc=json.loads(p.with_suffix('.json').read_text());s=Scenario(**doc['scenario']);cfg=QPConfig(**{
            k:tuple(v) if isinstance(v,list) else v for k,v in doc['config'].items()})
        static=doc['static_detour'];truth=(lambda t:((2.75,0.),(.25,.35))) if static else s.truth
        x=np.array(s.start);minimum=10.;contact=False;goal_time=None;records=[]
        for k,line in enumerate(p.read_text().splitlines()):
            r=json.loads(line);records.append(r)
            if r['cycle']!=k or r['epoch_ns']!=round(k*cfg.period*1e9):raise ValueError('trace epoch mismatch')
            np.testing.assert_allclose(r['initial'],x,rtol=0,atol=1e-9)
            np.testing.assert_allclose(r['command'],x[3:]+cfg.period*np.array(r['acceleration']),rtol=0,atol=1e-9)
            if np.any(np.abs(r['acceleration'])>np.array([1.,1.,2.])+1e-6):raise ValueError('acceleration outside bounds')
            x,lo,sampled,hit=step_and_audit(x,r['acceleration'],k*cfg.period,cfg.period,truth,doc['physical_domain'],0. if static else s.speed_bound)
            np.testing.assert_allclose([r['physical_clearance_lower_m'],r['physical_clearance_sampled_m']],[lo,sampled],rtol=0,atol=1e-9)
            if r['physical_contact']!=hit:raise ValueError('physical contact mismatch')
            x=np.array(x);minimum=min(minimum,lo);contact|=hit;count+=1
            if np.linalg.norm(x[:2]-s.goal[:2])<=.12 and np.max(np.abs(x[3:]))<=.05:goal_time=(k+1)*cfg.period
        report=doc['report']
        if report['cycles']!=len(records) or report['physical_collision']!=contact or report['completion_time_s']!=goal_time:
            raise ValueError('report count/contact/goal mismatch')
        np.testing.assert_allclose(report['minimum_physical_clearance_lower_m'],minimum,rtol=0,atol=1e-9)
        if report['20hz_deadline_miss_count']!=sum(r['control_elapsed_s']>cfg.period for r in records):raise ValueError('deadline count mismatch')
        expected=bool(goal_time is not None and not contact and minimum>=.05)
        if report['offline_physical_task_gate']!=expected:raise ValueError('physical gate mismatch')
        if report['offline_physical_and_20hz_gate']!=bool(expected and report['20hz_deadline_miss_count']==0):raise ValueError('combined gate mismatch')
    summary=json.loads((output/'summary.json').read_text())
    if len(summary['reports'])!=len(list(output.glob('*.jsonl'))):raise ValueError('incomplete matrix')
    return {"status":"PASS","trials":len(summary['reports']),"cycles_replayed":count,
            "scope":"Commands replayed through independent scalar swept physical oracle; does not certify real-world plant or MPPI handoff."}


def main():
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--frontend',type=Path);a.add_argument('--verify-only',action='store_true')
    args=a.parse_args()
    if args.verify_only:
        print(json.dumps(verify(args.output),indent=2));return
    if args.frontend is None:a.error('--frontend is required')
    args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).parent;sources=[p for p in root.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.sh','.yaml','.xml')]
    manifest={"schema":"temporal_mpc_realtime_shadow/v1","date":"2026-10-04 Asia/Shanghai","config":asdict(QPConfig()),
        "python":sys.version,"numpy":np.__version__,"scipy":scipy.__version__,"osqp":osqp.__version__,"platform":platform.platform(),
        "thread_environment":{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')},
        "source_sha256":{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)},
        "frontend_binary_sha256":hashlib.sha256(args.frontend.read_bytes()).hexdigest(),
        "scope":"Offline fixed-yaw ideal plant. T-DT static path/corridor reuse. Control timing includes window+QP; physical simulation freezes during compute. MPPI requests logged, NOT executed.",
        "physical_safety_gate_m":.05,"deployment_accepted":False}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    reports=[]
    try:
        for s in SCENARIOS:
            for mode in ('observed_polygon','nominal_diameter'):
                report=simulate(s,mode,QPConfig(),args.frontend,args.output);reports.append(report)
                print(json.dumps({k:report[k] for k in ('scenario','geometry_mode','model_goal_reached','physical_collision','control_p95_s','offline_physical_and_20hz_gate')}),flush=True)
        static=Scenario('static_map_detour','static',domain=(-1.,7.,-2.,2.),duration=15.)
        reports.append(simulate(static,'static_map',QPConfig(),args.frontend,args.output,True))
    except BaseException as error:
        (args.output/'run_failure.json').write_text(json.dumps({'error':type(error).__name__,'reason':str(error),'completed':reports},indent=2)+'\n')
        raise
    (args.output/'summary.json').write_text(json.dumps({'reports':reports,'deployment_accepted':False},indent=2)+'\n')
    (args.output/'independent_replay.json').write_text(json.dumps(verify(args.output),indent=2)+'\n')


if __name__=='__main__':main()
