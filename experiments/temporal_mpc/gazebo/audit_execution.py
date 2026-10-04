#!/usr/bin/env python3
"""Read-only exact-input model replay and independent native rejection checks.

No physical oracle data is consumed here. Callback input identities come from
health, rather than inferred DDS receipt order. Timing remains a recorded metric,
not a replay-machine budget result or a continuous safety certificate.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from audit_run import namespace, epoch, stats
from temporal_mpc.contracts import PublicAdapter, predict, ContractError
from temporal_mpc.execution_guard import ExecutionGuard, StaticCells
from temporal_mpc.geometry import clearance
from temporal_mpc.dynamics import rollout_zoh


def native_states(d, proposal):
    x=np.array(d['initial_state'],float);x[5]=0.;result=[x.copy()]
    age=(d['evaluation_ns']-d['proposal_ns'])*1e-9
    a=np.array([[v['x'],v['y'],v['z']] for v in proposal['accelerations']])
    for k in range(30):
        if k>=27: control=np.clip(-x[3:]/.05,[-1.,-1.,0.],[1.,1.,0.])
        else:
            begin=age+k*.05;end=begin+.05;left=np.arange(30)*.05
            overlap=np.maximum(0.,np.minimum(end,left+.05)-np.maximum(begin,left))
            control=overlap@a/.05
        remaining=(29-k)*.05
        target=np.clip(x[3:5]+.05*control[:2],-remaining,remaining)
        if d.get('reanchor_projection') == 'velocity_and_stop/v2':
            target=np.clip(target,[-.5,-.5],[.8,.5])
        control[:2]=np.clip((target-x[3:5])/.05,-1.,1.)
        x=rollout_zoh(x,control[None,:],.05)[1];result.append(x.copy())
    return np.array(result)


def evaluate(root):
    events=[json.loads(s) for s in (root/'events.jsonl').open()]
    maps={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/map'}
    cells={}
    for ns,m in maps.items():
        i=m['info'];grid=np.array(m['data']).reshape(i['height'],i['width'])
        costs=np.where((grid<0)|(grid>=65),254,0).astype(np.uint8)
        cells[ns]=StaticCells(costs,i['resolution'],(i['origin']['position']['x'],i['origin']['position']['y']))
    predictions={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/dynamic_obstacle_predictions'}
    proposals={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/temporal_mpc/proposal'}
    errors=[];missing=[];guard_count=0;native_count=0;clearance_count=0;slew=[];statuses=Counter();constraints=Counter()
    elapsed=[];tick_elapsed=[];guards=[];executed_rejections=[];native_model_errors=[]

    def snapshot(ns,evaluation):
        if ns is None:return None
        return PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(predictions[ns]),evaluation)

    for e in events:
        if e['topic']=='/temporal_mpc/execution_health':
            d=json.loads(e['data']['data']);guards.append((e,d));statuses[d['status']]+=1
            elapsed.append(d['elapsed_s']);tick_elapsed.append(d.get('tick_to_output_s',d['elapsed_s']))
            if 'initial' not in d:missing.append('guard inputs absent');continue
            previous=np.array(d['previous']);command=np.array(d['command']);slew.append(float(np.max(np.abs(command-previous))))
            if not np.isfinite(command).all() or slew[-1]>.05000000001:errors.append('nonfinite/unbounded guard output')
            if d['reason']=='guard deadline' or d['reason'].endswith(': guard boundary'):
                expected=np.sign(previous)*np.maximum(np.abs(previous)-[.05,.05,0.],0.)
                if d['model_certified'] or not np.allclose(expected,command,atol=1e-12):errors.append('invalid boundary brake')
                continue
            try:
                snap=snapshot(d['source_ns'],d['epoch_ns'])
            except ContractError:snap=None
            except KeyError:missing.append('guard source missing');continue
            g=ExecutionGuard();g.previous=previous
            # Remove replay-host scheduling; recorded budget fields stay above.
            with patch('temporal_mpc.execution_guard.time.perf_counter',return_value=0.):
                result=g.step(np.array(d['initial'],float),np.array(d['requested'],float),d['epoch_ns'],
                              snap,cells.get(d['map_revision']),d['state_age_s'] if d['state_age_s'] is not None else float('inf'))
            guard_count+=1
            if (result.status!=d['status'] or result.reason!=d['reason'] or result.model_certified!=d['model_certified']
                    or not np.allclose(result.command,command,atol=1e-12,rtol=0.)
                    or (result.constraint_min is None)!=(d['constraint_min'] is None)
                    or (result.constraint_min is not None and abs(result.constraint_min-d['constraint_min'])>1e-9)):
                errors.append('guard exact-input replay mismatch at tick '+str(d['tick_count']))
        if e['topic']=='/temporal_mpc/health':
            d=json.loads(e['data']['data']);constraints[d.get('constraint','unrecorded')]+=1
            if d['executed'] and not d['ready']:executed_rejections.append(d)
            if d.get('step',-1)<0 or d.get('constraint') not in ('accepted','dynamic_clearance','corridor_x_lower','corridor_x_upper','corridor_y_lower','corridor_y_upper','velocity_x_lower','velocity_x_upper','velocity_y'):continue
            try:p=proposals[d['proposal_ns']]
            except KeyError:missing.append('native proposal missing');continue
            points=native_states(d,p);checked=np.array(d['checked_state']);native_count+=1
            error=float(np.max(np.abs(points[d['step']]-checked)));native_model_errors.append(error)
            if error>1e-8:errors.append('native held-model state mismatch')
            if d['constraint']=='dynamic_clearance':
                try:
                    timeline=predict(snapshot(d['prediction_ns'],d['evaluation_ns']),d['evaluation_ns'],np.arange(31)*.05)
                    i=timeline.track_ids.index(d['track_id'])
                    value=float(clearance(checked[None,:],timeline.centers[i][d['step']][None,:],timeline.geometries[i],(.355,.330))[0]
                                -.02-.025*(np.hypot(.8,.5)+timeline.speeds[i]))
                    clearance_count+=1
                    if abs(value-d['slack'])>1e-8 or value>=0.:errors.append('native clearance rejection mismatch')
                except (KeyError,ValueError,ContractError):missing.append('native dynamic input missing')
    ticks=[d['tick_count'] for e,d in guards if 'tick_count' in d]
    return dict(guard_exact_input_replays=guard_count,native_zoh_state_replays=native_count,
                native_model_error=stats(native_model_errors),native_dynamic_rejection_checks=clearance_count,
                native_constraint_counts=dict(constraints),executed_rejections=executed_rejections,
                guard_statuses=dict(statuses),guard_slew_max=max(slew) if slew else None,
                guard_core_s=stats(elapsed),guard_tick_to_output_s=stats(tick_elapsed),
                guard_producer_start_interval_s=stats([d['producer_start_interval_s'] for e,d in guards if d.get('producer_start_interval_s') is not None]),
                guard_producer_output_interval_s=stats([d['producer_output_interval_s'] for e,d in guards if d.get('producer_output_interval_s') is not None]),
                guard_tick_to_output_cpu_s=stats([d['tick_to_output_cpu_s'] for e,d in guards if 'tick_to_output_cpu_s' in d]),
                guard_nominal_missed_slots=sum(d.get('nominal_missed_slots',0) for e,d in guards),
                guard_record_tick_gaps=[b-a for a,b in zip(ticks,ticks[1:]) if b-a!=1],
                guard_observed_10ms_gate=bool(tick_elapsed and max(tick_elapsed)<=.010),
                errors=errors,missing_inputs=Counter(missing),all_model_replays_pass=bool(guard_count and native_count and not errors and not missing),
                scope='Model replay and recorded timing only; no continuous actuator or detection safety certificate; no oracle inputs')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args();root=Path(args.directory)
    result=evaluate(root);(root/'execution_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='executed_rejections'},indent=2))
