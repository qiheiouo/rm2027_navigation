#!/usr/bin/env python3
"""Recorded worker/native input joins and 2x2 dynamic-margin interventions.

All interventions use the SAME native evaluation/proposal time. Substitution
of the older measured state is a model counterfactual, not a propagated plant
prediction or executed trajectory. No oracle or future actor command is read.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from audit_run import namespace, epoch, stats
from audit_execution import native_states
from temporal_mpc.contracts import PublicAdapter, predict, ContractError
from temporal_mpc.dynamics import rollout_zoh


def dynamic_margins(states,snapshot,evaluation):
    timeline=predict(snapshot,evaluation,np.arange(31)*.05)
    values=[]
    ux=np.c_[np.cos(states[:,2]),np.sin(states[:,2])]
    uy=np.c_[-np.sin(states[:,2]),np.cos(states[:,2])]
    for centers,g,speed in zip(timeline.centers,timeline.geometries,timeline.speeds):
        if g.kind!='circle':raise ContractError('native circle diagnostic domain')
        rel=centers-states[:,:2]
        local=np.c_[np.sum(rel*ux,axis=1),np.sum(rel*uy,axis=1)]
        distance=np.linalg.norm(np.maximum(np.abs(local)-[.355,.330],0.),axis=1)
        values.append(distance-g.radius-.02-.025*(np.hypot(.8,.5)+speed))
    return np.array(values),timeline.track_ids


def factorial(health,proposal,worker,old,new):
    if (worker.get('input_validated') is not True or worker['epoch_ns']!=health['proposal_ns']
            or worker['generation']!=health['generation'] or worker['input_source_ns']!=old.source_ns):
        raise ValueError('worker/proposal/native input identity mismatch')
    initial=np.array(worker['initial_state'],float)
    if initial.shape!=(6,) or not np.isfinite(initial).all():raise ValueError('worker initial state')
    cases={}
    for state_name,state in (('worker',initial),('native',health['initial_state'])):
        points=native_states(dict(health,initial_state=list(state)),proposal)
        for source_name,snapshot in (('worker',old),('native',new)):
            name=state_name+'_state__'+source_name+'_prediction'
            try:
                margins,ids=dynamic_margins(points,snapshot,health['evaluation_ns'])
                record=None
                if health['track_id'] in ids:record=float(margins[ids.index(health['track_id']),health['step']])
                where=np.unravel_index(np.argmin(margins),margins.shape) if margins.size else None
                cases[name]=dict(at_recorded_track_step_m=record,
                    hypothetical_full_min_m=None if where is None else float(margins[where]),
                    hypothetical_full_min_step=None if where is None else int(where[1]),
                    hypothetical_full_min_track=None if where is None else ids[where[0]])
            except ContractError as error:cases[name]=dict(unavailable=str(error))
    original=rollout_zoh(initial.copy()*[1,1,1,1,1,0],
                np.array([[a['x'],a['y'],a['z']] for a in proposal['accelerations']]),.05)
    values,_=dynamic_margins(original,old,worker['epoch_ns'])
    return dict(evaluation_ns=health['evaluation_ns'],proposal_ns=health['proposal_ns'],
                worker_source_ns=old.source_ns,native_source_ns=new.source_ns,
                measured_state_delta=(np.array(health['initial_state'])-initial).tolist(),
                recorded_native_constraint=health['constraint'],recorded_step=health['step'],
                recorded_track_id=health['track_id'],recorded_slack_m=health['slack'],cases=cases,
                worker_original_dynamic_min_m=None if not values.size else float(values.min()),
                worker_constraint_min=worker.get('constraint_min'))


def evaluate(root):
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    predictions={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/dynamic_obstacle_predictions'}
    proposals={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/temporal_mpc/proposal'}
    diagnostics=[json.loads(e['data']['data']) for e in events if e['topic']=='/temporal_mpc/solver_diagnostic']
    workers={(d['epoch_ns'],d.get('generation')):d for d in diagnostics if d.get('schema')=='temporal_mpc_solver_diagnostic/v2_inputs'}
    cases=[];shadow_cases=[];joins=0;missing=Counter();errors=[];all_checks=0;native_checks=[]
    for e in events:
        if e['topic']!='/temporal_mpc/health':continue
        h=json.loads(e['data']['data'])
        if h['step']<0 or h['constraint'] not in ('accepted','dynamic_clearance'):continue
        all_checks+=1
        try:
            w=workers[(h['proposal_ns'],h['generation'])];p=proposals[h['proposal_ns']]
            if w['map_revision']!=p['map_revision'] or w['input_validated'] is not True:raise KeyError('worker map/input')
            joins+=1
            if h['constraint']!='dynamic_clearance':continue
            old=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(predictions[w['input_source_ns']]),w['epoch_ns'])
            new=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(predictions[h['prediction_ns']]),h['evaluation_ns'])
            value=factorial(h,p,w,old,new);value['executed']=h['executed']
            (cases if h['executed'] else shadow_cases).append(value)
            recorded=value['cases']['native_state__native_prediction']['at_recorded_track_step_m']
            native_checks.append(recorded)
            if recorded is None or abs(recorded-h['slack'])>1e-8:errors.append('actual native margin mismatch')
        except (KeyError,ValueError,ContractError) as error:missing[str(error)]+=1
    traces=[d for d in diagnostics if d.get('strategy')=='portfolio']
    feasible=[d for d in traces if d['feasible']]
    choices=Counter(d.get('chosen_candidate') or 'none' for d in traces)
    optimized_choices=Counter(d.get('chosen_candidate') or 'none' for d in feasible if not d['fallback'])
    attempted=Counter(t['candidate'] for d in traces for t in d.get('candidates',[]) if t['attempted'])
    return dict(checked_native_records=all_checks,worker_input_joins=joins,missing=dict(missing),errors=errors,
                executed_dynamic_factorials=cases,shadow_dynamic_factorials=shadow_cases,
                native_dynamic_margins_checked=len(native_checks),actual_native_margins_match=not errors,
                worker_solver_s=stats([d['solver_s'] for d in diagnostics if 'solver_s' in d]),
                worker_iterations_max=max([d['iterations'] for d in diagnostics],default=0),
                portfolio_choices=dict(choices),portfolio_optimized_choices=dict(optimized_choices),portfolio_attempts=dict(attempted),
                portfolio_max_attempts=max([sum(t['attempted'] for t in d.get('candidates',[])) for d in traces],default=0),
                portfolio_diagnostic_count=len(traces),portfolio_feasible_count=len(feasible),
                accepted_portfolio_budget_gate=all(d['elapsed_s']<=.04 and d['solver_s']<=.015 and d['iterations']<=390
                                                  for d in feasible) if feasible else None,
                scope='Exact recorded input joins. Four dynamic-margin counterfactuals at one native epoch; no backdating stale predictions, no execution/physical certificate, no unique additive causal attribution.',
                dynamic_acceptance=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    if (root/'reanchor_inputs_audit.json').exists():raise SystemExit('refuse audit overwrite')
    result=evaluate(root);(root/'reanchor_inputs_audit.json').write_text(json.dumps(result,allow_nan=False,indent=2)+'\n')
    print(json.dumps(result,indent=2))
