#!/usr/bin/env python3
"""Relative-time/shift/terminal projection model counterfactuals, not execution."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from audit_run import epoch, namespace
from audit_execution import native_states
from audit_reanchor_inputs import dynamic_margins
from temporal_mpc.contracts import PublicAdapter, ContractError
from temporal_mpc.dynamics import rollout_zoh


def cases(health,proposal,worker,old,new):
    if (worker.get('input_validated') is not True or worker['epoch_ns']!=health['proposal_ns']
            or worker['generation']!=health['generation'] or worker['input_source_ns']!=old.source_ns):
        raise ValueError('age audit input identity')
    x=np.array(worker['initial_state'],float);x[5]=0.
    controls=np.array([[a['x'],a['y'],a['z']] for a in proposal['accelerations']])
    original=rollout_zoh(x,controls,.05)
    proposal_ns=health['proposal_ns'];evaluation=health['evaluation_ns'];age=(evaluation-proposal_ns)*1e-9
    if not 0<=age<=.15:raise ValueError('native proposal age domain')
    left=np.arange(30)*.05;begin=left+age
    overlaps=np.maximum(0.,np.minimum(begin[:,None]+.05,left[None,:]+.05)-np.maximum(begin[:,None],left[None,:]))
    shifted=rollout_zoh(x,overlaps@controls/.05,.05)
    unshifted=native_states(dict(health,initial_state=x.tolist(),proposal_ns=evaluation),proposal)
    projected=native_states(dict(health,initial_state=x.tolist()),proposal)
    groups=[('worker_epoch_original',original,proposal_ns,old),
            ('worker_epoch_native_projection',unshifted,proposal_ns,old),
            ('evaluation_original_relative_controls',original,evaluation,old),
            ('evaluation_unshifted_native_projection',unshifted,evaluation,old),
            ('evaluation_shifted_without_terminal_projection',shifted,evaluation,old),
            ('evaluation_native_projection_old_inputs',projected,evaluation,old),
            ('actual_native_new_inputs',native_states(health,proposal),evaluation,new)]
    output={}
    for name,points,now,snapshot in groups:
        try:
            margins,ids=dynamic_margins(points,snapshot,now)
            at=None if health['track_id'] not in ids else float(margins[ids.index(health['track_id']),health['step']])
            output[name]=dict(recorded_track_step_slack_m=at,full_dynamic_min_m=None if not margins.size else float(margins.min()),
                terminal_speed_max=float(np.max(np.abs(points[-1,3:]))),
                velocity_bounds_pass=bool(np.all(points[:,3]>=-.5-1e-6) and np.all(points[:,3]<=.8+1e-6) and np.max(np.abs(points[:,4]))<=.5+1e-6))
        except ContractError as error:output[name]=dict(unavailable=str(error))
    actual=output['actual_native_new_inputs'].get('recorded_track_step_slack_m')
    return dict(evaluation_ns=evaluation,proposal_ns=proposal_ns,age_ns=evaluation-proposal_ns,
                worker_source_ns=old.source_ns,native_source_ns=new.source_ns,
                executed=health['executed'],step=health['step'],track_id=health['track_id'],recorded_slack_m=health['slack'],
                actual_replay_match=actual is not None and abs(actual-health['slack'])<1e-8,cases=output)


def evaluate(root):
    path=root/'events.jsonl';compressed=not path.exists();path=path.with_suffix('.jsonl.gz') if compressed else path
    with (gzip.open(path,'rt') if compressed else path.open()) as stream:events=[json.loads(line) for line in stream]
    predictions={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/dynamic_obstacle_predictions'}
    proposals={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/temporal_mpc/proposal'}
    workers={}
    for e in events:
        if e['topic']=='/temporal_mpc/solver_diagnostic':
            d=json.loads(e['data']['data']);workers[(d['epoch_ns'],d['generation'])]=d
    records=[];errors=[]
    for e in events:
        if e['topic']!='/temporal_mpc/health':continue
        h=json.loads(e['data']['data'])
        if h['constraint']!='dynamic_clearance' or h['step']<0:continue
        try:
            w=workers[(h['proposal_ns'],h['generation'])];p=proposals[h['proposal_ns']]
            old=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(predictions[w['input_source_ns']]),w['epoch_ns'])
            new=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(predictions[h['prediction_ns']]),h['evaluation_ns'])
            records.append(cases(h,p,w,old,new))
        except (KeyError,ValueError,ContractError) as error:errors.append(str(error))
    return dict(records=records,errors=errors,all_actual_replays_match=bool(records) and all(r['actual_replay_match'] for r in records),
                scope='Recorded model counterfactuals. Same old worker state, no plant propagation; old/new epochs explicitly differ. Unprojected shifts may violate stop/bounds and are never proposals. Relative grid stays 1.5s, stale inputs unavailable; no additive causal or physical certificate.',dynamic_acceptance=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');a=p.parse_args();target=Path(a.output)
    if target.exists():raise SystemExit('refuse age audit overwrite')
    target.parent.mkdir(parents=True,exist_ok=True);d=evaluate(Path(a.input));target.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(records=len(d['records']),errors=d['errors'],all_actual_replays_match=d['all_actual_replays_match']),indent=2))
