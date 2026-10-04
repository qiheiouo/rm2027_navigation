#!/usr/bin/env python3
"""Read-only current occupancy diagnostics; no oracle, search or control changes.

The fixed-yaw circle model is the native revalidator's model. Exclusion of the
entire goal disk at a recorded epoch follows from the 1-Lipschitz distance to a
translated rectangle. A free transverse section is only a current spatial
witness, not a temporal trajectory or physical safety certificate.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import yaml
from audit_run import namespace, epoch, stats
from temporal_mpc.contracts import PublicAdapter, predict, ContractError

HALF = (.355, .330)
RESERVE = .025 * math.hypot(.8, .5)


def circles(timeline):
    if any(g.kind != 'circle' for g in timeline.geometries):
        raise ValueError('diagnostic supports the native fixed-yaw circle model only')
    return [dict(track_id=i, center=[float(v) for v in c[0]], radius=g.radius,
                 speed=s, expanded_radius=g.radius+.02+RESERVE+.025*s)
            for i,c,g,s in zip(timeline.track_ids,timeline.centers,timeline.geometries,timeline.speeds)]


def point_slack(point, obstacle):
    cx,cy=obstacle['center'];x,y=point
    return math.hypot(max(abs(cx-x)-HALF[0],0.),max(abs(cy-y)-HALF[1],0.))-obstacle['expanded_radius']


def goal_occupancy(goal, tolerance, obstacles):
    if not obstacles:
        return dict(point_slack_m=None, goal_disk_upper_slack_m=None,
                    point_excluded=False, entire_goal_disk_excluded=False, witness_track=None)
    values=[(point_slack(goal,o),o['track_id']) for o in obstacles]
    value,track=min(values)
    upper=value+tolerance
    return dict(point_slack_m=value, goal_disk_upper_slack_m=upper,
                point_excluded=value<0., entire_goal_disk_excluded=upper<0.,witness_track=track)


def free_sections(x, bounds, obstacles):
    """Exact continuous y intervals for all current circles, native corridor reserve."""
    lo,hi=bounds[2]+RESERVE,bounds[3]-RESERVE
    if not bounds[0]+RESERVE <= x <= bounds[1]-RESERVE or lo>=hi:
        return dict(in_corridor=False,free_intervals=[],free_width_m=0.)
    blocked=[]
    for o in obstacles:
        cx,cy=o['center'];r=o['expanded_radius'];qx=max(abs(cx-x)-HALF[0],0.)
        # Tangency is feasible (zero slack); no strictly negative region then.
        if qx>=r:continue
        reach=HALF[1]+math.sqrt(r*r-qx*qx)
        a,b=max(lo,cy-reach),min(hi,cy+reach)
        if a<b:blocked.append((a,b))
    merged=[]
    for a,b in sorted(blocked):
        if merged and a<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],b)
        else:merged.append([a,b])
    free=[];start=lo
    for a,b in merged:
        if start<a:free.append([start,a])
        start=max(start,b)
    if start<hi:free.append([start,hi])
    return dict(in_corridor=True,free_intervals=free,
                free_width_m=sum(b-a for a,b in free),
                scope='Positive-width current sections; isolated zero-width tangencies omitted')


def preference_at_cut(initial, bounds, obstacles):
    ahead=[o for o in obstacles if 0. <= o['center'][0]-initial[0] < 6.]
    if not ahead:return None
    obstacle=min(ahead,key=lambda o:o['center'][0]-initial[0])
    cx,cy=obstacle['center']
    reach=obstacle['radius']+math.hypot(*HALF)+.02+RESERVE+.12
    levels=[cy+reach,cy-reach]
    fits=[bool(bounds[0]+RESERVE<=initial[0]<=bounds[1]-RESERVE
               and bounds[2]+RESERVE<=y<=bounds[3]-RESERVE) for y in levels]
    cut=free_sections(cx,bounds,obstacles)
    return dict(track_id=obstacle['track_id'],cut_x=cx,preference_levels_y=levels,
                preference_target_fits=fits,hard_current_section=cut,
                preference_excludes_both_but_current_section_exists=bool(not any(fits) and cut['free_width_m']>1e-8),
                scope='Horizontal route candidate availability only; does not reconstruct side latch or prove horizon feasibility')


def evaluate(root):
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    summary=json.loads((root/'run_summary.json').read_text());begin=int(round(summary['goal_epoch_s']*1e9))
    goal_message=next(e['data']['pose'] for e in events if e['topic']=='goal_sent')
    if goal_message['header']['frame_id']!='map':raise ValueError('goal frame')
    pose=goal_message['pose'];goal=[pose['position']['x'],pose['position']['y']]
    config=yaml.safe_load((root/'scene/nav2.yaml').read_text())['controller_server']['ros__parameters']
    tolerance=float(config['general_goal_checker']['xy_goal_tolerance'])
    if not math.isfinite(tolerance) or not 0.<tolerance<1.:raise ValueError('goal tolerance')
    raw_predictions={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/dynamic_obstacle_predictions'}
    proposals={epoch(e['data']['header']['stamp']):e['data'] for e in events if e['topic']=='/temporal_mpc/proposal'}
    adapter=PublicAdapter(geometry_mode='nominal_diameter')
    rows=[];rejected=Counter();modes=Counter();solver_reasons=Counter();native_constraints=Counter();missing=Counter();executed=[]
    for e in events:
        if e['topic']=='/dynamic_obstacle_predictions':
            try:snap=adapter.consume(namespace(e['data']),e['receipt_sim_ns'])
            except ContractError as err:rejected[str(err)]+=1;continue
            if snap.source_ns<begin:continue
            timeline=predict(snap,snap.source_ns,[0.,.05]);obs=circles(timeline)
            rows.append(dict(kind='public_source',source_ns=snap.source_ns,
                             receipt_ns=e['receipt_sim_ns'],obstacles=obs,
                             goal=goal_occupancy(goal,tolerance,obs)))
        if e['topic']=='/temporal_mpc/solver_diagnostic':
            d=json.loads(e['data']['data'])
            if d['epoch_ns']>=begin:
                modes[d.get('local_reference_mode','absent')]+=1;solver_reasons[d['reason']]+=1
        if e['topic']=='/temporal_mpc/health':
            d=json.loads(e['data']['data'])
            if d['evaluation_ns']<begin:continue
            native_constraints[d['constraint']]+=1
            if d['executed'] and not d['ready']:executed.append(d)
            # Input rejection health can retain the preceding validation's
            # step. A nonnegative step alone is not a geometry-check witness.
            checked_constraints=('accepted','dynamic_clearance','corridor_x_lower','corridor_x_upper',
                                 'corridor_y_lower','corridor_y_upper','velocity_x_lower','velocity_x_upper','velocity_y')
            if d['step']<0 or d['constraint'] not in checked_constraints:continue
            try:
                p=proposals[d['proposal_ns']]
                snap=PublicAdapter(geometry_mode='nominal_diameter').consume(namespace(raw_predictions[d['prediction_ns']]),d['evaluation_ns'])
                obs=circles(predict(snap,d['evaluation_ns'],[0.,.05]))
            except (KeyError,ContractError,ValueError) as err:missing[str(err)]+=1;continue
            initial=d['initial_state'];bounds=p['centre_bounds']
            if abs(initial[2])>1e-6 or abs(p['fixed_yaw'])>1e-6:
                missing['horizontal fixed-zero-yaw diagnostic domain']+=1;continue
            rows.append(dict(kind='native_evaluation',evaluation_ns=d['evaluation_ns'],proposal_ns=d['proposal_ns'],
                             prediction_ns=d['prediction_ns'],executed=d['executed'],ready=d['ready'],
                             initial_state=initial,centre_bounds=bounds,constraint=d['constraint'],
                             step=d['step'],track_id=d['track_id'],slack=d['slack'],obstacles=obs,
                             goal=goal_occupancy(goal,tolerance,obs),
                             current_section=free_sections(initial[0],bounds,obs),
                             preference=preference_at_cut(initial,bounds,obs)))
    public=[r for r in rows if r['kind']=='public_source'];native=[r for r in rows if r['kind']=='native_evaluation']
    def counts(items):
        return dict(samples=len(items),nonempty_samples=sum(bool(r['obstacles']) for r in items),
                    point_excluded=sum(r['goal']['point_excluded'] for r in items),
                    entire_goal_disk_excluded=sum(r['goal']['entire_goal_disk_excluded'] for r in items),
                    point_slack_m=stats([r['goal']['point_slack_m'] for r in items if r['obstacles']]),
                    goal_disk_upper_slack_m=stats([r['goal']['goal_disk_upper_slack_m'] for r in items if r['obstacles']]))
    result=dict(goal=goal,goal_tolerance_m=tolerance,current_public_source=counts(public),
                native_exact_epochs=counts(native),public_receipt_rejections=dict(rejected),
                diagnostic_missing_inputs=dict(missing),native_constraint_counts=dict(native_constraints),
                actual_executed_rejections=executed,worker_postsolve_modes=dict(modes),worker_reasons=dict(solver_reasons),
                native_current_section_exists=sum(r['current_section']['free_width_m']>1e-8 for r in native),
                native_preference_no_target_but_current_cut_exists=sum(bool(r['preference'] and r['preference']['preference_excludes_both_but_current_section_exists']) for r in native),
                input_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'events.jsonl',root/'scene/nav2.yaml')},
                scope='Recorded current model only. Source rows use source epoch (no double coast); native rows use exact recorded evaluation/prediction. Disk upper bound proves sampled-epoch exclusion only. Free section is not a temporal route. Preference does not reconstruct latch. No oracle, geometry changes, continuous-time infeasibility or physical safety proof.',
                dynamic_acceptance=False)
    return result,rows


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    report=root/'model_occupancy_audit.json';stream=root/'model_occupancy.jsonl'
    if report.exists() or stream.exists():raise SystemExit('refuse diagnostic overwrite')
    result,rows=evaluate(root)
    stream.write_text(''.join(json.dumps(r,allow_nan=False,separators=(',',':'))+'\n' for r in rows))
    report.write_text(json.dumps(result,allow_nan=False,indent=2)+'\n')
    print(json.dumps(result,indent=2))
