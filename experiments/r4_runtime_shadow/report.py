#!/usr/bin/env python3
"""Read-only analysis of real cycles. Never recompute predictions or controls."""
import argparse
import bisect
import collections
import csv
import hashlib
import json
import math
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def number(row, key):
    try:
        return float(row.get(key, ''))
    except (ValueError, TypeError):
        return float('nan')


def stats(values):
    values = [v for v in values if math.isfinite(v)]
    return dict(n=len(values), median=float(np.median(values)) if values else None,
                p95=float(np.percentile(values, 95)) if values else None,
                max=max(values) if values else None,
                min=min(values) if values else None)


def forward(row):
    yaw = number(row, 'yaw')
    vx, vy = number(row, 'vx'), number(row, 'vy')
    return ((math.cos(yaw)*vx-math.sin(yaw)*vy)*number(row, 'route_tangent_x') +
            (math.sin(yaw)*vx+math.cos(yaw)*vy)*number(row, 'route_tangent_y'))


def summarize(rows):
    valid = [r for r in rows if r['valid'] == '1']
    invoked = [r for r in rows if r.get('solve_invoked') == '1']
    kernel = [r for r in invoked if r.get('solver_status') not in ['', 'not_run']]
    adjacent = [(a, b) for a, b in zip(rows, rows[1:])
                if a['valid'] == b['valid'] == '1' and int(b['cycle']) == int(a['cycle'])+1]
    same_route = [(a, b) for a, b in adjacent if a['path_digest'] == b['path_digest']]
    same_context = [(a,b) for a,b in same_route if all(a[k] == b[k] for k in ['map_digest','body_digest','limits_digest'])]
    stages_nonmonotone = 0
    for r in valid:
        stages = json.loads(r['stages_json'])
        stages_nonmonotone += any(b[5] < a[5]-1e-8 for a, b in zip(stages, stages[1:]))
    return dict(
        cycles=len(rows), valid_proposals=len(valid),
        valid_ratio=len(valid)/len(rows) if rows else None,
        unavailable_ratio=1-len(valid)/len(rows) if rows else None,
        follow_calls=len(invoked), osqp_calls=len(kernel),
        osqp_failure_ratio=sum(r['valid']!='1' for r in kernel)/len(kernel) if kernel else None,
        reasons=dict(collections.Counter(r['reason'] or 'available' for r in rows)),
        solver_statuses=dict(collections.Counter(r['solver_status'] or 'not_invoked' for r in rows)),
        solver_ms=stats(number(r, 'solver_ms') for r in kernel),
        follow_call_ms=stats(number(r, 'follow_call_ms') for r in invoked),
        acquire_to_finish_ms=stats(number(r, 'acquire_to_solve_finish_ms') for r in invoked),
        corridor_prepare_ms=stats(number(r, 'corridor_prepare_ms') for r in rows),
        prediction_age_at_acquire_ms=stats(number(r, 'prediction_age_at_acquire_ms') for r in rows),
        observation_age_at_acquire_ms=stats(number(r, 'observation_age_at_acquire_ms') for r in rows),
        state_age_at_acquire_ms=stats(number(r, 'state_age_at_acquire_ms') for r in rows),
        observation_to_proposal_ms=stats(number(r, 'observation_to_proposal_age_ms') for r in valid),
        prediction_to_proposal_ms=stats(number(r, 'prediction_to_proposal_age_ms') for r in valid),
        remaining_75ms_at_proposal=stats(number(r, 'remaining_75ms_at_proposal_ms') for r in valid),
        valid_wait_like=sum(math.hypot(number(r, 'vx'), number(r, 'vy')) <= .02 for r in valid),
        valid_forward=sum(forward(r) > .05 for r in valid),
        warm_proposals=sum(r['used_warm'] == '1' for r in valid),
        adjacent_valid_pairs=len(adjacent),
        vx_delta=stats(abs(number(b, 'vx')-number(a, 'vx')) for a, b in adjacent),
        vy_delta=stats(abs(number(b, 'vy')-number(a, 'vy')) for a, b in adjacent),
        direction_reversals={axis:sum(number(a, axis)*number(b, axis) < 0 and
                                     min(abs(number(a, axis)), abs(number(b, axis))) > .02
                                     for a, b in adjacent) for axis in ['vx', 'vy']},
        same_route_progress_pairs=len(same_route),
        measured_progress_decreases=sum(number(b, 'progress_input') < number(a, 'progress_input')-1e-6
                                        for a, b in same_route),
        same_path_static_body_limits_pairs=len(same_context),
        same_context_progress_decreases=sum(number(b,'progress_input') < number(a,'progress_input')-1e-6 for a,b in same_context),
        proposal_stage_progress_decreases=stages_nonmonotone,
        progress_rate=stats(number(r, 'progress_rate') for r in valid),
        dynamic_cost=stats(number(r, 'dynamic_cost') for r in valid),
        predicted_observed_clearance=stats(number(r,'min_predicted_observed_clearance') for r in valid),
        valid_plateau_stages=sum(int(r['plateau_stages']) for r in valid),
        nominal_dynamic_cost=stats(number(r, 'nominal_dynamic_cost') for r in kernel),
        nonzero_track_cycles=sum(number(r, 'tracks') > 0 for r in rows),
        fixed_yaw_incompatible_cycles=sum(abs(number(r, 'measured_wz')) > 1e-6 for r in rows),
        absolute_measured_wz=stats(abs(number(r, 'measured_wz')) for r in rows),
        body_versions=len({r['body_digest'] for r in rows if r.get('body_digest')}),
        path_versions=len({r['path_revision'] for r in rows if r.get('path_digest')}),
        producer_generations=sorted({(r.get('producer_id'), r.get('producer_generation'))
                                    for r in rows if r.get('producer_id')}))


def scene_report(out):
    with (out/'cycles.csv').open() as f:
        rows = list(csv.DictReader(f))
    refs = list(csv.DictReader((out/'references.csv').open()))
    events = json.loads((out/'events.json').read_text()) if (out/'events.json').exists() else {'events': []}
    event_by_kind = {e['kind']:e for e in events['events']}
    start = event_by_kind.get('goal_accepted', {}).get('ROS_ns')
    end = event_by_kind.get('observation_end', {}).get('ROS_ns')
    active = [r for r in rows if start is not None and number(r, 'acquire_ros_ns') >= start and
              (end is None or number(r, 'acquire_ros_ns') <= end)]
    actual = sorted((r for r in refs if r['kind'] == 'actual_output'),
                    key=lambda r:int(r['receipt_steady_ns']))
    owner_times = [int(r['receipt_steady_ns']) for r in actual]
    derived = []
    for r in rows:
        d = dict(cycle=r['cycle'], estimated_next_owner_phase_ms='', would_expire_before_next_owner_send='')
        if r['valid'] == '1':
            finish = int(r['follow_finish_steady_ns'])
            j = bisect.bisect_left(owner_times, finish)
            if j < len(owner_times):
                next_send_proxy = owner_times[j]
                deadline = int(r['acquire_steady_ns'])+75000000
                d.update(estimated_next_owner_phase_ms=(next_send_proxy-finish)/1e6,
                         would_expire_before_next_owner_send=int(next_send_proxy >= deadline))
        derived.append(d)
    with (out/'lease_estimates.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=['cycle','estimated_next_owner_phase_ms','would_expire_before_next_owner_send'],lineterminator='\n')
        writer.writeheader();writer.writerows(derived)
    active_ids = {r['cycle'] for r in active}
    estimated = [d for d in derived if d['cycle'] in active_ids and d['would_expire_before_next_owner_send'] != '']
    clear = event_by_kind.get('clear_target', {}).get('ROS_ns')
    resume = None
    consecutive = 0
    for r in active:
        if clear is None or number(r,'acquire_ros_ns') < clear:
            continue
        consecutive = consecutive+1 if r['valid'] == '1' and forward(r) > .05 else 0
        if consecutive >= 3:
            resume = (number(r,'acquire_ros_ns')-clear)/1e9
            break
    native_end = event_by_kind.get('native_goal_result', {}).get('ROS_ns')
    native_moving = [r for r in active if native_end is None or number(r, 'acquire_ros_ns') <= native_end]
    resets = sum(number(b,'acquire_ros_ns') <= number(a,'acquire_ros_ns') for a,b in zip(rows,rows[1:]))
    real_time_factor = ((number(active[-1],'acquire_ros_ns')-number(active[0],'acquire_ros_ns')) /
                        (number(active[-1],'acquire_steady_ns')-number(active[0],'acquire_steady_ns'))) if len(active)>1 else None
    summary = dict(scene=out.name, all_cycles=summarize(rows), goal_window=summarize(active),
                   native_motion_window=summarize(native_moving), events=events, clock_nonincreasing_cycles=resets,
                   goal_window_real_time_factor=real_time_factor,
                   offline_75ms=dict(estimated_cycles=len(estimated), valid_ratio=sum(d['would_expire_before_next_owner_send']==0 for d in estimated)/len(estimated) if estimated else None,
                                     next_phase_ms=stats(d['estimated_next_owner_phase_ms'] for d in estimated),
                                     meaning='Next actual cmd RECEIPT after Follow finish; proxy only, no owner send stamp/production enforcement/lease PASS.'),
                   first_three_forward_after_clear_target_seconds=resume,
                   dynamic_resume_confirmed=None,
                   clear_meaning='Target schedule event only. Actual sensed obstacle clearance must be checked separately; NA is not WAIT/resume.',
                   references=dict(collections.Counter(r['kind'] for r in refs)),
                   native_actual_vx=stats(number(r,'vx') for r in actual),
                   bag_counts=json.loads((out/'bag_message_counts.json').read_text()) if (out/'bag_message_counts.json').exists() else {})
    (out/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    if rows:
        origin=start if start is not None else number(rows[0],'acquire_ros_ns')
        times=[(number(r,'acquire_ros_ns')-origin)/1e9 for r in rows]
        fig, axes=plt.subplots(6,1,figsize=(12,14),sharex=True)
        def series(ax,key,label,valid_only=False,kernel_only=False,route_breaks=False):
            values=[number(r,key) if (not valid_only or r['valid']=='1') and
                    (not kernel_only or r.get('solver_status') not in ['', 'not_run']) else np.nan for r in rows]
            if route_breaks:
                for i in range(1,len(rows)):
                    if rows[i]['path_revision'] != rows[i-1]['path_revision']:values[i]=np.nan
            ax.plot(times,values,label=label,linewidth=1)
        for axis,key in [(axes[0],'vx'),(axes[1],'vy')]:
            series(axis,key,'R4 shadow',True)
            for kind,label in [('mppi_upstream','native MPPI upstream'),('actual_output','native actual output')]:
                rr=[r for r in refs if r['kind']==kind]
                axis.plot([(number(r,'receipt_ros_ns')-origin)/1e9 for r in rr],[number(r,key) for r in rr],label=label,alpha=.6,linewidth=.8)
            axis.set_ylabel(key+' (m/s)');axis.legend(loc='upper right',fontsize=8)
        series(axes[2],'dynamic_cost','solved cost',True);series(axes[2],'nominal_dynamic_cost','nominal cost (kernel calls)',kernel_only=True);axes[2].set_ylabel('dynamic cost')
        series(axes[3],'progress_input','measured projection (break at replans)',route_breaks=True);series(axes[3],'progress_next','proposal free-s next',True,route_breaks=True);axes[3].set_ylabel('route progress (m)')
        series(axes[4],'prediction_age_at_acquire_ms','prediction age @ acquire');series(axes[4],'observation_to_proposal_age_ms','observed-members age @ proposal',True);axes[4].set_ylabel('ROS age (ms)')
        series(axes[5],'acquire_to_solve_finish_ms','acquire → finish');series(axes[5],'remaining_75ms_at_proposal_ms','remaining original 75ms',True)
        axes[5].scatter([t for r,t in zip(rows,times) if r['valid']!='1'],[-5 for r in rows if r['valid']!='1'],s=3,label='unavailable');axes[5].set_ylabel('steady time (ms)')
        for ax in axes:
            ax.grid(alpha=.25)
            if start is not None:
                ax.set_xlim(-1,20.5)
            for e in events['events']:
                ax.axvline((e['ROS_ns']-origin)/1e9,alpha=.15,color='black')
        for ax in axes[2:]:ax.legend(loc='upper right',fontsize=8)
        axes[-1].set_xlabel('ROS seconds since accepted native goal (startup retained in CSV)')
        fig.suptitle(out.name+' — real inputs / diagnostic proposals only / unavailable gaps retained')
        fig.tight_layout();fig.savefig(out/'timeseries.png',dpi=140);plt.close(fig)
    return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=pathlib.Path);args=parser.parse_args()
    result={}
    for scene in ['S0','S1','S2']:
        out=args.output/scene
        if (out/'cycles.csv').exists():result[scene]=scene_report(out)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    paths=[p for p in args.output.rglob('*') if p.is_file() and
           any(parent.name in ['S0','S1','S2'] or 'failure' in parent.name for parent in p.parents) and
           p.name != 'evidence_hashes.json']
    hashes={str(p.relative_to(args.output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
    (args.output/'evidence_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')


if __name__ == '__main__':main()
