#!/usr/bin/env python3
"""Descriptive paired run evidence; one native MPPI noise realization per run."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from audit_run import audit, stats, epoch

PAIRS=(('crossing','shadow06','mpc01'),('head_on','head_on02','head_on_mpc01'))

def run(root):
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    a=audit(root);(root/'audit.json').write_text(json.dumps(a,indent=2)+'\n')
    s=json.loads((root/'run_summary.json').read_text())
    begin=s['goal_epoch_s']*1e9
    finish=next((e['receipt_sim_ns'] for e in events if e['topic']=='goal_result'),s['final_sim_s']*1e9)
    commands=[e for e in events if e['topic']=='/nav2/cmd_vel' and begin<=e['receipt_sim_ns']<=finish]
    actuator=[e for e in events if e['topic']=='/cmd_vel' and begin<=e['receipt_sim_ns']<=finish]
    gaps=[(b['receipt_monotonic_ns']-a['receipt_monotonic_ns'])/1e9 for a,b in zip(commands,commands[1:])]
    actuator_gaps=[(b['receipt_monotonic_ns']-a['receipt_monotonic_ns'])/1e9 for a,b in zip(actuator,actuator[1:])]
    selections=[dict(sim_s=e['receipt_sim_ns']/1e9,wall_ns=e['receipt_monotonic_ns'],controller=e['data']['data'])
                for e in events if e['topic']=='/controller_selector']
    contacts=[dict(sim_s=epoch(e['data']['header']['stamp'])/1e9,
                   pairs=[(c['collision1']['name'],c['collision2']['name']) for c in e['data']['contacts']])
              for e in events if e['topic']=='/simulation/oracle/contacts' and 'rm_sentry_2027' in json.dumps(e['data'])]
    fallback=None
    for i,selected in enumerate(selections):
        if selected['controller']=='FollowPathMPPI' and i>0 and selections[i-1]['controller']=='FollowPathTemporalMPC':
            prior=[e for e in events if e['topic']=='/temporal_mpc/health'
                   and selections[i-1]['wall_ns']<=e['receipt_monotonic_ns']<=selected['wall_ns']]
            executed_fail=[e for e in prior if json.loads(e['data']['data'])['executed'] and not json.loads(e['data']['data'])['ready']]
            bad=executed_fail[-1] if executed_fail else next((e for e in reversed(prior) if not json.loads(e['data']['data'])['ready']),None)
            near=[e for e in events if e['topic']=='/temporal_mpc/health'
                  and abs(e['receipt_monotonic_ns']-selected['wall_ns'])<50_000_000
                  and json.loads(e['data']['data'])['executed']
                  and not json.loads(e['data']['data'])['ready']]
            correlated=min(near,key=lambda e:abs(e['receipt_monotonic_ns']-selected['wall_ns'])) if near else None
            nearby=[e for e in events if e['topic']=='/temporal_mpc/health'
                    and abs(e['receipt_monotonic_ns']-selected['wall_ns'])<50_000_000
                    and (not json.loads(e['data']['data'])['ready'] or json.loads(e['data']['data'])['fallback_requested'])]
            diagnostic=min(nearby,key=lambda e:abs(e['receipt_monotonic_ns']-selected['wall_ns'])) if nearby else None
            fallback=dict(nearby_health_diagnostic=json.loads(diagnostic['data']['data']) if diagnostic else None,
                          nearby_health_receipt_offset_s=(diagnostic['receipt_monotonic_ns']-selected['wall_ns'])/1e9 if diagnostic else None,
                          causal_failure_to_switch_s=None,
                          causal_health_receipt_observed_before_selection=bad is not None,
                          nearby_executed_rejection_reason=json.loads(correlated['data']['data'])['reason'] if correlated else None,
                          nearby_rejection_receipt_offset_s=(correlated['receipt_monotonic_ns']-selected['wall_ns'])/1e9 if correlated else None,
                          receipt_order_note='Independent DDS subscriptions; post-selection receipt is correlation, not measured causal latency',
                          selection_sim_s=selected['sim_s'],
                          health_reason=json.loads(bad['data']['data'])['reason'] if bad else None,
                          health_to_selection_wall_s=(selected['wall_ns']-bad['receipt_monotonic_ns'])/1e9 if bad else None)
    graph=[e['data'] for e in events if e['topic']=='graph_publishers']
    unique=bool(graph and all(len({tuple(i['endpoint_gid']) for i in g['/nav2/cmd_vel']})==1
        and len({tuple(i['endpoint_gid']) for i in g['/cmd_vel']})==1 for g in graph))
    feasible_solves=[json.loads(e['data']['data'])['elapsed_s'] for e in events
                     if e['topic']=='/temporal_mpc/solver_diagnostic' and json.loads(e['data']['data'])['feasible']]
    rejections=Counter()
    for e in events:
        if e['topic']=='/temporal_mpc/solver_diagnostic':
            d=json.loads(e['data']['data'])
            if not d['feasible']:rejections[d['reason']]+=1
    execution=json.loads((root/'execution_audit.json').read_text()) if (root/'execution_audit.json').exists() else None
    return dict(execution_audit=execution,run=root.name,action_status=s['result']['status'] if s['result'] else None,
                goal_sim_s=s['goal_epoch_s'],duration_sim_s=s['final_sim_s']-s['goal_epoch_s'],
                actual_robot_contact_messages=len(contacts),first_actual_contact_sim_s=contacts[0]['sim_s'] if contacts else None,
                sampled_clearance_m=a['sampled_full_envelope_clearance_m'],diagnostic_sweep_lower_m=a['diagnostic_sweep_lower_m'],
                oracle_max_gap_s=a['oracle_gap_s']['max'] if a['oracle_gap_s'] else None,
                native_executed_count=a['native_executed_s']['count'] if a['native_executed_s'] else 0,
                native_executed_ready_count=a['native_executed_ready_count'],native_executed_s=a['native_executed_s'],
                feasible_worker_s=stats(feasible_solves),worker_all_s=a['solver_s'],solver_rejections=dict(rejections),
                controller_command_count=len(commands),controller_command_gap_wall_s=stats(gaps),
                actuator_command_count=len(actuator),actuator_command_gap_wall_s=stats(actuator_gaps),
                command_endpoints_unique_at_snapshots=unique,selections=selections,observed_fallback=fallback,
                accepted_mpc_cycle_share=a['native_executed_ready_count']/len(commands) if commands else 0.,
                sensor_contract_gate=a['sensor_contract_gate'],measured_model_gate=a['measured_model_gate'],
                physical_acceptance=False,false_block=None,
                logged_actuator_75ms_continuity_gate=bool(actuator_gaps and max(actuator_gaps)<=.075),
                logged_controller_75ms_continuity_gate=bool(gaps and max(gaps)<=.075),
                physical_fail_reasons=[reason for failed,reason in (
                    (not a['action_success'],'action not successful'),(bool(contacts),'actual contact'),
                    (a['diagnostic_sweep_lower_m'] is None or a['diagnostic_sweep_lower_m']<.05,'clearance lower diagnostic below .05m'),
                    (True,'continuous plant error/speed bound not certified')) if failed])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--pairs-file');parser.add_argument('--common-guard',action='store_true');args=parser.parse_args();root=Path(args.directory)
    pairs=[]
    names=json.loads(Path(args.pairs_file).read_text()) if args.pairs_file else PAIRS
    for scenario,b0,mpc in names:
        left,right=root/b0,root/mpc
        matched={name:hashlib.sha256((left/'scene'/name).read_bytes()).hexdigest()==hashlib.sha256((right/'scene'/name).read_bytes()).hexdigest()
                 for name in ('world.sdf','nav2.yaml','tracker.yaml','bridge.yaml')}
        pairs.append(dict(scenario=scenario,config_byte_identical=matched,b0=run(left),candidate=run(right),
                          repetition_count=1,mppi_internal_noise_seed=None,statistical_net_benefit_established=False))
    result=dict(baseline_strategy='MPPI + common ExecutionGuard' if args.common_guard else 'MPPI + VelocitySmoother',
                candidate_strategy='MPC + MPPI fallback + common ExecutionGuard' if args.common_guard else 'MPC + MPPI fallback + VelocitySmoother',
                pairs=pairs,all_pair_config_identical=all(all(p['config_byte_identical'].values()) for p in pairs),
                candidate_frozen_for_deployment=False,dynamic_acceptance=False,
                conclusion='Recorded engineering/physical results only. No safe successful B0 witness or continuous safety certificate; deployment acceptance remains false.')
    (root/'paired_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
