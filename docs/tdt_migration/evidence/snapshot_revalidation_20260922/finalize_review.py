#!/usr/bin/env python3
"""Audit the targeted A*/QP series from raw streams, without filling missing trials."""
import hashlib
import json
import statistics
from reference_experiment import (REPO, ROOT, HERE, PLANNERS, verify, model_metrics,
                                  endpoint, inspect_trial, read_rows, yaw_metrics, write_new)


def main():
    manifest=verify(); trials=[]; groups=[]
    expected={(p,t) for p in PLANNERS for t in range(1,6)}
    for planner in PLANNERS:
        group=[]; stopped=False
        for trial in range(1,6):
            path=ROOT/f'{planner}_{trial}'
            if not path.exists(): continue
            assert not stopped, 'A failed group must not have later repeats'
            assert trial==len(group)+1, 'Trial sequence must be contiguous'
            saved=json.loads((path/'target_geometry_summary.json').read_text())
            raw=inspect_trial(path); s=raw['summary']
            assert raw['raw_summary_matches'] and s['evidence_valid']
            assert saved['source_commit']==manifest['source_commit']
            assert saved['profile_sha256']==manifest['profiles'][planner]
            assert (saved['planner'],saved['trial'])==(planner,trial)
            for name,h in saved['artifacts_sha256'].items():
                assert hashlib.sha256((path/name).read_bytes()).hexdigest()==h, name
            rows=read_rows(path/'observation/trajectory.jsonl')
            geom=model_metrics(rows,manifest['body_polygon_m'],.03)
            assert geom==saved['target_geometry_metrics']
            assert yaw_metrics(path)==saved['yaw_metrics']
            checks=dict(s['checks']); checks.update({k:geom[k] for k in ('body_clearance_at_least_005m','padded_footprint_no_contact')})
            assert checks==saved['target_static_checks']
            assert saved['target_static_pass']==bool(s['evidence_valid'] and all(checks.values()))
            for key in ('action_status','recoveries','final_xy_error_m','final_yaw_error_rad','cross_track_rms_m','preflight'):
                assert saved[key]==s[key],key
            capture=json.loads((path/'runtime_geometry.json').read_text())
            for key,pose in [('nominal_after_navigation',(4.3,0.,0.)),('actual_final_after_navigation',s['final_pose'])]:
                assert json.loads(json.dumps(endpoint(capture,manifest['body_polygon_m'],.03,pose)))==saved[key]
            diagnostics=read_rows(path/'observation/planner_diagnostics.jsonl')
            admissions=[{'sim_t':e['t'],**json.loads(e['message'].split('snapshot_admission_v1=',1)[1])}
                        for e in diagnostics if e['message'].startswith('snapshot_admission_v1=')]
            assert admissions==saved['snapshot_admissions']
            assert saved['planner_success_diagnostics_complete']
            report={**saved,'raw_summary_recomputed':True,
                    'latest_snapshot_rejections':[a for a in admissions if a['decision']!='revalidated'],
                    'planner_failure_events':[e for e in diagnostics if 'failed to plan' in e['message']],
                    'safe_fallbacks':[e for e in diagnostics if 'GridBased: validated A* fallback:' in e['message']]}
            group.append(report); trials.append(report); stopped=not saved['target_static_pass']
        validated=[a for t in group for a in t['snapshot_admissions'] if a['decision']=='revalidated']
        groups.append({'planner':planner,'recorded':len(group),'target_static_pass':sum(t['target_static_pass'] for t in group),
            'recovery_counts':[t['recoveries'] for t in group],
            'preflight_statuses':[t['preflight']['status'] for t in group],
            'snapshot_revalidations':len(validated),
            'snapshot_revalidation_max_ms':max([a['validation_seconds']*1000 for a in validated],default=None),
            'snapshot_revalidation_mean_ms':statistics.mean(a['validation_seconds']*1000 for a in validated) if validated else None,
            'latest_snapshot_rejections':sum(len(t['latest_snapshot_rejections']) for t in group),
            'endpoint_rejections':sum(t['endpoint_rejections']['rejections'] for t in group),
            'mean_cross_track_rms_m':statistics.mean(t['cross_track_rms_m'] for t in group) if group else None,
            'qp_adopted':sum(t['qp_adopted_debug_messages'] for t in group),
            'safe_astar_fallbacks':sum(t['validated_astar_fallback_debug_messages'] for t in group)})
    recorded={(t['planner'],t['trial']) for t in trials};complete=recorded==expected
    result={'schema':'rm_tdt_planner/snapshot_revalidation_matrix/v1','series':ROOT.name,
        'source_commit':manifest['source_commit'],'consistent_and_audited':bool(trials),
        'recorded':len(trials),'planned_tdt_trials':10,'complete_targeted_matrix':complete,
        'complete_20_trial_matrix':False,'unexecuted_baselines':['navfn','smac2d'],
        'all_recorded_target_static_checks_pass':bool(trials and all(t['target_static_pass'] for t in trials)),
        'targeted_static_gate_pass':bool(complete and all(t['target_static_pass'] for t in trials)),
        'missing_trials':sorted(expected-recorded),'groups':groups,'trials':trials,
        'accepted_for_deployment':False,
        'scope':'Existing new-car 382/126 mm reference polygon on surrogate dynamics; same profiles as geometry reference v2. Heading and terminal selection unchanged.',
        'limits':['no new Navfn/Smac2D baselines','no dynamic obstacle or real vehicle acceptance',
                  'snapshot validation time is not the entire master-lock hold time',
                  'fixed footprint only; dynamic footprint reconfiguration synchronization not validated']}
    write_new(ROOT/'aggregate.json',result);write_new(HERE/'aggregate.json',result)
    print(json.dumps({k:result[k] for k in ('recorded','complete_targeted_matrix','targeted_static_gate_pass','groups')},indent=2))

if __name__=='__main__':main()
