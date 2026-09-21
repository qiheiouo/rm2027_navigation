#!/usr/bin/env python3
"""Recompute reference results and freeze a truthful, incomplete targeted matrix."""
import hashlib
import json
import math
from pathlib import Path
import sys
from reference_experiment import (REPO, ROOT, HERE, verify, model_metrics, endpoint,
                                  read_rows, inspect_trial, write_new, yaw_metrics)


def main():
    m=verify(); trials=[]; sensitivity=[]
    a=(.382+math.sqrt(2)*.127)/2;b=.191
    alt=[(a,b),(b,a),(-b,a),(-a,b),(-a,-b),(-b,-a),(b,-a),(a,-b)]
    for name in ('tdt_astar','tdt_qp'):
        p=ROOT/(name+'_1'); saved=json.loads((p/'target_geometry_summary.json').read_text())
        original=inspect_trial(p)
        assert original['raw_summary_matches'] and original['summary']['evidence_valid']
        rows=read_rows(p/'observation/trajectory.jsonl');raw=json.loads((p/'runtime_geometry.json').read_text())
        assert model_metrics(rows,m['body_polygon_m'],.03)==saved['target_geometry_metrics']
        assert yaw_metrics(p)==saved['yaw_metrics']
        assert json.loads(json.dumps(endpoint(raw,m['body_polygon_m'],.03,(4.3,0.,0.))))==saved['nominal_after_navigation']
        assert json.loads(json.dumps(endpoint(raw,m['body_polygon_m'],.03,original['summary']['final_pose'])))==saved['actual_final_after_navigation']
        assert saved['planner_success_diagnostics_complete']
        assert not saved['target_static_checks']['no_recovery'] and not saved['target_static_pass']
        assert all(v for k,v in saved['target_static_checks'].items() if k!='no_recovery')
        assert saved['endpoint_rejections']['rejections']==0
        # Check immutable raw input hashes recorded after container cleanup.
        for file,h in saved['artifacts_sha256'].items():
            assert hashlib.sha256((p/file).read_bytes()).hexdigest()==h, file
        trial={**saved,'raw_summary_recomputed':True}
        diagnostics=read_rows(p/'observation/planner_diagnostics.jsonl')
        trial['snapshot_rejections']=[d for d in diagnostics if 'costmap or footprint changed' in d['message']]
        trial['safe_fallbacks']=[d for d in diagnostics if 'GridBased: validated A* fallback:' in d['message']]
        trials.append(trial)
        sensitivity.append({'planner':name,'scope':'127 mm offline replay on the SAME measured trajectory/map; no 127 mm navigation run',
            'trajectory_metrics':model_metrics(rows,alt,.03),
            'nominal_after_navigation':endpoint(raw,alt,.03,(4.3,0.,0.)),
            'actual_final_after_navigation':endpoint(raw,alt,.03,original['summary']['final_pose'])})
    report={'schema':'rm_tdt_planner/reference_geometry_pilot/v2','series':ROOT.name,
        'consistent_and_audited':True,'recorded':2,'planned_tdt_trials':10,'complete':False,
        'complete_20_trial_matrix':False,'all_recorded_target_static_checks_pass':False,
        'missing_trials':[[n,t] for n in ('tdt_astar','tdt_qp') for t in range(2,6)],
        'unexecuted_baselines':['navfn','smac2d'],'source_commit':m['source_commit'],
        'scope':'382/126 mm existing reference footprint; unchanged surrogate dynamics. Polygon-at-measured-yaw safety oracle; existing conservative-circle TDT backend.',
        'trials':trials,'sensitivity_127mm':sensitivity,'accepted_for_deployment':False,
        'decision':'No immediate terminal selector for this nominal goal; investigate retained snapshot invalidation and recovery separately, without weakening safety gates.'}
    write_new(ROOT/'aggregate.json',report)
    # A compact report in tracked docs retains all requested metrics; full raw logs stay in /home/build.
    write_new(HERE/'aggregate.json',report)
    print('Audited 2/2 trials; static gate fails only no_recovery. Repeats stopped, no complete-matrix claim.')
    return report

if __name__=='__main__':main()
