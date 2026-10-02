#!/usr/bin/env python3
"""Compare full gates before/after SG on four fixed native cost contexts."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
from analyze_trial import load_truth_rows
from analyze_native_factor_aggregation import full_context
from audit_mechanical_footprint import components
from audit_raw_rollout_objective import geometry
from audit_scan_geometry import fixture
from native_sg_stage_io import read_paths, FACTORS, STAGES
from native_snapshot_io import read_snapshot
from prepare_processed_cv import verify
from verify_raw_rollout_geometry import verify as verify_scalar
from verify_native_factor_raw import verify_record


def analyze(source,aggregation,prepared,paths,native_json,score_json):
    verify(source);verify(aggregation)
    record=read_paths(paths);identity=json.loads((prepared/'input_identity.json').read_text())
    if (identity['stage_paths_sha256']!=hashlib.sha256(paths.read_bytes()).hexdigest() or
            identity['factor_input_sha256']!=hashlib.sha256((prepared/'factor_input.bin').read_bytes()).hexdigest() or
            not identity['four_after_paths_all_nine_blocks_bit_exact']):
        raise ValueError('stage path/query identity required')
    rows=[json.loads(l) for l in native_json.read_text().splitlines()]
    old=[json.loads(l) for l in gzip.open(aggregation/'native_output.jsonl.gz','rt')]
    if (len(rows)!=5 or rows[0]['rows']!=8 or rows[0]['actual_baseline_bit_exact_mask']!=127 or
            rows[0]['native_library']!='/opt/ros/humble/lib/libmppi_controller.so' or
            not rows[0]['measured_prefix_exact_all_eight'] or
            rows[0]['path_order']!='factor-major; bounded_mean_before_SG then actual_history_after_SG' or rows[1:]!=old[1:]):
        raise ValueError('original native context and factor controls must remain exact')
    for row,(factor,stage) in enumerate(record['row_identity']):
        n=rows[row//2+1]
        if n['factor']!=factor:raise ValueError('stage factor join mismatch')
        key='mean_before_SG' if row%2==0 else 'mean_after_SG'
        for axis,field in enumerate(['cvx','cvy','cwz']):
            if struct.pack('<30f',*record['blocks'][field]['values'][row*30:(row+1)*30])!=struct.pack('<30f',*n[key][axis*30:(axis+1)*30]):
                raise ValueError('stage control tensor join mismatch')
    costs=[json.loads(l) for l in score_json.read_text().splitlines()]
    if (len(costs)!=1233 or costs[0]['processed_rows']!=8 or costs[0]['source_ordinal']!=298 or
            costs[0]['actual_raw_risk_bit_exact_rows']!=300 or costs[0]['actual_raw_after_bit_exact_rows']!=300 or
            costs[0]['model_dt']!=record['model_dt']):
        raise ValueError('eight-stage scoring matrix budget')
    indexed={(r['factor'],r['path_kind'],r['row']):r for r in costs[1:]}
    if set(indexed)!={(f,k,i) for f in FACTORS for k,n in [('raw',300),('processed',8)] for i in range(n)}:
        raise ValueError('complete unique stage scores required')
    frozen={(r['factor'],r['path_kind'],r['row']):r for r in [json.loads(l) for l in gzip.open(aggregation/'scores.jsonl.gz','rt')][1:]}
    for key,r in indexed.items():
        if (not math.isfinite(r['risk']) or r['risk']<0 or not math.isfinite(r['minimum_clearance']) or
                r['collision_time'] is not None and not 0<r['collision_time']<=30*record['model_dt']):
            raise ValueError('finite stage score required')
        if key[1]=='raw' or key[2]%2:
            expected=frozen[key if key[1]=='raw' else (key[0],key[1],key[2]//2)]
            for field in ['risk','minimum_clearance','collision_time']:
                a,b=r[field],expected[field]
                if not (a is None and b is None or a is not None and b is not None and struct.pack('<d',a)==struct.pack('<d',b)):
                    raise ValueError('raw/four after scores changed')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()))
    scene=fixture(source/'scene_inputs');shapes=components(source/'scene_inputs');policy=json.loads((source/'policy.json').read_text())
    previous=json.loads((aggregation/'analysis.json').read_text())
    epochs={e:r['epoch'] for e,r in previous['epoch_summaries'].items()}
    path_blocks={k:record['blocks'][k] for k in ['x','y','yaw']}
    dynamic={e:geometry(native,path_blocks,record['initial_yaw'],truth,shapes,scene,t,policy) for e,t in epochs.items()}
    full={e:full_context(source,native,blocks,record,truth,scene,shapes,t,policy,dynamic[e]) for e,t in epochs.items()}
    after_exact={}
    for e,labels in full.items():
        after_exact[e]=all(labels[2*i+1]==previous['rows'][i]['full_conditional_labels'][e] and
            dynamic[e]['rows'][2*i+1]==previous['rows'][i]['physical_dynamic_labels'][e] for i in range(4))
    if not all(after_exact.values()):raise ValueError('four after-SG full geometry/gates changed')
    output_rows=[]
    for row,(factor,stage) in enumerate(record['row_identity']):
        control=[record['blocks'][k]['values'][row*30+1] for k in ['cvx','cvy','cwz']]
        output_rows.append({'native_factor_row':row,'factor':factor,'stage':stage,
            'role':'pre_SG_model_hypothesis' if row%2==0 else 'actual_after_SG_baseline_replica' if row==1 else 'after_SG_counterfactual',
            'offset1_control':control,'matching_factor_model_score':indexed[(factor,'processed',row)],
            'all_factor_model_scores':{f:indexed[(f,'processed',row)] for f in FACTORS},
            'physical_dynamic_labels':{e:v['rows'][row] for e,v in dynamic.items()},
            'full_conditional_labels':{e:v[row] for e,v in full.items()}})
    summaries={e:{'epoch':v['epoch'],'horizon':v['horizon'],'samples':v['samples'],'max_sample_interval':v['max_sample_interval'],
        'padded_contact_on_aggregate_grid_rows':sum(r['physical_dynamic_labels'][e]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in output_rows),
        'full_conditional_safe_progress_rows_by_stage':{s:sum(r['full_conditional_labels'][e]['conditional_safe_control_with_progress'] for r in output_rows if r['stage']==s) for s in STAGES}}
        for e,v in dynamic.items()}
    report={'schema':'native_sg_stage_v1','native_source_ordinal':298,'four_after_full_labels_exact':after_exact,
        'stage_paths_sha256':hashlib.sha256(paths.read_bytes()).hexdigest(),'native_CPP_sha256':hashlib.sha256(native_json.read_bytes()).hexdigest(),
        'pure_score_CPP_sha256':hashlib.sha256(score_json.read_bytes()).hexdigest(),'epoch_summaries':summaries,'rows':output_rows,
        'scope':'Eight fixed-context bounded-mean/actual-history SG model paths; original complete three-second gates.',
        'limits':['Before-SG hypotheses are not published controls or actual executed native plans.',
            'Legacy SG_full_sequence_and_return_bounds key tests the full sequence and offset1 at either stage; it does not assert SG was applied to before paths.',
            'Raw216 cell source remains unclassified; no threshold/cell deletion or online oracle.',
            'No closed-loop causal claim, CA/sampler/ranking restoration, new physical support bound or deployment acceptance.',
            'Original task/raw203/age FAILED and composite NOT ESTABLISHED remain.']}
    scalar=verify_scalar(source,report,path_blocks,'native_factor_row',1,'padded_contact_on_aggregate_grid_rows')
    raw=verify_record(source,record,report)
    report['stage_pairs']=[{'factor':f,'before_row':2*i,'after_row':2*i+1,
        'epochs':{e:{'before_full_pass':full[e][2*i]['conditional_safe_control_with_progress'],
            'after_full_pass':full[e][2*i+1]['conditional_safe_control_with_progress'],
            'before_sampled_raw_contact':raw['epochs'][e]['rows'][2*i]['sampled_raw_cell_or_boundary_contact'],
            'after_sampled_raw_contact':raw['epochs'][e]['rows'][2*i+1]['sampled_raw_cell_or_boundary_contact'],
            'before_raw_margin_m':full[e][2*i]['raw_interval_margin_lower'],'after_raw_margin_m':full[e][2*i+1]['raw_interval_margin_lower']} for e in epochs}} for i,f in enumerate(FACTORS)]
    return report,scalar,raw


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','aggregation','prepared','paths','native_json','score_json','report','scalar_report','raw_report']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    reports=analyze(args.source.resolve(),args.aggregation.resolve(),args.prepared,args.paths,args.native_json,args.score_json)
    for path,value in zip([args.report,args.scalar_report,args.raw_report],reports):
        with path.open('x') as stream:stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'epochs':reports[0]['epoch_summaries'],'pairs':reports[0]['stage_pairs'],'scalar':reports[1]['verdict'],'raw':reports[2]['verdict']}),flush=True)
