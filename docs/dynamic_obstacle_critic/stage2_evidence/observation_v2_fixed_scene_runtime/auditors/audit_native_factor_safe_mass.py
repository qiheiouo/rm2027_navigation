#!/usr/bin/env python3
"""Describe frozen raw-cost weights using distinct individual SDK SG labels."""
import argparse
import array
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
from analyze_native_factor_aggregation import full_context
from analyze_native_weights import analyze as original_weight_audit
from analyze_trial import load_truth_rows
from audit_mechanical_footprint import components
from audit_scan_geometry import fixture
from native_snapshot_io import read_snapshot
from native_witness_io import records
from prepare_processed_cv import verify
from verify_native_factor_raw import verify_record

FACTORS=('actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle')


def groups(rows,factor,epoch):
    """Descriptive fractions only; weights remain installed float32 values."""
    weights=[r['factor_weights'][factor] for r in rows];total=math.fsum(weights)
    safe=[r['individual_full_labels'][epoch]['conditional_safe_control_with_progress'] for r in rows]
    mass=math.fsum(w for w,ok in zip(weights,safe) if ok)
    partition=[]
    for raw_clear in [False,True]:
        for individual_safe in [False,True]:
            selected=[r for r in rows if (r['raw_factor_model_scores'][factor]['minimum_clearance']>.02)==raw_clear and
                r['individual_full_labels'][epoch]['conditional_safe_control_with_progress']==individual_safe]
            partition.append({'raw_model_margin_clear':raw_clear,'individual_SG_full_safe_progress':individual_safe,
                'count':len(selected),'native_weight_mass':math.fsum(r['factor_weights'][factor] for r in selected)})
    return {'individual_safe_count':sum(safe),'weight_sum_float64':total,'safe_individual_weight_mass':mass,
        'safe_individual_weight_fraction':mass/total,'failed_individual_weight_mass':math.fsum(w for w,ok in zip(weights,safe) if not ok),
        'failure_mass_by_gate':{key:math.fsum(r['factor_weights'][factor] for r in rows if not r['individual_full_labels'][epoch]['gates'][key]) for key in rows[0]['individual_full_labels'][epoch]['gates']},
        'raw_model_vs_distinct_individual_SG_partition':partition,
        'top_weights':[{'sampler_raw_row':r['sampler_raw_row'],'processed_witness_row':r['processed_witness_row'],
            'weight':r['factor_weights'][factor],'post_gamma_cost':r['factor_gamma_costs'][factor],
            'individual_SG_full_safe_progress':r['individual_full_labels'][epoch]['conditional_safe_control_with_progress'],
            'raw_model_score':r['raw_factor_model_scores'][factor],
            'individual_SG_model_score':r['individual_factor_model_scores'][factor]}
            for r in sorted(rows,key=lambda r:(-r['factor_weights'][factor],r['sampler_raw_row']))[:10]]}


def audit(source,witness,processed,factors,aggregation,stages,raw_archive,native_json=None):
    roots={'source':source,'witness':witness,'processed':processed,'factors':factors,'aggregation':aggregation,'stages':stages,'raw':raw_archive}
    for root in roots.values():verify(root)
    provenance=json.loads((witness/'provenance.json').read_text())
    if provenance['selected_source_ordinal']!=298 or provenance['source_manifest_sha256']!=hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest():
        raise ValueError('registered complete source298 witness required')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    selected=list(records(witness/'selected/witnesses.bin.gz'))
    if len(selected)!=1 or selected[0]['row_count']!=311 or not all(selected[0][k] for k in ['raw_velocity_pose_exact','aggregate_SG_exact','actual_command_double_bit_exact']):
        raise ValueError('actual SDK311 identity required')
    selected=selected[0];record={k:selected[k] for k in ['model_dt','initial_yaw']};record['rows']=311
    record['blocks']={name:{'values':values,'shape':[311,30]} for name,values in selected['blocks'].items()}
    for name,index in [('vx',0),('vy',1),('wz',5)]:
        if any(struct.pack('<f',record['blocks'][name]['values'][row*30])!=struct.pack('<f',native['speed'][index]) for row in range(311)):
            raise ValueError('individual SDK measured prefix replaced')
    legacy=json.loads(gzip.open(witness/'selected/analysis.json.gz','rb').read())['records'][0]['proposals']
    dynamic_frozen=json.loads((processed/'analysis.json').read_text())
    identity=json.loads((processed/'prepared/input_identity.json').read_text())
    if identity['witness_binary_sha256']!=hashlib.sha256((witness/'selected/witnesses.bin.gz').read_bytes()).hexdigest() or len(dynamic_frozen['rows'])!=311:
        raise ValueError('processed labels must refer to exact SDK311 tensor')
    for i,r in enumerate(dynamic_frozen['rows']):
        expected=identity['rows'][i]
        if r['processed_row']!=i or r['name']!=legacy[i]['name'] or r['name']!=expected['name'] or (i>=11 and r['name']!=f'sampler_{i-11}_bounds_SG_counterfactual'):
            raise ValueError('raw row to distinct processed row mapping')
    frozen=[json.loads(l) for l in gzip.open(aggregation/'native_output.jsonl.gz','rt')]
    native_rows=[json.loads(l) for l in native_json.read_text().splitlines()] if native_json else frozen
    if len(native_rows)!=5 or native_rows[0]!=frozen[0] or [r['factor'] for r in native_rows[1:]]!=list(FACTORS) or native_rows[1:]!=frozen[1:]:
        raise ValueError('frozen native factor costs/weights/controls changed')
    oldproof=original_weight_audit(source/'native_optimizer/weights_output.jsonl.gz',source/'native_optimizer/sdk_final_output.jsonl.gz',witness/'selected/analysis.json.gz')
    if oldproof['verdict']!='NATIVE WEIGHTS REAGGREGATION EXACT' or oldproof['selected_safe_label_count']!=39:
        raise ValueError('full original348 native weights and original39 witnesses required')
    scores={(r['factor'],r['path_kind'],r['row']):r for r in [json.loads(l) for l in gzip.open(factors/'factors.jsonl.gz','rt')][1:]}
    if set(scores)!={(f,k,i) for f in FACTORS for k,n in [('raw',300),('processed',311)] for i in range(n)}:
        raise ValueError('complete 2444 distinct path scores required')
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));scene=fixture(source/'scene_inputs');shapes=components(source/'scene_inputs')
    policy=json.loads((source/'policy.json').read_text())
    if policy['raw_costmap_threshold']!=203:raise ValueError('original raw203 threshold required')
    margin=json.loads((source/'dynamic_scores/score_298.json').read_text())['cost_parameters']['safety_margin']
    if margin!=.02:raise ValueError('original CV model margin required')
    dynamic={e:{**summary,'rows':[r['physical_dynamic_labels'][e] for r in dynamic_frozen['rows']]} for e,summary in dynamic_frozen['epoch_summaries'].items()}
    full={e:full_context(source,native,blocks,record,truth,scene,shapes,v['epoch'],policy,v) for e,v in dynamic.items()}
    source_pose=full['source_pose_clock_sensitivity']
    if any(r['geometry']!=legacy[i]['geometry'] or r['gates']!=legacy[i]['gates'] or r['progress_x']!=legacy[i]['progress_x'] or
        r['raw_interval_margin_lower']!=legacy[i]['raw_interval_margin_lower'] or r['conditional_safe_control_with_progress']!=legacy[i]['conditional_safe_control_with_progress'] for i,r in enumerate(source_pose)):
        raise ValueError('all original311 full source-pose labels changed')
    aggregate=json.loads((aggregation/'analysis.json').read_text());stage=json.loads((stages/'analysis.json').read_text())
    raw=json.loads((raw_archive/'analysis.json').read_text())
    if raw['native_source_ordinal']!=298 or len(raw['rows'])!=300:raise ValueError('same-raw physical label table required')
    rows=[]
    for i in range(300):
        p=i+11
        rows.append({'sampler_raw_row':i,'processed_witness_row':p,'processed_name':legacy[p]['name'],
            'factor_weights':{f:native_rows[j+1]['native_softmax_weights'][i] for j,f in enumerate(FACTORS)},
            'factor_gamma_costs':{f:native_rows[j+1]['post_gamma_costs'][i] for j,f in enumerate(FACTORS)},
            'raw_factor_model_scores':{f:scores[(f,'raw',i)] for f in FACTORS},
            'individual_factor_model_scores':{f:scores[(f,'processed',p)] for f in FACTORS},
            'raw_physical_dynamic_labels':raw['rows'][i]['physical_dynamic_labels'],
            'individual_physical_dynamic_labels':{e:v['rows'][p] for e,v in dynamic.items()},
            'individual_full_labels':{e:v[p] for e,v in full.items()}})
    summaries={f:{e:groups(rows,f,e) for e in dynamic} for f in FACTORS}
    baseline=summaries['actual_baseline']['source_pose_clock_sensitivity']
    if baseline['safe_individual_weight_mass']!=oldproof['selected_safe_label_weight_mass'] or baseline['safe_individual_weight_fraction']!=oldproof['selected_safe_label_weight_fraction']:
        raise ValueError('original safe39 weight mass changed')
    original_safe={r['sampler_raw_row'] for r in rows if r['individual_full_labels']['source_pose_clock_sensitivity']['conditional_safe_control_with_progress']}
    primary_safe={r['sampler_raw_row'] for r in rows if r['individual_full_labels']['actual_score_clock']['conditional_safe_control_with_progress']}
    top={max(range(300),key=lambda i:(native_rows[j+1]['native_softmax_weights'][i],-i)) for j in range(4)}
    subset=sorted(original_safe|primary_safe|top|{0});compact={'rows':len(subset),'model_dt':record['model_dt'],'initial_yaw':record['initial_yaw'],'blocks':{}}
    for name,block in record['blocks'].items():
        compact['blocks'][name]={'shape':[len(subset),30],'values':array.array('f',[v for i in subset for v in block['values'][(i+11)*30:(i+12)*30]])}
    scalar_input={'native_source_ordinal':298,'epoch_summaries':{e:{k:v[k] for k in ['epoch','samples']} for e,v in dynamic.items()},
        'rows':[{'factor':f'sampler_raw_{i}_individual_bounds_SG','full_conditional_labels':rows[i]['individual_full_labels']} for i in subset]}
    scalar=verify_record(source,compact,scalar_input)
    scalar['verified_sampler_raw_rows']=subset;scalar['selection']={'original_source_pose_safe_rows':sorted(original_safe),'primary_scoreclock_safe_rows':sorted(primary_safe),'four_factor_highest_weight_rows':sorted(top),'registered_row0':0}
    for e,data in scalar['epochs'].items():
        for r in data['rows']:r['sampler_raw_row']=subset[r['row']];r['processed_witness_row']=subset[r['row']]+11
    result={'schema':'native_factor_safe_mass_v1','native_source_ordinal':298,'sampler_count':300,'witness_processed_offset':11,
        'all311_source_pose_full_geometry_and_gates_exact':True,'original348_native_weights_full_chain_exact':True,
        'baseline_original_safe39_mass_and_fraction_exact':True,'model_margin':margin,
        'epoch_summaries':{e:{k:v[k] for k in ['epoch','horizon','samples','max_sample_interval']} for e,v in dynamic.items()},
        'factor_summaries':summaries,'aggregate_stage_gate_results':{f:{e:{'before':stage['rows'][2*j]['full_conditional_labels'][e],
            'after':aggregate['rows'][j]['full_conditional_labels'][e]} for e in dynamic} for j,f in enumerate(FACTORS)},
        'independent_raw_subset':subset,'rows':rows,
        'references':{name:hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() for name,root in roots.items()},
        'scope':'Descriptive native raw-cost weights versus separately bounded/actual-history SG individual paths and failed aggregates.',
        'limits':['Raw risk, individual SG physical labels and aggregate labels refer to different controls/paths and cannot replace one another.',
            'All source-pose source39 and all primary safe labels independently checked for frozen raw203; remaining failed labels use conservative vector bounds, not all300 exact scalar distances.',
            'Gate failure masses overlap. Descriptive fractions do not choose controls or imply convexity, optimizer/sampler global failure, or execution.',
            'Future/source oracles inadmissible online; no new motion/support assumption, CA/sampler/custom ranking or deployment acceptance.',
            'Original task/raw203/age FAILED and composite NOT ESTABLISHED remain.']}
    return result,scalar


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','processed','factors','aggregation','stages','raw_archive','report','scalar_report']:parser.add_argument(name,type=Path)
    parser.add_argument('--native-json',type=Path);args=parser.parse_args()
    if args.report.exists() or args.scalar_report.exists():raise FileExistsError('refusing to overwrite safe mass reports')
    values=audit(*[getattr(args,n).resolve() for n in ['source','witness','processed','factors','aggregation','stages','raw_archive']],args.native_json)
    for path,value in zip([args.report,args.scalar_report],values):
        with path.open('x') as stream:stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'factor_summaries':values[0]['factor_summaries'],'verified_raw_subset':values[0]['independent_raw_subset'],'scalar':values[1]['verdict']}),flush=True)
