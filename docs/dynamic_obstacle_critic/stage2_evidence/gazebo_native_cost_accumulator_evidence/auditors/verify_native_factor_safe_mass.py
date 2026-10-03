#!/usr/bin/env python3
"""Independent exact-rational mass, row-provenance and positive raw-label checks."""
import argparse
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import yaml
from native_witness_io import records
from native_snapshot_io import read_snapshot
from prepare_processed_cv import verify as verify_archive

FACTORS=('actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle')


def same_float(a,b,width=64):
    if a is None or b is None:return a is None and b is None
    return struct.pack('<d' if width==64 else '<f',a)==struct.pack('<d' if width==64 else '<f',b)


def rational_sum(values):return float(sum((Fraction.from_float(v) for v in values),Fraction()))


def check(source,witness,processed,factors,aggregation,stages,raw_archive,report,scalar):
    roots={'source':source,'witness':witness,'processed':processed,'factors':factors,'aggregation':aggregation,'stages':stages,'raw':raw_archive}
    for name,root in roots.items():
        verify_archive(root)
        if report['references'][name]!=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest():raise ValueError('safe mass reference identity')
    rows=report['rows']
    if report['schema']!='native_factor_safe_mass_v1' or report['native_source_ordinal']!=298 or report['sampler_count']!=300 or len(rows)!=300 or report['witness_processed_offset']!=11 or report['model_margin']!=.02:
        raise ValueError('complete 300-row safe mass table required')
    record=list(records(witness/'selected/witnesses.bin.gz'))[0];native,_=read_snapshot(source/'native_cycles/cycle_298.json')
    legacy=json.loads(gzip.open(witness/'selected/analysis.json.gz','rb').read())['records'][0]['proposals']
    dynamic=json.loads((processed/'analysis.json').read_text());raw=json.loads((raw_archive/'analysis.json').read_text())
    weights=[json.loads(l) for l in gzip.open(aggregation/'native_output.jsonl.gz','rt')][1:]
    scores={(r['factor'],r['path_kind'],r['row']):r for r in [json.loads(l) for l in gzip.open(factors/'factors.jsonl.gz','rt')][1:]}
    policy=json.loads((source/'policy.json').read_text());epochs=report['epoch_summaries']
    follow=yaml.safe_load((source/'profile.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    if set(epochs)!=set(dynamic['epoch_summaries']) or set(report['factor_summaries'])!=set(FACTORS):raise ValueError('complete epoch/factor summaries required')
    for e,s in epochs.items():
        for key in ['epoch','horizon','samples','max_sample_interval']:
            if s[key]!=dynamic['epoch_summaries'][e][key]:raise ValueError('original full epoch grid changed')
    gate_keys=set(legacy[11]['gates'])
    for i,r in enumerate(rows):
        p=i+11
        if r['sampler_raw_row']!=i or r['processed_witness_row']!=p or r['processed_name']!=f'sampler_{i}_bounds_SG_counterfactual':
            raise ValueError('raw/individual SG row mapping confused')
        if set(r['factor_weights'])!=set(FACTORS) or set(r['factor_gamma_costs'])!=set(FACTORS):raise ValueError('complete factor weight fields required')
        for j,f in enumerate(FACTORS):
            if not same_float(r['factor_weights'][f],weights[j]['native_softmax_weights'][i]) or not same_float(r['factor_gamma_costs'][f],weights[j]['post_gamma_costs'][i]):
                raise ValueError('native factor weight/cost bits changed')
            for field,kind,index in [('raw_factor_model_scores','raw',i),('individual_factor_model_scores','processed',p)]:
                if r[field][f]!=scores[(f,kind,index)]:raise ValueError('cost path identity confused')
        if r['raw_physical_dynamic_labels']!=raw['rows'][i]['physical_dynamic_labels'] or r['individual_physical_dynamic_labels']!=dynamic['rows'][p]['physical_dynamic_labels']:
            raise ValueError('raw/individual physical labels confused')
        if set(r['individual_full_labels'])!=set(epochs):raise ValueError('complete individual epoch labels required')
        for e,label in r['individual_full_labels'].items():
            if set(label['gates'])!=gate_keys or any(type(v) is not bool for v in label['gates'].values()) or label['conditional_safe_control_with_progress']!=all(label['gates'].values()):
                raise ValueError('complete full gate truth table required')
            geo=label['geometry'];values={name:record['blocks'][name][p*30:(p+1)*30] for name in ['cvx','cvy','cwz']}
            if not math.isfinite(label['raw_interval_margin_lower']) or not math.isfinite(label['progress_x']) or any(not math.isfinite(v) for g in geo.values() for v in g.values()):
                raise ValueError('finite full geometry values required')
            tol=policy['bounds_tolerance'];end=record['blocks']['x'][p*30+29];rounded=struct.unpack('<f',struct.pack('<f',native['pose'][0]))[0]
            expected={'body_clearance':min(geo['body']['dynamic_lower'],geo['body']['static_lower'])>=policy['body_clearance'],
                'full_mechanical_clearance':min(geo['mechanical']['dynamic_lower'],geo['mechanical']['static_lower'])>=policy['body_clearance'],
                'padded_no_contact':all(min(geo[name]['dynamic_lower'],geo[name]['static_lower'])>0 for name in ['padded_configured','padded_native']),
                'frozen_native_raw203_with_interval_reserve':label['raw_interval_margin_lower']>1e-9,
                'SG_full_sequence_and_return_bounds':all(follow['vx_min']-tol<=x<=follow['vx_max']+tol for x in values['cvx']) and all(abs(x)<=follow['vy_max']+tol for x in values['cvy']) and all(abs(x)<=follow['wz_max']+tol for x in values['cwz']),
                'strict_positive_progress':end-native['pose'][0]>0,'progress_beyond_source_float_rounding':end>rounded}
            if label['gates']!=expected or label['progress_x']!=end-native['pose'][0]:raise ValueError('derived full gates/progress changed')
            for name in ['body','mechanical','padded_native']:
                d=r['individual_physical_dynamic_labels'][e]['geometry'][name]
                if geo[name]['dynamic_lower']!=d['interval_lower'] or geo[name]['dynamic_sample_min']!=d['sample_min']:
                    raise ValueError('full dynamic labels changed')
            if e=='source_pose_clock_sensitivity':
                original=legacy[p]
                if any(label[k]!=original[k] for k in ['gates','conditional_safe_control_with_progress','progress_x','geometry','raw_interval_margin_lower']):
                    raise ValueError('original complete source-pose label changed')
    maximum_error=0.;partitions=0
    for f in FACTORS:
        for e in epochs:
            s=report['factor_summaries'][f][e];safe=[r for r in rows if r['individual_full_labels'][e]['conditional_safe_control_with_progress']]
            failed=[r for r in rows if not r['individual_full_labels'][e]['conditional_safe_control_with_progress']]
            total=rational_sum(r['factor_weights'][f] for r in rows);mass=rational_sum(r['factor_weights'][f] for r in safe)
            expected={'individual_safe_count':len(safe),'weight_sum_float64':total,'safe_individual_weight_mass':mass,
                'safe_individual_weight_fraction':mass/total,'failed_individual_weight_mass':rational_sum(r['factor_weights'][f] for r in failed)}
            for key,value in expected.items():
                if s[key]!=value:raise ValueError('independent rational safe mass/count differs')
                maximum_error=max(maximum_error,abs(s[key]-value))
            if set(s['failure_mass_by_gate'])!=gate_keys:raise ValueError('complete failure gate masses required')
            for key in gate_keys:
                if s['failure_mass_by_gate'][key]!=rational_sum(r['factor_weights'][f] for r in rows if not r['individual_full_labels'][e]['gates'][key]):
                    raise ValueError('independent rational gate mass differs')
            group=s['raw_model_vs_distinct_individual_SG_partition']
            if len(group)!=4 or [(g['raw_model_margin_clear'],g['individual_SG_full_safe_progress']) for g in group]!=[(a,b) for a in [False,True] for b in [False,True]]:
                raise ValueError('complete disjoint partition required')
            for g in group:
                selected=[r for r in rows if (r['raw_factor_model_scores'][f]['minimum_clearance']>report['model_margin'])==g['raw_model_margin_clear'] and r['individual_full_labels'][e]['conditional_safe_control_with_progress']==g['individual_SG_full_safe_progress']]
                if g['count']!=len(selected) or g['native_weight_mass']!=rational_sum(r['factor_weights'][f] for r in selected):raise ValueError('independent rational partition mass/count differs')
                partitions+=1
            top=sorted(rows,key=lambda r:(-r['factor_weights'][f],r['sampler_raw_row']))[:10]
            if len(s['top_weights'])!=10 or [r['sampler_raw_row'] for r in s['top_weights']]!=[r['sampler_raw_row'] for r in top]:raise ValueError('top raw weight row ordering changed')
            for observed,original in zip(s['top_weights'],top):
                if (observed['processed_witness_row']!=original['processed_witness_row'] or observed['weight']!=original['factor_weights'][f] or observed['post_gamma_cost']!=original['factor_gamma_costs'][f] or
                    observed['individual_SG_full_safe_progress']!=original['individual_full_labels'][e]['conditional_safe_control_with_progress'] or
                    observed['raw_model_score']!=original['raw_factor_model_scores'][f] or observed['individual_SG_model_score']!=original['individual_factor_model_scores'][f]):
                    raise ValueError('top raw/processed labels changed')
    original_safe={r['sampler_raw_row'] for r in rows if r['individual_full_labels']['source_pose_clock_sensitivity']['conditional_safe_control_with_progress']}
    primary_safe={r['sampler_raw_row'] for r in rows if r['individual_full_labels']['actual_score_clock']['conditional_safe_control_with_progress']}
    tops={max(rows,key=lambda r:(r['factor_weights'][f],-r['sampler_raw_row']))['sampler_raw_row'] for f in FACTORS}
    subset=sorted(original_safe|primary_safe|tops|{0})
    if len(original_safe)!=39 or report['independent_raw_subset']!=subset or scalar['verified_sampler_raw_rows']!=subset or scalar['verdict']!='PASS':
        raise ValueError('all positive safe labels and preregistered raw subset required')
    for e,s in scalar['epochs'].items():
        if s['threshold']!=203 or s['samples_per_path']!=epochs[e]['samples'] or len(s['rows'])!=len(subset):raise ValueError('complete scalar subset grid required')
        for j,w in enumerate(s['rows']):
            raw_index=subset[j];label=rows[raw_index]['individual_full_labels'][e]
            if w['row']!=j or w['sampler_raw_row']!=raw_index or w['processed_witness_row']!=raw_index+11 or w['raw_gate_pass']!=label['gates']['frozen_native_raw203_with_interval_reserve'] or w['captured_conservative_margin_m']!=label['raw_interval_margin_lower']:
                raise ValueError('positive raw scalar/row join changed')
    return {'verdict':'PASS','raw_rows':300,'individual_SG_rows':300,'processed_offset':11,'factor_epoch_groups':8,
        'disjoint_partitions':partitions,'maximum_exact_rational_mass_error':maximum_error,'independent_raw_subset_rows':len(subset),
        'all_primary_and_original_positive_labels_in_scalar_subset':True,
        'scope':'Independent rational masses and frozen row/type/gate joins; scalar subset distances recomputed by separate raw verifier; no control selection.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','processed','factors','aggregation','stages','raw_archive','report','scalar','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=check(*[getattr(args,n).resolve() for n in ['source','witness','processed','factors','aggregation','stages','raw_archive']],json.loads(args.report.read_text()),json.loads(args.scalar.read_text()))
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
