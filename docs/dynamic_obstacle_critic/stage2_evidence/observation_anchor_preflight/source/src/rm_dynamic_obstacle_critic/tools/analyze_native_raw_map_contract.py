#!/usr/bin/env python3
"""Same original raw paths: SDK map queries, vector reserve and frozen CV labels."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import numpy as np
from dynamic_consumption_io import read_consumption
from native_raw_map_io import read_input
from native_snapshot_io import read_snapshot,COST
from prepare_native_raw_map_queries import build_queries
from verify_native_raw_map_queries import verify
from witness_geometry import raw_clearance_lower,travel


EPOCHS=['actual_score_clock','source_pose_clock_sensitivity']


def analyze(source,witness,raw_archive,prepared,cpp):
    source,witness,raw_archive=[p.resolve() for p in [source,witness,raw_archive]]
    blob,identity=build_queries(source,witness,raw_archive)
    if (prepared/'queries.bin').read_bytes()!=blob or json.loads((prepared/'input_identity.json').read_text())!=identity:
        raise ValueError('original raw30/dense model poses and full provenance must be regenerated exactly')
    context=read_input(prepared/'queries.bin');scalar=verify(prepared,cpp)
    if scalar['verdict']!='PASS':raise ValueError('complete independent raw grid queries required')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json');consumed,used=read_consumption(source/'dynamic_scores/score_298.json')
    if consumed['score_stamp']!=48.004 or consumed['world_transform']!=[0,0,0] or consumed['evaluation_frame']!='odom' or consumed['model_dt']!=context['dt'] or consumed['pose']!=native['pose'] or consumed['speed']!=native['speed'] or consumed['padded_footprint']!=native['padded_footprint'] or any(used[k]['bytes']!=blocks[k]['bytes'] for k in ['x','y','yaw']) or used['costs_after_dynamic']['bytes']!=blocks[COST]['bytes']:
        raise ValueError('original actually consumed raw model and after-cost bytes must join native source298')
    old=json.loads((raw_archive/'analysis.json').read_text());policy=json.loads((source/'policy.json').read_text())
    if policy['raw_costmap_threshold']!=203:raise ValueError('original raw203 policy required')
    footprint=np.asarray(context['footprint']);radius=np.linalg.norm(footprint,axis=1).max();lower={};knots_exact=0;yaw_offsets=[]
    for ei,epoch in enumerate(EPOCHS):
        dense=context['traces'][ei*600:ei*600+300];original=context['traces'][ei*600+300:(ei+1)*600]
        poses=np.array([np.array(r['poses'])[:,1:] for r in dense]);movement=travel(poses[:,:-1],poses[:,1:],radius)
        reserve=np.zeros(poses.shape[:2]);reserve[:,:-1]=.5*movement;reserve[:,1:]=np.maximum(reserve[:,1:],.5*movement)
        grid={**{k:native['map'][k] for k in ['resolution','origin','size']},'data':context['raw']}
        lower[epoch]=(raw_clearance_lower(poses,footprint,grid,reserve,203)-reserve).min(axis=1)
        # The original queries must preserve source f32 bits promoted to f64;
        # dense yaw is a separately unwrapped representation.
        for row,trace in enumerate(original):
            for step,p in enumerate(trace['poses']):
                values=[blocks[k]['values'][row*30+step] for k in ['x','y','yaw']]
                if struct.pack('<3d',*p[1:])!=struct.pack('<3d',*values):raise ValueError('original raw30 pose promotion changed')
                knots_exact+=1
                knot=old['epoch_summaries'][epoch]['epoch']+(step+1)*context['dt']
                index=next(i for i,q in enumerate(dense[row]['poses']) if q[0]==knot-trace['start'])
                q=dense[row]['poses'][index]
                if q[1:3]!=p[1:3]:raise ValueError('dense original knot x/y must match captured raw exactly')
                if q[3]!=p[3]:yaw_offsets.append({'epoch':epoch,'raw_row':row,'step':step,'original_yaw':p[3],'dense_unwrapped_yaw':q[3]})
    rows=[]
    for row,legacy in enumerate(old['rows']):
        if legacy['raw_native_row']!=row or len(old['rows'])!=300 or struct.pack('<d',legacy['actual_dynamic_risk'])!=struct.pack('<d',used['dynamic_risk_double']['values'][row]):raise ValueError('original same-path risk/physical row join changed')
        labels={}
        for ei,epoch in enumerate(EPOCHS):
            a,b=scalar['traces'][ei*600+row],scalar['traces'][ei*600+300+row]
            if a['path_row']!=row or b['path_row']!=row or a['path_kind']!=2 or b['path_kind']!=3:raise ValueError('raw30/dense scalar trace join changed')
            labels[epoch]={'dense_SDK_queries':a,'original_raw30_SDK_queries':b,
                'raw203_conservative_vector_interval_lower_m':float(lower[epoch][row]),
                'raw203_conservative_vector_gate_pass':bool(lower[epoch][row]>1e-9),
                'frozen_same_raw_physical_dynamic_labels':legacy['physical_dynamic_labels'][epoch]}
        rows.append({'raw_native_row':row,'actual_native_weight':legacy['native_weight'],
            'original_seven_and_StaticStopping_cost_sum_before_dynamic':used['costs_before_dynamic']['values'][row],
            'actual_dynamic_risk':legacy['actual_dynamic_risk'],'actual_model_minimum_clearance':legacy['actual_model_minimum_clearance'],
            'actual_model_margin_clear':legacy['actual_model_margin_clear'],'actual_model_collision_time':legacy['actual_model_collision_time'],
            'original_after_dynamic_cost':used['costs_after_dynamic']['values'][row],'epochs':labels,
            'actual_CostCritic_individual_increment_or_collision':'unknown','raw_path_is_an_accepted_SG_control':False})
    summaries={}
    for epoch in EPOCHS:
        summaries[epoch]={**old['epoch_summaries'][epoch],
            'whole_polygon_vector_pass_rows':sum(r['epochs'][epoch]['raw203_conservative_vector_gate_pass'] for r in rows),
            'partition':[{'vector_raw203_pass':passed,'dense_boundary_samples_below203':below,
                'rows':sum(r['epochs'][epoch]['raw203_conservative_vector_gate_pass']==passed and (r['epochs'][epoch]['dense_SDK_queries']['maximum_native_footprint_cost']<203)==below for r in rows),
                'actual_weight_mass':math.fsum(r['actual_native_weight'] for r in rows if r['epochs'][epoch]['raw203_conservative_vector_gate_pass']==passed and (r['epochs'][epoch]['dense_SDK_queries']['maximum_native_footprint_cost']<203)==below)} for passed in [False,True] for below in [False,True]],
            'raw30_and_dense_ge203_classification_differences':[r['raw_native_row'] for r in rows if (r['epochs'][epoch]['dense_SDK_queries']['maximum_native_footprint_cost']>=203)!=(r['epochs'][epoch]['original_raw30_SDK_queries']['maximum_native_footprint_cost']>=203)]}
    return {'schema':'native_raw_map_contract_v1','native_source_ordinal':298,'raw_rows':300,'query_count':142500,'trace_count':1200,
        'input_sha256':identity['input_sha256'],'native_query_sha256':hashlib.sha256(cpp.read_bytes()).hexdigest(),'reference_manifests':identity['reference_manifests'],
        'original_raw30_pose_promotion_exact_queries':knots_exact,'dense_unwrapped_yaw_changes':yaw_offsets,
        'scalar_mismatch_count':scalar['scalar_mismatch_count'],'epoch_summaries':summaries,'rows':rows,
        'actual_CostCritic_full_state':'unknown','raw_cell_source_classification':'unknown',
        'scope':'Same original300 raw paths: actually consumed CV/weight/cost sums, original raw30 versus dense installed SDK queries and conservative whole-polygon interval lower.',
        'limits':['Vector raw203 labels for all300 are conservative; no all300 exact polygon-square scalar audit is claimed.',
            'The recorded pre-Dynamic sum does not reveal any individual original critic increment or actual branch/trajectory-step selection.',
            'Raw controls are unprocessed sampled controls, not SG-safe output candidates; no final bounds/full conditional acceptance label is created.',
            'Raw30 queries preserve captured f32 poses; dense queries use unwrapped-yaw interpolation, with representation changes listed explicitly.',
            'No new costs, weights, ranking, commands, threshold, cells, runtime, TF or public interfaces.',
            'Physical support bounds unknown; original task/raw203/age FAILED and composite NOT ESTABLISHED remain.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_archive','prepared','cpp','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite original raw map report')
    result=analyze(*[getattr(args,n) for n in ['source','witness','raw_archive','prepared','cpp']])
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'queries':result['query_count'],'raw30_exact':result['original_raw30_pose_promotion_exact_queries'],'yaw_changes':len(result['dense_unwrapped_yaw_changes']),'epochs':result['epoch_summaries']}),flush=True)
