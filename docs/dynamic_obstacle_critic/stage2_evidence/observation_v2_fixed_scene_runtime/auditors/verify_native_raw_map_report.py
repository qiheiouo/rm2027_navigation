#!/usr/bin/env python3
"""Exact risk/cost/weight joins and rational partitions, separate from geometry."""
import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import struct
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot,COST
from prepare_native_raw_map_queries import build_queries
from verify_native_raw_map_subset import ROWS
from verify_native_raw_map_queries import verify as verify_queries


def verify(source,witness,raw_archive,prepared,cpp,report,subset):
    source,witness,raw_archive=[p.resolve() for p in [source,witness,raw_archive]]
    blob,identity=build_queries(source,witness,raw_archive)
    if (prepared/'queries.bin').read_bytes()!=blob or json.loads((prepared/'input_identity.json').read_text())!=identity or report['schema']!='native_raw_map_contract_v1' or report['reference_manifests']!=identity['reference_manifests'] or report['input_sha256']!=identity['input_sha256'] or report['native_query_sha256']!=hashlib.sha256(cpp.read_bytes()).hexdigest() or report['native_source_ordinal']!=298 or report['raw_rows']!=300 or report['query_count']!=142500 or report['trace_count']!=1200 or report['original_raw30_pose_promotion_exact_queries']!=18000 or report['dense_unwrapped_yaw_changes'] or report['scalar_mismatch_count']!=0 or report['actual_CostCritic_full_state']!='unknown':raise ValueError('complete same raw query report identity required')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json');_,used=read_consumption(source/'dynamic_scores/score_298.json')
    old=json.loads((raw_archive/'analysis.json').read_text());errors=[]
    scalar=verify_queries(prepared,cpp)
    if scalar['verdict']!='PASS':raise ValueError('native query scalar proof required for report stats')
    if len(report['rows'])!=300 or subset['verdict']!='PASS' or subset['verified_original_raw_rows']!=ROWS:raise ValueError('complete300 rows and exact ten-row subset required')
    for index,row in enumerate(report['rows']):
        original=old['rows'][index]
        if type(row['raw_native_row']) is not int or row['raw_native_row']!=index or row['raw_path_is_an_accepted_SG_control'] is not False or row['actual_CostCritic_individual_increment_or_collision']!='unknown':raise ValueError('raw row/SG/actual critic scope changed')
        for key in ['actual_native_weight','actual_dynamic_risk','actual_model_minimum_clearance']:
            expected=original['native_weight' if key=='actual_native_weight' else key]
            if struct.pack('<d',row[key])!=struct.pack('<d',expected):raise ValueError('original same-path weight/risk/minimum join changed')
        for key,block in [('original_seven_and_StaticStopping_cost_sum_before_dynamic','costs_before_dynamic'),('original_after_dynamic_cost','costs_after_dynamic')]:
            if struct.pack('<f',row[key])!=struct.pack('<f',used[block]['values'][index]) or not math.isfinite(row[key]):raise ValueError('actual original cost sum changed')
        if struct.pack('<f',row['original_seven_and_StaticStopping_cost_sum_before_dynamic']+row['actual_dynamic_risk'])!=struct.pack('<f',row['original_after_dynamic_cost']) or struct.pack('<f',row['original_after_dynamic_cost'])!=blocks[COST]['bytes'][index*4:(index+1)*4]:raise ValueError('original double risk addition / f32 native cost changed')
        for ei,epoch in enumerate(['actual_score_clock','source_pose_clock_sensitivity']):
            labels=row['epochs'][epoch];a=labels['dense_SDK_queries'];b=labels['original_raw30_SDK_queries'];lower=labels['raw203_conservative_vector_interval_lower_m']
            if a!=scalar['traces'][ei*600+index] or b!=scalar['traces'][ei*600+300+index]:raise ValueError('original CPP query statistics changed')
            if not math.isfinite(lower) or type(labels['raw203_conservative_vector_gate_pass']) is not bool or labels['raw203_conservative_vector_gate_pass']!=(lower>1e-9) or labels['frozen_same_raw_physical_dynamic_labels']!=original['physical_dynamic_labels'][epoch]:raise ValueError('separate original physical/raw vector labels changed')
            if a['path_kind']!=2 or b['path_kind']!=3 or a['path_row']!=index or b['path_row']!=index or a['epoch_index']!=ei or b['epoch_index']!=ei or a['trace']!=ei*600+index or b['trace']!=ei*600+300+index or a['samples']!=(208 if ei==0 else 207) or b['samples']!=30:raise ValueError('separate raw30/dense trace identity changed')
    for epoch,summary in report['epoch_summaries'].items():
        expected=old['epoch_summaries'][epoch]
        if any(summary[key]!=value for key,value in expected.items()) or len(summary['partition'])!=4:raise ValueError('original epoch/physical summary changed')
        for p,(passed,below) in zip(summary['partition'],[(False,False),(False,True),(True,False),(True,True)]):
            group=[r for r in report['rows'] if r['epochs'][epoch]['raw203_conservative_vector_gate_pass']==passed and (r['epochs'][epoch]['dense_SDK_queries']['maximum_native_footprint_cost']<203)==below]
            exact=float(sum((Fraction.from_float(r['actual_native_weight']) for r in group),Fraction()))
            errors.append(abs(p['actual_weight_mass']-exact))
            if p['vector_raw203_pass']!=passed or p['dense_boundary_samples_below203']!=below or p['rows']!=len(group) or p['actual_weight_mass']!=exact:raise ValueError('exact rational raw weight partition changed')
        if summary['whole_polygon_vector_pass_rows']!=sum(r['epochs'][epoch]['raw203_conservative_vector_gate_pass'] for r in report['rows']) or summary['raw30_and_dense_ge203_classification_differences']!=[r['raw_native_row'] for r in report['rows'] if (r['epochs'][epoch]['dense_SDK_queries']['maximum_native_footprint_cost']>=203)!=(r['epochs'][epoch]['original_raw30_SDK_queries']['maximum_native_footprint_cost']>=203)]:raise ValueError('raw map summary count changed')
        for index,r in zip(ROWS,subset['epochs'][epoch]['rows']):
            label=report['rows'][index]['epochs'][epoch]
            if r['original_raw_row']!=index or r['captured_conservative_margin_m']!=label['raw203_conservative_vector_interval_lower_m'] or r['raw_gate_pass']!=label['raw203_conservative_vector_gate_pass']:raise ValueError('independent subset/vector identity changed')
    return {'verdict':'PASS','original_same_path_rows':300,'original_double_risk_f32_cost_additions_exact':300,
        'rational_partition_count':8,'maximum_weight_mass_error':max(errors),'exact_cell_subset_raw_rows':ROWS,
        'scope':'Independent original bits, rational mass and registered geometry subset joins; no actual CostCritic increment or all300 exact-cell/SG acceptance claim.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_archive','prepared','cpp','report','subset','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite raw report verification')
    result=verify(args.source,args.witness,args.raw_archive,args.prepared,args.cpp,json.loads(args.report.read_text()),json.loads(args.subset.read_text()))
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result),flush=True)
