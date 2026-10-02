#!/usr/bin/env python3
"""Join installed boundary queries to distinct frozen whole-footprint gates."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
from native_cost_map_io import read_input
from native_snapshot_io import read_snapshot
from prepare_processed_cv import verify as verify_manifest
from prepare_native_cost_map_queries import build_queries
from verify_native_cost_map_queries import verify,verify_synthetic


EPOCHS=['actual_score_clock','source_pose_clock_sensitivity']


def analyze(source,witness,stages,mass,prepared,cpp,synthetic):
    source,witness,stages,mass=[p.resolve() for p in [source,witness,stages,mass]]
    identity=json.loads((prepared/'input_identity.json').read_text());refs={}
    for name,root in [('source',source),('witness',witness),('stages',stages),('mass',mass)]:
        verify_manifest(root);refs[name]=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()
    if identity['reference_manifests']!=refs:raise ValueError('frozen reference manifests changed')
    expected_blob,expected_identity=build_queries(source,witness,stages,mass)
    if (prepared/'queries.bin').read_bytes()!=expected_blob or identity!=expected_identity:
        raise ValueError('original model paths and complete query provenance must be rebuilt exactly')
    context=read_input(prepared/'queries.bin');native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    if context['raw']!=blocks['raw_costmap']['values'] or context['footprint']!=[tuple(p) for p in native['padded_footprint']] or context['origin']!=native['map']['origin']:
        raise ValueError('original native map bytes/footprint/origin required')
    scalar=verify(prepared,cpp);synthetic_check=verify_synthetic(synthetic)
    if scalar['verdict']!='PASS' or synthetic_check['verdict']!='PASS':raise ValueError('independent native query/synthetic proofs required')
    stage=json.loads((stages/'analysis.json').read_text());stage_raw=json.loads((stages/'raw_scalar_verification.json').read_text())
    individuals=json.loads(gzip.open(mass/'report.json.gz','rb').read());individual_raw=json.loads(gzip.open(mass/'scalar_verification.json.gz','rb').read())
    if stage_raw['verdict']!='PASS' or individual_raw['verdict']!='PASS' or individuals['independent_raw_subset']!=identity['individual_raw_rows'] or len(identity['traces'])!=102:
        raise ValueError('original stage/43 individual scalar gates required')
    rows=[]
    for trace,meta in zip(scalar['traces'],identity['traces']):
        index=trace['path_row'];epoch=EPOCHS[trace['epoch_index']];kind=trace['path_kind']
        if meta['trace']!=trace['trace'] or meta['path_row']!=index or meta['epoch']!=epoch or meta['samples']!=trace['samples'] or meta['epoch_time']!=individuals['epoch_summaries'][epoch]['epoch'] or meta['path_kind']!=('native_stage' if kind==0 else 'individual_bounds_SG'):
            raise ValueError('query source/stage/epoch identity join changed')
        if kind==0:
            original=stage['rows'][index];full=original['full_conditional_labels'][epoch];raw=stage_raw['epochs'][epoch]['rows'][index]
            factor=original['factor'];stage_name=original['stage'];weight=None
            if meta['stage']!=stage_name or original['native_factor_row']!=index or raw['row']!=index:raise ValueError('original stage row join')
        else:
            original=individuals['rows'][index];full=original['individual_full_labels'][epoch]
            raw=next(r for r in individual_raw['epochs'][epoch]['rows'] if r['sampler_raw_row']==index)
            factor=None;stage_name='individual_actual_history_SG';weight=original['factor_weights']
            if original['sampler_raw_row']!=index or original['processed_witness_row']!=index+11 or meta['witness_row']!=index+11:raise ValueError('distinct raw/SG row join')
        raw_pass=full['gates']['frozen_native_raw203_with_interval_reserve']
        if raw_pass!=raw['raw_gate_pass']:raise ValueError('independent original raw203 gate changed')
        first=trace['first_native_footprint_ge203'];contact=raw['first_contact']
        same=first is not None and contact is not None and first['future_seconds']==contact['future_seconds'] and first['pose']==contact['pose']
        rows.append({**trace,'epoch':epoch,'epoch_time':meta['epoch_time'],'factor':factor,'stage':stage_name,
            'factor_weights_on_distinct_raw_cost_path':weight,'original_full_gates':full['gates'],
            'original_full_conditional_safe_progress':full['conditional_safe_control_with_progress'],
            'original_raw203_gate_pass':raw_pass,'original_raw203_conservative_interval_lower_m':full['raw_interval_margin_lower'],
            'independent_whole_polygon_interval_lower_m':raw['scalar_exact_raw_interval_margin_m'],
            'original_whole_polygon_sampled_contact':raw['sampled_raw_cell_or_boundary_contact'],
            'first_whole_polygon_contact':contact,'first_native203_same_pose_and_time_as_polygon_contact':same,
            'boundary_query_samples_below203':trace['maximum_native_footprint_cost']<203,
            'actual_CostCritic_score_or_collision':'unknown'})
    summaries={}
    for epoch in EPOCHS:
        summaries[epoch]={}
        for kind,name in [(0,'eight_stages'),(1,'43_individual_SG_paths')]:
            group=[r for r in rows if r['epoch']==epoch and r['path_kind']==kind]
            partitions=[{'original_raw203_gate_pass':raw_pass,'boundary_query_samples_below203':below,
                'count':sum(r['original_raw203_gate_pass']==raw_pass and r['boundary_query_samples_below203']==below for r in group)}
                for raw_pass in [False,True] for below in [False,True]]
            summaries[epoch][name]={'paths':len(group),'partition':partitions,'all_centers_raw_zero':all(r['maximum_center_raw']==0 for r in group),
                'maximum_boundary_cost':max(r['maximum_native_footprint_cost'] for r in group),
                'boundary_254_samples':sum(r['native_footprint_samples_eq254'] for r in group),
                'full_positive_labels_with_boundary203_samples':sum(r['original_full_conditional_safe_progress'] and not r['boundary_query_samples_below203'] for r in group)}
    return {'schema':'native_cost_map_contract_v1','native_source_ordinal':298,'reference_manifests':refs,
        'input_sha256':identity['input_sha256'],'native_query_sha256':hashlib.sha256(cpp.read_bytes()).hexdigest(),
        'native_synthetic_sha256':hashlib.sha256(synthetic.read_bytes()).hexdigest(),'query_count':21165,'trace_count':102,
        'horizon':context['horizon'],'scalar_mismatch_count':scalar['scalar_mismatch_count'],'synthetic_verification':synthetic_check,
        'map_raw_bytes_sha256':identity['map_raw_bytes_sha256'],'epoch_summaries':summaries,'rows':rows,
        'actual_CostCritic_full_state':'unknown','raw_map_cell_source_classification':'unknown',
        'scope':'Original whole polygon plus interval-reserve gates versus installed SDK boundary-raster queries on interpolated frozen model poses.',
        'limits':['Boundary sampled costs below203 do not certify whole polygon or continuous intervals.',
            'These 51 model paths differ from the original 300 raw trajectories scored by native critics.',
            'No actual CostCritic increment, cached inflation/layered state, branch selection, collision verdict or execution is reconstructed.',
            'critical_cost=300 is a recorded parameter; this query experiment does not establish its score semantics.',
            'No changes to threshold, cells, costs, SG, sampler, optimizer, runtime, TF or public contract.',
            'Original task/raw203/age FAILED, composite NOT ESTABLISHED and physical support bounds unknown remain.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','stages','mass','prepared','cpp','synthetic','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite native map contract report')
    result=analyze(*[getattr(args,n) for n in ['source','witness','stages','mass','prepared','cpp','synthetic']])
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'queries':result['query_count'],'scalar_mismatches':result['scalar_mismatch_count'],'epochs':result['epoch_summaries']}),flush=True)
