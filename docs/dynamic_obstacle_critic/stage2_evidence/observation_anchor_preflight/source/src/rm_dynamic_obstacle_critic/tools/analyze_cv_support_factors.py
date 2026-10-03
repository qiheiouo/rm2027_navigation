#!/usr/bin/env python3
"""Compare explicit factor interventions against unchanged same-path labels."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from analyze_native_weights import contents
from prepare_processed_cv import verify


def analyze(source, raw_archive, processed_archive, prepared, cpp):
    for root in [source, raw_archive, processed_archive]:
        verify(root)
    identity = json.loads((prepared/'input_identity.json').read_text())
    if identity['source_manifest_sha256'] != hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest():
        raise ValueError('factor source manifest mismatch')
    for name, key in [('actual_input.bin', 'actual_input_sha256'), ('factor_input.bin', 'factor_input_sha256')]:
        if hashlib.sha256((prepared/name).read_bytes()).hexdigest() != identity[key]:
            raise ValueError('factor prepared identity mismatch')
    raw = json.loads((raw_archive/'analysis.json').read_text())
    processed = json.loads((processed_archive/'analysis.json').read_text())
    if raw['native_source_ordinal']!=298 or processed['native_source_ordinal']!=298:
        raise ValueError('registered fixed physical label source required')
    output = [json.loads(line) for line in cpp.read_text().splitlines()]
    header, entries = output[0], output[1:]
    names = [row['name'] for row in identity['factors']]
    if (header['scope'] != 'offline factor interventions, no public/actual oracle events' or
            header['source_ordinal'] != 298 or header['raw_rows'] != 300 or header['processed_rows'] != 311 or
            header['model_dt'] != identity['model_dt'] or header['steps'] != 30 or
            header['actual_raw_risk_bit_exact_rows'] != 300 or header['actual_raw_after_bit_exact_rows'] != 300 or
            len(entries) != 4*611 or len(names) != 4):
        raise ValueError('factor CPP full format/context required')
    indexed = {(r['factor'],r['path_kind'],r['row']):r for r in entries}
    expected = {(name,kind,row) for name in names for kind,count in [('raw',300),('processed',311)] for row in range(count)}
    if set(indexed)!=expected or len(indexed)!=len(entries):
        raise ValueError('factor complete unique rows required')
    for row in entries:
        if (not math.isfinite(row['risk']) or row['risk']<0 or row['minimum_clearance'] is None or
                not math.isfinite(row['minimum_clearance']) or
                row['collision_time'] is not None and not 0<row['collision_time']<=30*identity['model_dt']):
            raise ValueError('factor cost/geometry finite range')
    margin = raw['model_margin']
    if processed['model_margin'] != margin:
        raise ValueError('factor baseline margin mismatch')
    def same(a,b):
        return a is None and b is None or a is not None and b is not None and struct.pack('<d',a)==struct.pack('<d',b)
    for kind, labels in [('raw',raw['rows']),('processed',processed['rows'])]:
        for i, label in enumerate(labels):
            baseline = indexed[('actual_baseline',kind,i)]
            fields = [('risk','actual_dynamic_risk'),('minimum_clearance','actual_model_minimum_clearance'),('collision_time','actual_model_collision_time')] if kind=='raw' else [('risk','CV_risk'),('minimum_clearance','CV_minimum_clearance'),('collision_time','CV_collision_time')]
            if any(not same(baseline[new],label[old]) for new,old in fields):
                raise ValueError('factor baseline does not reproduce prior exact costs')
    summaries = {}
    rows = []
    epoch = 'actual_score_clock'
    for factor in names:
        summaries[factor] = {}
        for kind, labels in [('raw',raw['rows']),('processed',processed['rows'])]:
            result = []
            for i,label in enumerate(labels):
                values = indexed[(factor,kind,i)]
                model_clear = values['minimum_clearance']>margin
                dynamic = label['physical_dynamic_labels'][epoch]
                contact = dynamic['geometry']['padded_native']['original_rollout_grid_min']==0
                row = {**values, 'model_margin_clear':model_clear, 'physical_dynamic_gates_clear':dynamic['all_dynamic_geometry_gates'],
                       'physical_padded_grid_contact':contact}
                if kind=='raw':
                    row['original_native_weight'] = label['native_weight']
                else:
                    row['processed_name'] = label['name'];row['processed_kind'] = label['kind']
                rows.append(row);result.append(row)
            subset = [r for r in result if r['model_margin_clear'] and r['physical_padded_grid_contact']]
            summaries[factor][kind] = {'rows':len(result),'model_margin_clear_rows':sum(r['model_margin_clear'] for r in result),
                'model_clear_but_physical_dynamic_gates_failed':sum(r['model_margin_clear'] and not r['physical_dynamic_gates_clear'] for r in result),
                'model_clear_but_physical_padded_grid_contact':len(subset)}
            if kind=='raw':
                mass = math.fsum(r['original_native_weight'] for r in subset)
                summaries[factor][kind]['unchanged_original_weight_mass_on_model_clear_grid_contacts'] = mass
                summaries[factor][kind]['unchanged_original_weight_fraction_on_model_clear_grid_contacts'] = mass/math.fsum(r['native_weight'] for r in labels)
            if factor=='future_circle_oracle' and subset:
                raise ValueError('future containing circle failed to detect a same-grid true padded contact')
    registered = [{**indexed[(name,'processed',i)], 'processed_name':processed['rows'][i]['name'],
                   'physical_dynamic_labels':processed['rows'][i]['physical_dynamic_labels'][epoch]} for name in names for i in range(11)]
    return {'schema':1,'source_native_ordinal':298,'associated_actor_id':identity['associated_track_id'],
            'other_usable_tracks':identity['other_usable_tracks'],'input_identity_sha256':hashlib.sha256((prepared/'input_identity.json').read_bytes()).hexdigest(),
            'cpp_output_sha256':hashlib.sha256(cpp.read_bytes()).hexdigest(),'physical_label_epoch':identity['score_clock'],
            'source_geometry_epoch':identity['source_stamp'],'source_truth_bracket':identity['source_velocity_bracket'],
            'model_margin':margin,'factors':summaries,'registered_proposals':registered,'rows':rows,
            'scope':'Offline factor interventions on identical original raw/SDK processed paths; original weights and physical labels stay fixed.',
            'limits':['Source bracket slope is finite-sample truth, not exact instantaneous speed or a physical maximum.',
                      'Future circle is an inadmissible truth oracle, never a CV/CA predictor or an online input.',
                      'Original weights are descriptive only; no new softmax, control selection or execution.',
                      'Same-grid padded contacts are a dynamic subset; full physical/task/raw203/age FAILED and NOT ESTABLISHED gates remain.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','raw_archive','processed_archive','prepared','cpp_output','output']:
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    report=analyze(args.source.resolve(),args.raw_archive.resolve(),args.processed_archive.resolve(),args.prepared,args.cpp_output)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['registered_proposals','rows']}),flush=True)
    print(json.dumps({'aggregate_and_zero':[r for r in report['registered_proposals'] if r['row'] in [0,1]]}),flush=True)
