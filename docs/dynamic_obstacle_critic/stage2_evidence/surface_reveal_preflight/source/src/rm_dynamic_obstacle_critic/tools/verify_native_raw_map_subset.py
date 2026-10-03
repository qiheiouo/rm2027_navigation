#!/usr/bin/env python3
"""Exact polygon/reserve check of three named paths and all seven discrepancies."""
import argparse
import json
from pathlib import Path
from native_snapshot_io import read_snapshot
from prepare_native_raw_map_queries import build_queries
from verify_native_factor_raw import verify_record


ROWS=[2,41,42,73,82,127,160,161,185,244]


def verify(source,witness,raw_archive,report):
    source=source.resolve();_,identity=build_queries(source,witness,raw_archive)
    if report['schema']!='native_raw_map_contract_v1' or report['native_source_ordinal']!=298 or report['raw_rows']!=300 or len(report['rows'])!=300 or report['input_sha256']!=identity['input_sha256'] or report['reference_manifests']!=identity['reference_manifests']:
        raise ValueError('original full raw query report/provenance required')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    record={'rows':len(ROWS),'model_dt':native['model_dt'],'initial_yaw':identity['initial_yaw'],
        'blocks':{k:{'values':[v for row in ROWS for v in blocks[k]['values'][row*30:(row+1)*30]]} for k in ['x','y','yaw']}}
    subset={'native_source_ordinal':298,'epoch_summaries':report['epoch_summaries'],'rows':[]}
    for index in ROWS:
        row=report['rows'][index]
        if row['raw_native_row']!=index:raise ValueError('registered original raw subset row join')
        subset['rows'].append({'factor':f'original_raw_{index}','full_conditional_labels':{epoch:{
            'raw_interval_margin_lower':labels['raw203_conservative_vector_interval_lower_m'],
            'gates':{'frozen_native_raw203_with_interval_reserve':labels['raw203_conservative_vector_gate_pass']}}
            for epoch,labels in row['epochs'].items()}})
    result=verify_record(source,record,subset)
    for epoch,values in result['epochs'].items():
        for index,row in zip(ROWS,values['rows']):row['original_raw_row']=index
    result['verified_original_raw_rows']=ROWS
    result['selection']='Original named raw73/127/185 plus all seven primary vector-fail/SDK-below203 rows; no control choice or all300 exact-cell claim.'
    result['scope']='Independent exact whole polygon/blocked-cell/interval reserve for ten selected original raw paths only; all300 vector labels remain conservative, not SG/full control acceptance.'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_archive','report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite original raw subset proof')
    result=verify(args.source,args.witness,args.raw_archive,json.loads(args.report.read_text()))
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'verdict':result['verdict'],'raw_rows':ROWS,'epochs':result['epochs']}),flush=True)
