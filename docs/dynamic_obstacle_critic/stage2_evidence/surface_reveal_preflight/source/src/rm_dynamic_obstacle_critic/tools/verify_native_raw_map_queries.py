#!/usr/bin/env python3
"""Independent scalar grid costs on exact raw30 and dense model queries."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from native_raw_map_io import read_input
from verify_native_cost_map_queries import scalar_cost


def verify(prepared,cpp):
    context=read_input(prepared/'queries.bin');identity=json.loads((prepared/'input_identity.json').read_text())
    if identity['schema']!='native_raw_map_query_input_v1' or identity['source_ordinal']!=298 or identity['trace_count']!=1200 or identity['query_count']!=142500 or identity['model_dt']!=context['dt'] or identity['horizon']!=context['horizon'] or identity['map']['size']!=context['size'] or identity['map']['resolution']!=context['resolution'] or identity['map']['origin']!=context['origin'] or identity['padded_footprint']!=[list(p) for p in context['footprint']] or identity['actual_CostCritic_full_state']!='unknown':raise ValueError('complete original raw map identity metadata required')
    if hashlib.sha256((prepared/'queries.bin').read_bytes()).hexdigest()!=identity['input_sha256'] or hashlib.sha256(context['raw']).hexdigest()!=identity['map_raw_bytes_sha256']:raise ValueError('raw query/map private identities changed')
    traces=[];errors=[]
    with cpp.open() as stream:
        header=json.loads(next(stream))
        if header['schema']!='native_raw_map_queries_v1' or header['source_ordinal']!=298 or header['trace_count']!=1200 or header['query_count']!=142500 or header['actual_CostCritic_state']!='unknown' or header['native_library']!='/opt/ros/humble/lib/libnav2_costmap_2d_core.so':raise ValueError('complete installed raw checker output required')
        for trace in context['traces']:
            rows=[]
            for sample,p in enumerate(trace['poses']):
                row=json.loads(next(stream))
                if any(type(row[k]) is not int for k in ['trace','path_kind','path_row','epoch_index','sample']) or any(row[k]!=trace[k] for k in ['trace','path_kind','path_row','epoch_index']) or row['sample']!=sample or isinstance(row['future_seconds'],bool) or struct.pack('<4d',row['future_seconds'],*row['pose'])!=struct.pack('<4d',*p):raise ValueError('raw private pose/time/row join changed')
                center,fp=scalar_cost(context,p[1:]);rows.append(row)
                if type(row['center_in_map']) is not bool or type(row['center_raw']) is not int or isinstance(row['native_footprint_cost'],bool) or row['center_in_map']!=(center>=0) or row['center_raw']!=center or row['native_footprint_cost']!=fp:
                    errors.append({'trace':trace['trace'],'sample':sample,'native_center':row['center_raw'],'scalar_center':center,'native_footprint':row['native_footprint_cost'],'scalar_footprint':fp})
            traces.append({'trace':trace['trace'],'path_kind':trace['path_kind'],'path_row':trace['path_row'],'epoch_index':trace['epoch_index'],'samples':len(rows),
                'maximum_center_raw':max(r['center_raw'] for r in rows),'center_samples_ge203':sum(r['center_raw']>=203 for r in rows),'center_samples_outside_map':sum(not r['center_in_map'] for r in rows),
                'maximum_native_footprint_cost':max(r['native_footprint_cost'] for r in rows),
                'native_footprint_samples_ge203':sum(r['native_footprint_cost']>=203 for r in rows),'native_footprint_samples_eq254':sum(r['native_footprint_cost']==254 for r in rows),
                'native_footprint_samples_eq255':sum(r['native_footprint_cost']==255 for r in rows),
                'first_native_footprint_ge203':next((r for r in rows if r['native_footprint_cost']>=203),None)})
        if stream.read():raise ValueError('raw query output trailing rows/data')
    return {'verdict':'PASS' if not errors else 'FAILED','query_count':142500,'trace_count':1200,'scalar_mismatch_count':len(errors),
        'scalar_mismatches':errors,'traces':traces,'actual_CostCritic_full_state':'unknown',
        'scope':'Exact installed center/boundary costs on separate original raw30 and dense queries; not whole polygon, continuous safety or actual critic scoring/classification.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['prepared','cpp','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite raw scalar verification')
    result=verify(args.prepared,args.cpp)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'verdict':result['verdict'],'queries':result['query_count'],'mismatches':result['scalar_mismatch_count']}),flush=True)
    raise SystemExit(0 if result['verdict']=='PASS' else 1)
