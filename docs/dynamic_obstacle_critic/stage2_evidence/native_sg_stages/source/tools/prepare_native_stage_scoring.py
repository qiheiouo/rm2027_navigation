#!/usr/bin/env python3
"""Prepare explicit eight-stage queries in unchanged source298 CV context."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
from native_factor_io import read_paths as read_factors
from native_sg_stage_io import read_paths, FACTORS, STAGES
from prepare_processed_cv import verify


def prepare(factors,aggregation,stage_paths,output):
    verify(factors);verify(aggregation)
    record=read_paths(stage_paths);old_paths=read_factors(aggregation/'native_paths.bin')
    for name in old_paths['blocks']:
        for row in range(4):
            actual=record['blocks'][name]['values'][(2*row+1)*30:(2*row+2)*30]
            frozen=old_paths['blocks'][name]['values'][row*30:(row+1)*30]
            if struct.pack('<30f',*actual)!=struct.pack('<30f',*frozen):raise ValueError('four after-SG paths changed')
    data=bytearray(gzip.open(factors/'prepared/factor_input.bin.gz','rb').read())
    old=json.loads((factors/'prepared/input_identity.json').read_text())
    if hashlib.sha256(data).hexdigest()!=old['factor_input_sha256'] or record['model_dt']!=old['model_dt']:
        raise ValueError('frozen factor input/time identity')
    struct.pack_into('<I',data,12,8);data=data[:708]
    for name in ['x','y','yaw']:data.extend(struct.pack('<240f',*record['blocks'][name]['values']))
    if len(data)!=3588:raise ValueError('eight-stage factor exact budget')
    actual=gzip.open(factors/'prepared/actual_input.bin.gz','rb').read()
    if hashlib.sha256(actual).hexdigest()!=old['actual_input_sha256']:raise ValueError('actual context identity')
    result={'schema':1,'source_ordinal':298,'grid':[8,30],'score_clock':old['score_clock'],'source_age':old['source_age_used'],
        'four_after_paths_all_nine_blocks_bit_exact':True,'row_identity':[{'factor':f,'stage':s} for f in FACTORS for s in STAGES],
        'stage_paths_sha256':hashlib.sha256(stage_paths.read_bytes()).hexdigest(),
        'actual_input_sha256':hashlib.sha256(actual).hexdigest(),'factor_input_sha256':hashlib.sha256(data).hexdigest(),
        'factor_manifest_sha256':hashlib.sha256((factors/'manifest.json').read_bytes()).hexdigest(),
        'aggregation_manifest_sha256':hashlib.sha256((aggregation/'manifest.json').read_bytes()).hexdigest(),
        'scope':'Eight explicit stage hypotheses; before-SG paths are not issued native controls or actual capture events.'}
    output.mkdir();(output/'actual_input.bin').write_bytes(actual);(output/'factor_input.bin').write_bytes(data)
    (output/'input_identity.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['factors','aggregation','stage_paths','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();print(json.dumps(prepare(args.factors.resolve(),args.aggregation.resolve(),args.stage_paths,args.output)),flush=True)
