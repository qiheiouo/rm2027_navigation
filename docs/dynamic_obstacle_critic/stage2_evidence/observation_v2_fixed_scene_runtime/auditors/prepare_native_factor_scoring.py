#!/usr/bin/env python3
"""Keep frozen raw scoring context while explicitly querying four aggregates."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
from native_factor_io import read_paths
from prepare_processed_cv import verify


def prepare(factors,native_paths,output):
    verify(factors);record=read_paths(native_paths)
    data=bytearray(gzip.open(factors/'prepared/factor_input.bin.gz','rb').read())
    old=json.loads((factors/'prepared/input_identity.json').read_text())
    if hashlib.sha256(data).hexdigest()!=old['factor_input_sha256'] or record['model_dt']!=old['model_dt']:
        raise ValueError('frozen factor input/time identity')
    struct.pack_into('<I',data,12,4)
    data=data[:708]
    for name in ['x','y','yaw']:data.extend(struct.pack('<120f',*record['blocks'][name]['values']))
    if len(data)!=2148:raise ValueError('four-aggregate factor exact budget')
    actual=gzip.open(factors/'prepared/actual_input.bin.gz','rb').read()
    if hashlib.sha256(actual).hexdigest()!=old['actual_input_sha256']:raise ValueError('actual raw context identity')
    output.mkdir();(output/'actual_input.bin').write_bytes(actual);(output/'factor_input.bin').write_bytes(data)
    result={'schema':1,'source_ordinal':298,'grid':[4,30],'score_clock':old['score_clock'],'source_age':old['source_age_used'],
            'native_paths_sha256':hashlib.sha256(native_paths.read_bytes()).hexdigest(),
            'actual_input_sha256':hashlib.sha256(actual).hexdigest(),'factor_input_sha256':hashlib.sha256(data).hexdigest(),
            'factor_archive_manifest_sha256':hashlib.sha256((factors/'manifest.json').read_bytes()).hexdigest(),
            'row_factors':[f['name'] for f in old['factors']],
            'scope':'Four explicitly offline native aggregation paths; row0 actual baseline replica, rows1-3 counterfactuals.',
            'limits':'Private four-row hypothesis, not the original 311-path witness or an actual critic event; no online oracle or deployment acceptance.'}
    (output/'input_identity.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['factors','native_paths','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();print(json.dumps(prepare(args.factors.resolve(),args.native_paths,args.output)),flush=True)
