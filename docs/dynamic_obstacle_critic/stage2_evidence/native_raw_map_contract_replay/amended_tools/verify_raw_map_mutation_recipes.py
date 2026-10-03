#!/usr/bin/env python3
"""Reconstruct archived negative inputs/outputs without copying full matrices."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def verify(archive):
    blob=(archive/'prepared/queries.bin').read_bytes();cpp=gzip.open(archive/'native_queries.jsonl.gz','rb').read()
    rows=None;checks={}
    paths=set((archive/'verification').rglob('*.mutation.json'))
    paths.update((archive/'verification').rglob('mutation.json'))
    for path in sorted(paths):
        recipe=json.loads(path.read_text());mode=recipe['mode']
        if mode=='binary_patch':
            if hashlib.sha256(blob).hexdigest()!=recipe['base_sha256']:raise ValueError('mutation base input changed')
            offset=recipe['offset'];data=blob[:offset]+bytes.fromhex(recipe['insert_hex'])+blob[offset+recipe['delete_bytes']:]
            checks[str(path.relative_to(archive))]=hashlib.sha256(data).hexdigest()==recipe['mutated_input_sha256']
        elif mode=='frozen_other_format':
            data=(archive.parent/'native_cost_map_contract/prepared/queries.bin').read_bytes()
            checks[str(path.relative_to(archive))]=recipe['schema']=='RMCOSTQ1' and hashlib.sha256(data).hexdigest()==recipe['input_sha256']
        elif mode=='replace_JSON_row_then_normalize_all_lines':
            if hashlib.sha256(cpp).hexdigest()!=recipe['base_CPP_sha256']:raise ValueError('mutation base native output changed')
            if rows is None:rows=[json.loads(line) for line in cpp.decode().splitlines()]
            changed=list(rows);changed[recipe['line_index']]=recipe['replacement']
            data=''.join(json.dumps(row,allow_nan=False)+'\n' for row in changed).encode()
            checks[str(path.relative_to(archive))]=hashlib.sha256(data).hexdigest()==recipe['mutated_CPP_sha256']
        else:raise ValueError('unknown raw map mutation recipe')
    if len(checks)!=38:raise ValueError('all32 binary, five native output and one coordinated evidence recipe required')
    return {'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'All retained raw-format negative inputs and native-output probes reconstruct exactly from frozen bases and lossless recipes.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite mutation verification')
    result=verify(args.archive)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'verdict':result['verdict'],'recipes':len(result['checks'])}),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
