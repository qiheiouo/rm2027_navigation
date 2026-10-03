#!/usr/bin/env python3
"""File-only conditional CV support evaluation; unknown bounds stay unknown."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml
from conditional_cv_support import conditional_radius, validate_assumptions


def evaluate(assumptions_path, query_path):
    assumptions = yaml.safe_load(assumptions_path.read_text())
    validate_assumptions(assumptions)
    query = json.loads(query_path.read_text())
    if (not isinstance(query,dict) or set(query)!= {'schema','velocity_xy','source_age_sec','future_offsets_sec','scope'} or
            type(query['schema']) is not int or query['schema']!=1 or not isinstance(query['scope'],str) or not query['scope'].strip()):
        raise ValueError('explicit query schema/scope required')
    offsets = query['future_offsets_sec']
    if not isinstance(offsets,list) or not 1<=len(offsets)<=4096:
        raise ValueError('bounded nonempty query list required')
    rows = [conditional_radius(assumptions,query['velocity_xy'],query['source_age_sec'],offset) for offset in offsets]
    return {'schema':1, 'assumptions_sha256':hashlib.sha256(assumptions_path.read_bytes()).hexdigest(),
            'query_sha256':hashlib.sha256(query_path.read_bytes()).hexdigest(),
            'query_scope':query['scope'], 'results':rows,
            'scope':'Offline caller-conditional support only; no geometry certification, online radius, command or deployment acceptance.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['assumptions','query','output']:
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError('refusing to overwrite conditional support output')
    result=evaluate(args.assumptions,args.query)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'rows':len(result['results']),'statuses':[r['status'] for r in result['results']]}),flush=True)
