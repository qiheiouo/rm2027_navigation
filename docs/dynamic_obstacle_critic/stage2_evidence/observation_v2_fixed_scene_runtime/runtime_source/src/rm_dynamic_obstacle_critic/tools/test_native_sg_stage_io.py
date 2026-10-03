#!/usr/bin/env python3
"""Reject stage/factor format confusion and incomplete stage baseline proofs."""
import argparse
import json
from pathlib import Path
import struct
from native_sg_stage_io import read_paths


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['paths','four_paths','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();args.output.mkdir();blob=args.paths.read_bytes();record=read_paths(args.paths)
    checks={'genuine_eight_stage_output':record['rows']==8 and len(record['row_identity'])==8}
    def mutation(offset,value,fmt):
        data=bytearray(blob);struct.pack_into(fmt,data,offset,value);return data
    probes=[('four_factor_schema',args.four_paths.read_bytes()),('fake_factor_magic',b'RMCVAGO1'+blob[8:]),
        ('source',mutation(8,297,'<I')),('four_rows',mutation(12,4,'<I')),('steps',mutation(16,29,'<I')),
        ('missing_baseline',mutation(20,63,'<I')),('dt',mutation(24,.09,'<f')),
        ('nonfinite_yaw',mutation(28,float('nan'),'<d')),('nonfinite_matrix',mutation(36,float('inf'),'<f')),
        ('truncated',blob[:-1]),('trailing',blob+b'x')]
    for name,data in probes:
        path=args.output/(name+'.bin');path.write_bytes(data)
        try:read_paths(path)
        except ValueError as error:
            checks[name+'_rejected']=True;(args.output/(name+'.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_rejected']=False
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Eight-stage private output identity; not a capture type or before-SG safety certificate.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
