#!/usr/bin/env python3
"""Keep four-row private factor output distinct from old witness/capture data."""
import argparse
import json
from pathlib import Path
import struct
from native_factor_io import read_paths


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();args.output.mkdir();original=args.paths.read_bytes();record=read_paths(args.paths)
    checks={'genuine_four_row_private_output':record['rows']==4 and record['actual_baseline_bit_exact_mask']==127}
    def mutation(offset,value,fmt):
        blob=bytearray(original);struct.pack_into(fmt,blob,offset,value);return blob
    for name,blob in [('old_witness_schema',b'RMSFWTO1'+original[8:]),('source',mutation(8,297,'<I')),
                     ('old_311_rows',mutation(12,311,'<I')),('incomplete_actual_baseline',mutation(20,63,'<I')),
                     ('dt',mutation(24,.09,'<f')),('nonfinite_yaw',mutation(28,float('nan'),'<d')),
                     ('nonfinite_matrix',mutation(36,float('inf'),'<f')),('truncated',original[:-1]),('trailing',original+b'x')]:
        path=args.output/(name+'.bin');path.write_bytes(blob)
        try:read_paths(path)
        except ValueError as error:
            checks[name+'_rejected']=True
            (args.output/(name+'.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_rejected']=False
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
            'scope':'Strict private four-row result integrity; original actual/capture/witness formats remain distinct.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
