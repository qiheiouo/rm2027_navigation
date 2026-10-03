#!/usr/bin/env python3
"""Reject a forged positive raw203 gate on the fixed future-oracle path."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','paths','report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();args.output.mkdir()
    data=json.loads(args.report.read_text());row=data['rows'][3]['full_conditional_labels']['actual_score_clock']
    if row['gates']['frozen_native_raw203_with_interval_reserve'] or row['raw_interval_margin_lower']>=0:
        raise ValueError('registered negative oracle raw203 gate changed')
    row['raw_interval_margin_lower']=.01;row['gates']['frozen_native_raw203_with_interval_reserve']=True
    forged=args.output/'forged_raw_gate.json';forged.write_text(json.dumps(data,indent=2)+'\n')
    out=args.output/'invalid_verification.json'
    with (args.output/'rejection.log').open('x') as log:
        code=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('verify_native_factor_raw.py')),str(args.source),str(args.paths),str(forged),str(out)],stdout=log,stderr=subprocess.STDOUT).returncode
    passed=code!=0 and not out.exists() and 'vector raw conservative lower exceeds independent exact cell bound' in (args.output/'rejection.log').read_text()
    result={'verdict':'PASS' if passed else 'FAILED','forged_positive_raw203_rejected':passed,
            'scope':'Independent raw-cell evidence integrity; original203 threshold and all failed gates remain.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
