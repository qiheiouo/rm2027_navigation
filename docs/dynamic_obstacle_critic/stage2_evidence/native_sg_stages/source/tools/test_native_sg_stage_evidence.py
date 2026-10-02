#!/usr/bin/env python3
"""Reject coordinated stage/control/score confusion and forged raw passage."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import struct
import sys
from native_sg_stage_io import read_paths
from verify_native_factor_raw import verify_record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','aggregation','prepared','paths','native_json','score_json','report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();args.output.mkdir();checks={}
    def reject(label,prepared,paths,native,score,reason):
        log=args.output/(label+'.log');outputs=[args.output/(label+suffix) for suffix in ['.report.json','.scalar.json','.raw.json']]
        with log.open('x') as stream:
            code=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('analyze_native_sg_stages.py')),str(args.source),str(args.aggregation),str(prepared),str(paths),str(native),str(score),*map(str,outputs)],stdout=stream,stderr=subprocess.STDOUT).returncode
        checks[label+'_rejected_before_report']=code!=0 and reason in log.read_text() and not any(p.exists() for p in outputs)
    wrong=args.output/'coordinated_swapped_paths.bin';data=bytearray(args.paths.read_bytes())
    # Swap before/after control rows and update the private hash. The join to
    # frozen native mean/SG still must reject, even when metadata agrees.
    data[36:156],data[156:276]=data[156:276],data[36:156];wrong.write_bytes(data)
    prepared=args.output/'coordinated_prepared';shutil.copytree(args.prepared,prepared)
    identity=json.loads((prepared/'input_identity.json').read_text());identity['stage_paths_sha256']=hashlib.sha256(data).hexdigest()
    (prepared/'input_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    reject('coordinated_stage_swap',prepared,wrong,args.native_json,args.score_json,'stage control tensor join mismatch')
    native=[json.loads(l) for l in args.native_json.read_text().splitlines()];native[1]['mean_before_SG'][0]+=.01
    wrong_json=args.output/'changed_before_mean.jsonl';wrong_json.write_text(''.join(json.dumps(r)+'\n' for r in native))
    reject('changed_before_mean',args.prepared,args.paths,wrong_json,args.score_json,'original native context and factor controls')
    costs=[json.loads(l) for l in args.score_json.read_text().splitlines()]
    target=next(r for r in costs[1:] if r['factor']=='actual_baseline' and r['path_kind']=='processed' and r['row']==1)
    bits=struct.unpack('<Q',struct.pack('<d',target['risk']))[0];target['risk']=struct.unpack('<d',struct.pack('<Q',bits+1))[0]
    changed=args.output/'changed_after_score.jsonl';changed.write_text(''.join(json.dumps(r)+'\n' for r in costs))
    reject('one_ULP_after_score',args.prepared,args.paths,args.native_json,changed,'raw/four after scores changed')
    duplicate=args.output/'duplicate_score.jsonl';lines=args.score_json.read_bytes().splitlines(keepends=True);lines[-1]=lines[-2];duplicate.write_bytes(b''.join(lines))
    reject('duplicate_score',args.prepared,args.paths,args.native_json,duplicate,'complete unique stage scores required')
    report=json.loads(args.report.read_text());report['rows'][6]['full_conditional_labels']['actual_score_clock']['raw_interval_margin_lower']=.01
    report['rows'][6]['full_conditional_labels']['actual_score_clock']['gates']['frozen_native_raw203_with_interval_reserve']=True
    forged=args.output/'forged_before_raw_pass.json';forged.write_text(json.dumps(report,indent=2)+'\n')
    try:verify_record(args.source,read_paths(args.paths),report)
    except ValueError as error:
        checks['forged_before_raw_pass_rejected']=str(error)=='vector raw conservative lower exceeds independent exact cell bound'
        (args.output/'forged_before_raw_pass.log').write_text(str(error)+'\n')
    else:checks['forged_before_raw_pass_rejected']=False
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Integrity probes for private stage joins and independent raw203 labels; no gate weakening.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
