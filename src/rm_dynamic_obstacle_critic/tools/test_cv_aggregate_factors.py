#!/usr/bin/env python3
"""Exercise the explicit four-path instantiation without weakening default311."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path);parser.add_argument('frozen_factors',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--binary',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir();checks={}
    actual=args.prepared/'actual_input.bin';hyp=args.prepared/'factor_input.bin';blob=hyp.read_bytes()
    def run(source,target,name):
        with (args.output/(name+'.log')).open('x') as log:
            return subprocess.run([str(args.binary),str(actual),str(source),str(target)],stdout=log,stderr=subprocess.STDOUT).returncode
    result=args.output/'scores.jsonl';code=run(hyp,result,'positive')
    if code!=0:raise ValueError('four aggregate score positive failed')
    data=[json.loads(l) for l in result.read_text().splitlines()];names=['actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle']
    indexed={(r['factor'],r['path_kind'],r['row']):r for r in data[1:]}
    checks['four_complete_300_raw_and_four_aggregate_scores']=(len(data)==1217 and set(indexed)=={(n,k,i) for n in names for k,c in [('raw',300),('processed',4)] for i in range(c)} and data[0]['actual_raw_risk_bit_exact_rows']==300 and data[0]['actual_raw_after_bit_exact_rows']==300)
    old={(r['factor'],r['path_kind'],r['row']):r for r in [json.loads(l) for l in gzip.open(args.frozen_factors/'factors.jsonl.gz','rt')][1:]}
    def same(a,b):return a is None and b is None or a is not None and b is not None and struct.pack('<d',a)==struct.pack('<d',b)
    checks['all_1200_frozen_raw_and_four_baseline_model_scores_bit_exact']=all(all(same(r[f],old[key][f]) for f in ['risk','minimum_clearance','collision_time']) for key,r in indexed.items() if key[1]=='raw' or key[2]==0)
    reduction=bytearray(blob);reduction[180:228]=reduction[36:84]
    xyvx=struct.unpack_from('<4d',blob,36);identity=json.loads((args.prepared/'input_identity.json').read_text());dt=struct.unpack_from('<f',blob,20)[0]
    for step in range(30):
        elapsed=identity['source_age']+(step+1)*dt
        struct.pack_into('<2d',reduction,228+step*16,xyvx[0]+xyvx[2]*elapsed,xyvx[1]+xyvx[3]*elapsed)
    rp=args.output/'CV_reduction.bin';rp.write_bytes(reduction);ro=args.output/'CV_reduction.jsonl';code=run(rp,ro,'CV_reduction')
    reduced=[json.loads(l) for l in ro.read_text().splitlines()][1:] if code==0 else []
    checks['future_circle_reduces_to_CV_on_all_304_paths']=code==0 and all(all(same(r[f],indexed[('actual_baseline',r['path_kind'],r['row'])][f]) for f in ['risk','minimum_clearance','collision_time']) for r in reduced if r['factor']=='future_circle_oracle') and len(reduced)==1216
    def mutation(offset,value,fmt):
        data=bytearray(blob);struct.pack_into(fmt,data,offset,value);return data
    probes=[('schema',b'INVALID1'+blob[8:],'hypothesis schema'),('source',mutation(8,297,'<I'),'source/grid/dt/count mismatch'),
        ('default311_not_accepted',mutation(12,311,'<I'),'source/grid/dt/count mismatch'),('dt',mutation(20,.09,'<f'),'source/grid/dt/count mismatch'),
        ('track_id',mutation(24,2,'<Q'),'actor id/state mismatch'),('factor_count',mutation(32,5,'<I'),'source/grid/dt/count mismatch'),
        ('nonfinite_state',mutation(36,float('nan'),'<d'),'nonfinite factor state'),('changed_baseline',mutation(36,5.,'<d'),'baseline state is not actual'),
        ('nonpositive_size',mutation(68,0.,'<d'),'physical size invalid'),('nonfinite_future',mutation(228,float('nan'),'<d'),'nonfinite oracle center'),
        ('nonfinite_path',mutation(708,float('nan'),'<f'),'nonfinite factor path'),('truncated',blob[:-1],'truncated dynamic replay'),('trailing',blob+b'x','trailing factor input')]
    for name,data,reason in probes:
        p=args.output/(name+'.bin');p.write_bytes(data);out=args.output/(name+'.jsonl');code=run(p,out,name)
        checks[name+'_rejected']=code==1 and reason in (args.output/(name+'.log')).read_text() and not out.exists()
    digest=hashlib.sha256(result.read_bytes()).hexdigest();code=run(hyp,result,'existing_output')
    checks['existing_output_preserved']=code==1 and hashlib.sha256(result.read_bytes()).hexdigest()==digest
    report={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
            'ELF_sha256':hashlib.sha256(args.binary.read_bytes()).hexdigest(),'cpp_sha256':digest,
            'scope':'Explicit four-row private hypotheses; unchanged default311 is independently regressed; no actual oracle event.'}
    (args.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
