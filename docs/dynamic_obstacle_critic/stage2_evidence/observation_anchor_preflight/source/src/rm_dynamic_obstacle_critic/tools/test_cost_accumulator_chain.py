#!/usr/bin/env python3
"""Actual chain corruption probes; read-only source links, full changed inputs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from verify_cost_accumulator_chain import verify


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('optimizer',type=Path)
    parser.add_argument('risk',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    source={}
    for folder in (args.trial,args.optimizer):
        for p in sorted(folder.rglob('*')):
            if p.is_file():source[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    source[str(args.risk)]=hashlib.sha256(args.risk.read_bytes()).hexdigest()
    marker=b',"weights_reaggregate_bounded_mean_bit_exact"'
    for name in ('cost_delta_rational','after_cost_ulp','after_fail_flag','risk_double_ulp','weights_probe','uniform_probe','control_comparison'):
        case=args.output/name;case.mkdir();trial=case/'trial';optimizer=case/'optimizer';trial.mkdir();optimizer.mkdir()
        for p in args.trial.iterdir():(trial/p.name).symlink_to(p.resolve(),target_is_directory=p.is_dir())
        for p in args.optimizer.iterdir():(optimizer/p.name).symlink_to(p.resolve(),target_is_directory=p.is_dir())
        risk=args.risk
        def change_json(path,change):
            data=json.loads(path.read_text());change(data);path.unlink();path.write_text(json.dumps(data,indent=2)+'\n')
        if name=='cost_delta_rational':change_json(trial/'cost_accumulator_audit.json',lambda m:m['cycles'][0]['exact_delta_rational'][0].__setitem__(0,m['cycles'][0]['exact_delta_rational'][0][0]+1))
        if name in ('after_cost_ulp','after_fail_flag'):
            folder=trial/'cost_after';folder.unlink();folder.mkdir()
            for p in (args.trial/'cost_after').iterdir():(folder/p.name).symlink_to(p.resolve(),target_is_directory=p.is_dir())
            if name=='after_fail_flag':change_json(folder/'cost_0.json',lambda m:m.update(fail_flag=True))
            else:
                meta=json.loads((folder/'cost_0.json').read_text());offset=next(b['offset'] for b in meta['blocks'] if b['name']=='cumulative_critic_costs')
                path=folder/'cost_0.bin';payload=bytearray(path.read_bytes());value=struct.unpack_from('<I',payload,offset)[0];struct.pack_into('<I',payload,offset,value+1)
                path.unlink();path.write_bytes(payload)
        if name=='risk_double_ulp':
            rows=args.risk.read_text().splitlines();first=json.loads(rows[0]);first['risk'][0]=math.nextafter(first['risk'][0],math.inf)
            rows[0]=json.dumps(first,separators=(',',':'));risk=case/'risk.jsonl';risk.write_text('\n'.join(rows)+'\n')
        if name in ('weights_probe','uniform_probe'):
            path=optimizer/('weights_output.jsonl' if name=='weights_probe' else 'uniform_output.jsonl')
            lines=path.read_bytes().splitlines(keepends=True);prefix,suffix=lines[1].split(marker,1)
            tail=json.loads(b'{"weights_reaggregate_bounded_mean_bit_exact"'+suffix)
            tail['weight_probe']='uniform_negative' if name=='weights_probe' else 'native'
            encoded=json.dumps(tail,separators=(',',':')).encode()
            lines[1]=prefix+b','+encoded[1:]+b'\n';path.unlink();path.write_bytes(b''.join(lines))
        if name=='control_comparison':change_json(optimizer/'positive_comparison.json',lambda m:m.update(outputs_double_bit_exact=m['cycles']-1))
        try:verify(trial,optimizer,risk)
        except ValueError as error:results.append({'name':name,'verdict':'REJECTED','reason':str(error)})
        else:raise AssertionError('corruption accepted '+name)
        changed={str(p.relative_to(case)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(case.rglob('*')) if p.is_file() and not p.is_symlink()}
        (case/'changed_input_identity.json').write_text(json.dumps(changed,indent=2)+'\n')
    for name,digest in source.items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise ValueError('source evidence changed during probes')
    result={'verdict':'PASS','checks':len(results),'results':results,'source_files_unchanged':len(source),
        'scope':'Seven actual chain corruption probes. Sources are read-only links; complete mutated files and identities are retained.'}
    (args.output/'source_identity.json').write_text(json.dumps(source,indent=2)+'\n')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
