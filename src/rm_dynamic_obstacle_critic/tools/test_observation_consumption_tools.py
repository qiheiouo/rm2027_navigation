#!/usr/bin/env python3
"""Execute v2 CPP-fixture reader/prepare/risk/processed tools and corruptions.

Processed fixture paths repeat actual raw paths solely to check protocol and
arithmetic. They are not SG outputs or a physical future-path witness.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
from dynamic_consumption_io import read_consumption
from prepare_dynamic_consumption_replay import prepare


def main():
    parser=argparse.ArgumentParser();parser.add_argument('build',type=Path);parser.add_argument('new_output',type=Path)
    args=parser.parse_args();args.new_output.mkdir(parents=True,exist_ok=False);out=args.new_output
    fixtures=Path(__file__).resolve().parents[1]/'test/fixtures';checks={};outputs={}
    def run(executable, arguments, label, code=0, reason=None):
        with (out/(label+'.log')).open('x') as log:
            result=subprocess.run([str(args.build/executable),*map(str,arguments)],stdout=log,stderr=subprocess.STDOUT)
        checks[label]=result.returncode==code and (reason is None or reason in (out/(label+'.log')).read_text())
        if not checks[label]:raise RuntimeError('unexpected helper result: '+label)

    legacy=out/'v1';prepare(fixtures/'dynamic_consumption/MovingInputWithCaptureAndBudgetExhaustionIsBitExact',legacy)
    for label in ['visible','nominal']:
        source=fixtures/'observation_consumption'/label;prepared=out/label;identity=prepare(source,prepared)
        meta,blocks=read_consumption(source/'score_0.json');batch,steps=blocks['x']['shape']
        risk=prepared/'risk.jsonl';run('replay_observation_consumption',[prepared/'input.bin',risk],label+'_risk')
        data=[json.loads(line) for line in risk.read_text().splitlines()]
        checks[label+'_raw_risk_after_bits']=len(data)==1 and data[0]['risk_double_bit_exact_rows']==batch and data[0]['after_float_bit_exact_rows']==batch
        paths=prepared/'processed_paths.bin'
        paths.write_bytes(b'RMDYNPX1'+struct.pack('<IIIf',meta['ordinal'],batch,steps,meta['model_dt'])+
                          b''.join(blocks[name]['bytes'] for name in ['x','y','yaw']))
        processed=prepared/'processed.jsonl'
        run('evaluate_observation_processed_cv',[prepared/'input.bin',paths,processed],label+'_processed')
        rows=[json.loads(line) for line in processed.read_text().splitlines()]
        checks[label+'_processed_repeated_raw_arithmetic']=rows[0]['actual_raw_risk_bit_exact_rows']==batch and len(rows)==batch+1 and (
            b''.join(struct.pack('<d',row['risk']) for row in rows[1:])==blocks['dynamic_risk_double']['bytes'])
        run('evaluate_processed_cv',[prepared/'input.bin',paths,prepared/'old_processed.jsonl'],label+'_old_processed_refuses_v2',1,'dynamic replay schema')
        outputs[label]=dict(input_sha256=identity['input_sha256'],risk_sha256=hashlib.sha256(risk.read_bytes()).hexdigest(),
                            processed_sha256=hashlib.sha256(processed.read_bytes()).hexdigest(),batch=batch,steps=steps)

    blob=(out/'nominal/input.bin').read_bytes();probes=[]
    run('replay_dynamic_consumption',[out/'nominal/input.bin',out/'old_refuses_v2.jsonl'],'old_refuses_v2',1,'dynamic replay schema')
    run('replay_observation_consumption',[legacy/'input.bin',out/'new_refuses_v1.jsonl'],'new_refuses_v1',1,'observation replay schema')
    def mutation(offset,value,fmt):
        data=bytearray(blob);struct.pack_into(fmt,data,offset,value);return bytes(data)
    probes += [('unknown_radius_enum',mutation(112,2,'<I'),1,'observation replay geometry budget'),
               ('half_diameter',mutation(116,.4,'<d'),2,None),
               ('oversize_prior',mutation(124,4097,'<I'),1,'observation replay geometry budget'),
               ('truncated_input',blob[:-1],1,'truncated dynamic replay'),
               ('trailing_input',blob+b'x',1,'trailing dynamic replay')]
    for label,content,code,reason in probes:
        path=out/(label+'.bin');path.write_bytes(content)
        run('replay_observation_consumption',[path,out/(label+'.jsonl')],label,code,reason)
    nominal_meta,nominal_blocks=read_consumption(fixtures/'observation_consumption/nominal/score_0.json')
    batch,steps=nominal_blocks['x']['shape'];tracks=len(nominal_meta['input_used']['tracks']);points=len(nominal_meta['padded_footprint'])
    prior=len(nominal_meta['limits']['geometry_prior_source'].encode())
    risk_offset=12+20+80+16+prior+16*points+57*tracks+3*batch*steps*4+batch*4
    risk_bits=struct.unpack_from('<Q',blob,risk_offset)[0];changed=mutation(risk_offset,risk_bits+1,'<Q')
    path=out/'one_risk_ulp.bin';path.write_bytes(changed)
    run('evaluate_observation_processed_cv',[path,out/'nominal/processed_paths.bin',out/'one_risk_ulp.jsonl'],
        'processed_risk_ulp_rejected',1,'actual raw risk/after bits mismatch')
    report=dict(status='PASS' if all(checks.values()) else 'FAILED',checks=checks,outputs=outputs,
        helper_sha256={name:hashlib.sha256((args.build/name).read_bytes()).hexdigest() for name in
            ['replay_dynamic_consumption','replay_observation_consumption','evaluate_processed_cv','evaluate_observation_processed_cv']},
        scope='CPP-fixture protocol/arithmetic only; processed fixture paths repeat raw paths, no SG or physical acceptance.')
    (out/'summary.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n');print(json.dumps(report,sort_keys=True))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
