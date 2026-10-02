#!/usr/bin/env python3
"""Actual executable baseline and IEEE corruption checks under SDK FP flags."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--binary',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir();checks={}
    def run(source,binary,json_path,label):
        with (args.output/(label+'.log')).open('x') as log:
            return subprocess.run([str(args.binary),str(source),str(binary),str(json_path)],stdout=log,stderr=subprocess.STDOUT).returncode
    binary=args.output/'positive_paths.bin';json_path=args.output/'positive.jsonl'
    code=run(args.input,binary,json_path,'positive')
    if code!=0:raise ValueError('native factor positive failed')
    data=args.input.read_bytes();blob=binary.read_bytes()
    rows=[json.loads(line) for line in json_path.read_text().splitlines()]
    header=struct.unpack_from('<IIIIfd',blob,8)
    names=['actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle']
    checks['complete_private_four_path_output']=(len(blob)==4356 and blob[:8]==b'RMCVAGO1' and header[:4]==(298,4,30,127) and
        len(rows)==5 and [r['factor'] for r in rows[1:]]==names and rows[0]['native_library']=='/opt/ros/humble/lib/libmppi_controller.so')
    baseline=rows[1]
    checks['independent_baseline_gamma_weights_mean_SG_command_bits']=(
        struct.pack('<300f',*baseline['post_gamma_costs'])==data[330120:331320] and
        struct.pack('<300f',*baseline['native_softmax_weights'])==data[331320:332520] and
        struct.pack('<90f',*baseline['mean_before_SG'])==data[576:936] and
        struct.pack('<90f',*baseline['mean_after_SG'])==data[936:1296] and
        struct.pack('<3d',*baseline['returned_control'])==data[1296:1320])
    checks['independent_baseline_nine_path_blocks_bits']=all(blob[36+axis*480:36+axis*480+120]==data[332520+axis*120:332520+(axis+1)*120] for axis in range(9))
    checks['all_four_native_weights_reaggregate_exact']=all(r['weights_reaggregate_bounded_mean_bit_exact'] for r in rows[1:])
    def mutation(offset,value,fmt):
        changed=bytearray(data);struct.pack_into(fmt,changed,offset,value);return changed
    def delta(offset,fmt,change):
        return mutation(offset,struct.unpack_from(fmt,data,offset)[0]+change,fmt)
    probes=[('schema',b'INVALID1'+data[8:],'hypothesis schema'),
        ('source',mutation(8,297,'<I'),'source/grid/count'),('batch',mutation(12,301,'<I'),'source/grid/count'),
        ('steps',mutation(16,29,'<I'),'source/grid/count'),('factor_count',mutation(20,5,'<I'),'source/grid/count'),
        ('dt',mutation(24,.09,'<f'),'settings invalid'),
        ('NaN_settings',mutation(36,float('nan'),'<f'),'nonfinite native factor field'),
        ('Inf_measured_double',mutation(64,float('inf'),'<d'),'nonfinite native factor field'),
        ('NaN_history',mutation(168,float('nan'),'<f'),'nonfinite native factor field'),
        ('NaN_controls',mutation(1320,float('nan'),'<f'),'nonfinite native factor field'),
        ('Inf_raw_model',mutation(109320,float('inf'),'<f'),'nonfinite native factor field'),
        ('NaN_cost',mutation(325320,float('nan'),'<f'),'nonfinite native factor field'),
        ('Inf_weights',mutation(331320,float('inf'),'<f'),'nonfinite native factor field'),
        ('changed_actual_raw_model',delta(109320,'<f',.01),'actual raw native model bits mismatch'),
        ('changed_actual_cost',delta(325320,'<f',.25),'actual gamma/weights/mean/SG/command bits mismatch'),
        ('changed_actual_gamma',delta(330120,'<f',.25),'actual gamma/weights/mean/SG/command bits mismatch'),
        ('changed_warm',delta(216,'<f',.05),'actual gamma/weights/mean/SG/command bits mismatch'),
        ('changed_history',delta(168,'<f',.05),'actual gamma/weights/mean/SG/command bits mismatch'),
        ('changed_command',delta(1296,'<d',.001),'actual gamma/weights/mean/SG/command bits mismatch'),
        ('changed_aggregate_path',delta(333240,'<f',.01),'actual aggregate path bits mismatch'),
        ('truncated',data[:-1],'exact byte budget'),('trailing',data+b'x','exact byte budget')]
    for name,probe,reason in probes:
        source=args.output/(name+'.bin');source.write_bytes(probe)
        b=args.output/(name+'.paths.bin');j=args.output/(name+'.jsonl')
        code=run(source,b,j,name)
        checks[name+'_rejected_before_output']=code==1 and reason in (args.output/(name+'.log')).read_text() and not b.exists() and not j.exists()
    digest=hashlib.sha256(blob).hexdigest();digest_json=hashlib.sha256(json_path.read_bytes()).hexdigest()
    fresh_json=args.output/'existing_binary_new.jsonl'
    code=run(args.input,binary,fresh_json,'existing_binary')
    checks['existing_binary_preserved']=code==1 and hashlib.sha256(binary.read_bytes()).hexdigest()==digest and not fresh_json.exists()
    fresh_binary=args.output/'existing_json_new.bin'
    code=run(args.input,fresh_binary,json_path,'existing_json')
    checks['existing_json_preserved']=code==1 and hashlib.sha256(json_path.read_bytes()).hexdigest()==digest_json and not fresh_binary.exists()
    report={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
            'native_factor_ELF_sha256':hashlib.sha256(args.binary.read_bytes()).hexdigest(),
            'positive_paths_sha256':digest,'positive_jsonl_sha256':digest_json,
            'scope':'Fixed actual baseline and private hypotheses; bitwise NaN/Inf rejection under SDK fast-math; no physical acceptance.'}
    (args.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True);return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
