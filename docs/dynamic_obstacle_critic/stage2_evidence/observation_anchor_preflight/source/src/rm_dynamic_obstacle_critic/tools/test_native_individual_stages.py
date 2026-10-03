#!/usr/bin/env python3
"""Actual native private-input failures and original-stage association probes."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
from analyze_native_individual_stages import context
from native_individual_stage_io import read_stages


def tests(executable,source,witness,prepared,legacy_binary,legacy_json,stage_file,output):
    output.mkdir();blob=(prepared/'input.bin').read_bytes();stage=stage_file.read_bytes();checks={}
    context(source,witness,prepared,legacy_binary,legacy_json,stage_file);checks['complete_original_context']=True
    def invoke(path,label,binary=None,js=None,stages=None):
        binary=binary or output/(label+'.witness.bin');js=js or output/(label+'.jsonl');stages=stages or output/(label+'.stages.bin')
        result=subprocess.run([str(executable),str(path),str(binary),str(js),str(stages),'--filtered-sampler'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (output/(label+'.log')).write_bytes(result.stdout);return result,binary,js,stages
    result,binary,js,stages=invoke(prepared/'input.bin','positive')
    checks['actual_stage_byte_exact']=result.returncode==0 and stages.read_bytes()==stage
    checks['actual_legacy_binary_byte_exact']=binary.read_bytes()==legacy_binary.read_bytes()
    checks['actual_legacy_JSON_byte_exact']=js.read_bytes()==legacy_json.read_bytes()
    def patch(data,offset,value,fmt):
        b=bytearray(data);struct.pack_into(fmt,b,offset,value);return bytes(b)
    probes=[('magic',b'BADWIT01'+blob[8:],'witness schema'),('count',patch(blob,8,2,'<I'),'fixed source grid'),
        ('batch',patch(blob,12,299,'<I'),'fixed source grid'),('steps',patch(blob,16,29,'<I'),'fixed source grid'),
        ('dt',patch(blob,20,.2,'<f'),'registered constraints'),('changed_vx_max',patch(blob,28,1.,'<f'),'registered constraints'),
        ('NaN_dt',patch(blob,20,float('nan'),'<f'),'nonfinite'),('Inf_limit',patch(blob,36,float('inf'),'<f'),'nonfinite'),
        ('ordinal',patch(blob,40,1,'<I'),'ordinal gap'),('NaN_pose',patch(blob,44,float('nan'),'<d'),'nonfinite'),
        ('Inf_speed',patch(blob,100,float('inf'),'<d'),'nonfinite'),('NaN_history',patch(blob,148,float('nan'),'<f'),'nonfinite'),
        ('Inf_mean',patch(blob,196,float('inf'),'<f'),'nonfinite'),('NaN_command',patch(blob,916,float('nan'),'<d'),'nonfinite'),
        ('Inf_control',patch(blob,940,float('inf'),'<f'),'nonfinite'),('NaN_reference',patch(blob,108940,float('nan'),'<f'),'nonfinite'),
        ('changed_raw_reference',patch(blob,108940,0.,'<f'),'baseline bits mismatch'),
        ('truncated',blob[:-1],'exact input budget'),('trailing',blob+b'x','exact input budget'),('old_stage_format',stage,'exact input budget')]
    for label,data,reason in probes:
        path=output/(label+'.input.bin');path.write_bytes(data)
        result,_,_,stages=invoke(path,label)
        checks[label+'_native_rejected']=result.returncode!=0 and reason.encode() in result.stdout
        try:read_stages(stages)
        except (OSError,ValueError):checks[label+'_no_valid_stage']=True
        else:checks[label+'_no_valid_stage']=False
    before={p:p.read_bytes() for p in [binary,js,output/'positive.stages.bin']}
    result,*_=invoke(prepared/'input.bin','existing_stage',binary,js,output/'positive.stages.bin')
    checks['existing_stage_protected']=result.returncode!=0 and b'refusing to overwrite stage' in result.stdout and all(p.read_bytes()==v for p,v in before.items())
    destination=output/'should_not_create.stages.bin';result,*_=invoke(prepared/'input.bin','existing_legacy',binary,js,destination)
    checks['existing_legacy_protected']=result.returncode!=0 and b'refusing to overwrite witness' in result.stdout and not destination.exists() and all(p.read_bytes()==v for p,v in before.items())
    stage_probes=[('old_schema',b'RMSFWTO1'+stage[8:]),('stage_truncated',stage[:-1]),('stage_trailing',stage+b'x'),
        ('stage_count',patch(stage,8,2,'<I')),('stage_batch',patch(stage,12,301,'<I')),('stage_steps',patch(stage,16,29,'<I')),
        ('stage_dt',patch(stage,20,float('nan'),'<f')),('stage_ordinal',patch(stage,24,1,'<I')),('stage_mask',patch(stage,28,62,'<I')),
        ('stage_kind',patch(stage,32,1,'<I')),('stage_rows',patch(stage,36,311,'<I')),
        ('before_kind',patch(stage,324040,2,'<I')),('after_rows',patch(stage,659932,300,'<I')),
        ('nonfinite_stage_tensor',patch(stage,40,float('inf'),'<f'))]
    for label,data in stage_probes:
        path=output/(label+'.stages.bin');path.write_bytes(data)
        try:read_stages(path)
        except ValueError as error:
            checks[label+'_reader_rejected']=True;(output/(label+'.reader.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[label+'_reader_rejected']=False
    # Finite tensor changes remain valid schemas but must fail their source joins.
    for label,offset in [('changed_raw_tensor',40),('changed_afterSG_tensor',659936),('changed_before_measured_prefix',436008)]:
        path=output/(label+'.stages.bin');path.write_bytes(patch(stage,offset,9.,'<f'))
        try:context(source,witness,prepared,legacy_binary,legacy_json,path)
        except ValueError as error:
            checks[label+'_context_rejected']=True;(output/(label+'.rejection.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[label+'_context_rejected']=False
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'input_sha256':hashlib.sha256(blob).hexdigest(),'stage_sha256':hashlib.sha256(stage).hexdigest(),
        'scope':'Actual compiled private-input rejection, bitwise baseline and bounded output/source associations. No exact all300 cell or execution certificate.'}
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['executable','source','witness','prepared','legacy_binary','legacy_json','stage_file','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=tests(*[getattr(args,n) for n in ['executable','source','witness','prepared','legacy_binary','legacy_json','stage_file','output']])
    print(json.dumps(result),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
