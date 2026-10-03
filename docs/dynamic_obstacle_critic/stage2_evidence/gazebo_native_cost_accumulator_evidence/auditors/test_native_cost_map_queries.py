#!/usr/bin/env python3
"""Actual installed-helper input failures and independent output-integrity probes."""
import argparse
import json
import math
from pathlib import Path
import struct
import subprocess
from native_cost_map_io import read_input
from verify_native_cost_map_queries import verify,verify_synthetic


def run_tests(executable,prepared,cpp,synthetic,other_formats,output):
    output.mkdir();blob=(prepared/'queries.bin').read_bytes();read_input(prepared/'queries.bin')
    checks={'independent_all_queries':verify(prepared,cpp)['verdict']=='PASS','synthetic_fixture':verify_synthetic(synthetic)['verdict']=='PASS'}
    def invoke(source,destination,label):
        result=subprocess.run([str(executable),str(source),str(destination)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (output/(label+'.log')).write_bytes(result.stdout)
        return result
    positive=output/'positive.jsonl';result=invoke(prepared/'queries.bin',positive,'positive')
    checks['actual_native_positive_byte_exact']=result.returncode==0 and positive.read_bytes()==cpp.read_bytes()
    actual_synthetic=output/'synthetic.jsonl';result=invoke('--synthetic',actual_synthetic,'synthetic')
    checks['actual_native_synthetic_byte_exact']=result.returncode==0 and actual_synthetic.read_bytes()==synthetic.read_bytes()
    def mutation(offset,value,fmt):
        data=bytearray(blob);struct.pack_into(fmt,data,offset,value);return data
    # Trace 0 is 208 points. Metadata starts at 10140; the first pose at 10164.
    individual=10140+8*(24+208*32);second_epoch=10140+51*(24+208*32)
    probes=[('magic',b'BADMAP01'+blob[8:],'private schema'),('source',mutation(8,297,'<I'),'source/map/trace budget'),
        ('width',mutation(12,99,'<I'),'source/map/trace budget'),('height',mutation(16,101,'<I'),'source/map/trace budget'),
        ('footprint_count',mutation(20,3,'<I'),'source/map/trace budget'),('trace_count',mutation(24,101,'<I'),'source/map/trace budget'),
        ('query_count',mutation(28,21164,'<I'),'source/map/trace budget'),('epoch_count',mutation(32,1,'<I'),'source/map/trace budget'),
        ('steps',mutation(36,29,'<I'),'source/map/trace budget'),('dt',mutation(40,.09,'<f'),'time/resolution mismatch'),
        ('NaN_dt',mutation(40,float('nan'),'<f'),'time/resolution mismatch'),('resolution',mutation(44,.04,'<d'),'time/resolution mismatch'),
        ('horizon',mutation(68,2.9,'<d'),'time/resolution mismatch'),('NaN_origin',mutation(52,float('nan'),'<d'),'nonfinite native map query'),
        ('Inf_footprint',mutation(76,float('inf'),'<d'),'nonfinite native map query'),('trace_kind',mutation(10140,1,'<I'),'trace row/epoch identity'),
        ('trace_row',mutation(10144,1,'<I'),'trace row/epoch identity'),('trace_epoch',mutation(10148,1,'<I'),'trace row/epoch identity'),
        ('trace_samples',mutation(10152,207,'<I'),'trace row/epoch identity'),('trace_start',mutation(10156,47.98,'<d'),'trace row/epoch identity'),
        ('NaN_pose',mutation(10172,float('nan'),'<d'),'nonfinite native map query'),('Inf_time',mutation(10164,float('inf'),'<d'),'nonfinite native map query'),
        ('first_time',mutation(10164,.001,'<d'),'complete horizon required'),('duplicate_time',mutation(10196,0,'<d'),'full monotonic grid'),
        ('short_horizon',mutation(10164+207*32,2.9995,'<d'),'complete horizon required'),
        ('individual_order',mutation(individual+24+208*32+4,0,'<I'),'individual order/duplicate'),
        ('epoch_row_join',mutation(second_epoch+4,1,'<I'),'trace row/epoch identity'),
        ('epoch_individual_join',mutation(second_epoch+8*(24+207*32)+4,1,'<I'),'epoch row mapping changed'),
        ('truncated',blob[:-1],'exact byte budget'),('trailing',blob+b'x','exact byte budget')]
    for i,path in enumerate(other_formats):probes.append((f'other_format_{i}',path.read_bytes(),'exact byte budget'))
    for name,data,reason in probes:
        path=output/(name+'.bin');path.write_bytes(data);destination=output/(name+'.jsonl');result=invoke(path,destination,name)
        checks[name+'_native_rejected']=result.returncode!=0 and reason.encode() in result.stdout and not destination.exists()
        try:read_input(path)
        except ValueError as error:
            checks[name+'_reader_rejected']=True;(output/(name+'.reader.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_reader_rejected']=False
    for source,destination,name in [(prepared/'queries.bin',positive,'existing_output'),('--synthetic',actual_synthetic,'existing_synthetic')]:
        before=destination.read_bytes();result=invoke(source,destination,name)
        checks[name+'_protected']=result.returncode!=0 and b'refusing to overwrite' in result.stdout and destination.read_bytes()==before
    data=[json.loads(line) for line in cpp.read_text().splitlines()]
    def output_probe(name,mutator,expect_exception):
        rows=json.loads(json.dumps(data));mutator(rows);path=output/(name+'.jsonl')
        path.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
        try:result=verify(prepared,path)
        except (ValueError,KeyError) as error:
            checks[name+'_rejected']=expect_exception;(output/(name+'.verification.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:
            checks[name+'_rejected']=not expect_exception and result['verdict']=='FAILED'
            (output/(name+'.verification.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    output_probe('wrong_source',lambda r:r[0].update(source_ordinal=297),True)
    output_probe('claimed_actual_critic',lambda r:r[0].update(actual_CostCritic_state='known'),True)
    output_probe('changed_pose',lambda r:r[1]['pose'].__setitem__(0,math.nextafter(r[1]['pose'][0],math.inf)),True)
    output_probe('duplicate_row',lambda r:r.__setitem__(2,r[1]),True)
    output_probe('missing_row',lambda r:r.pop(),True)
    output_probe('changed_center',lambda r:r[1].update(center_raw=r[1]['center_raw']+1),False)
    output_probe('changed_native_cost',lambda r:r[1].update(native_footprint_cost=r[1]['native_footprint_cost']+1),False)
    output_probe('wrong_boolean_type',lambda r:r[1].update(center_in_map=1),False)
    identity=json.loads((prepared/'input_identity.json').read_text());tampered=output/'identity_changed';tampered.mkdir()
    (tampered/'queries.bin').write_bytes(blob);identity['padded_footprint'][0][0]=math.nextafter(identity['padded_footprint'][0][0],math.inf)
    (tampered/'input_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    try:verify(tampered,cpp)
    except ValueError as error:
        checks['identity_footprint_rejected']=True;(output/'identity_changed.json').write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
    else:checks['identity_footprint_rejected']=False
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Compiled SDK-helper validation, exact private input and independent scalar query integrity; no actual CostCritic safety certificate.'}
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['executable','prepared','cpp','synthetic','factor_format','stage_format','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=run_tests(args.executable,args.prepared,args.cpp,args.synthetic,[args.factor_format,args.stage_format],args.output)
    print(json.dumps(result),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
