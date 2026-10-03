#!/usr/bin/env python3
"""Compiled raw-query format probes with lossless reproducible mutation recipes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
from native_raw_map_io import read_input
from verify_native_raw_map_queries import verify


def run_tests(executable,prepared,cpp,old_format,output):
    output.mkdir();blob=(prepared/'queries.bin').read_bytes();base_sha=hashlib.sha256(blob).hexdigest()
    checks={'positive_independent_scalar':verify(prepared,cpp)['verdict']=='PASS'}
    def invoke(source,destination,name):
        result=subprocess.run([str(executable),str(source),str(destination)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (output/(name+'.log')).write_bytes(result.stdout);return result
    positive=output/'positive.jsonl';result=invoke(prepared/'queries.bin',positive,'positive')
    checks['actual_native_output_byte_exact']=result.returncode==0 and positive.read_bytes()==cpp.read_bytes()
    def patch(offset,delete,insert):
        data=blob[:offset]+insert+blob[offset+delete:]
        return data,{'mode':'binary_patch','base_sha256':base_sha,'offset':offset,'delete_bytes':delete,'insert_hex':insert.hex(),'mutated_input_sha256':hashlib.sha256(data).hexdigest()}
    def mutation(offset,value,fmt):return patch(offset,struct.calcsize(fmt),struct.pack(fmt,value))
    first=10140;original=10140+300*(24+208*32)
    probes=[('magic',patch(0,8,b'BADRAW01'),'private schema'),('source',mutation(8,297,'<I'),'source/map/trace budget'),
        ('width',mutation(12,99,'<I'),'source/map/trace budget'),('height',mutation(16,101,'<I'),'source/map/trace budget'),
        ('footprint_count',mutation(20,3,'<I'),'source/map/trace budget'),('traces',mutation(24,600,'<I'),'source/map/trace budget'),
        ('queries',mutation(28,124500,'<I'),'source/map/trace budget'),('epochs',mutation(32,1,'<I'),'source/map/trace budget'),
        ('steps',mutation(36,29,'<I'),'source/map/trace budget'),('dt',mutation(40,.09,'<f'),'time/resolution mismatch'),
        ('NaN_dt',mutation(40,float('nan'),'<f'),'time/resolution mismatch'),('resolution',mutation(44,.04,'<d'),'time/resolution mismatch'),
        ('NaN_origin',mutation(52,float('nan'),'<d'),'nonfinite native map query'),('horizon',mutation(68,2.9,'<d'),'time/resolution mismatch'),
        ('Inf_footprint',mutation(76,float('inf'),'<d'),'nonfinite native map query'),
        ('wrong_dense_kind',mutation(first,3,'<I'),'trace row/epoch identity'),('wrong_row',mutation(first+4,1,'<I'),'trace row/epoch identity'),
        ('wrong_epoch',mutation(first+8,1,'<I'),'trace row/epoch identity'),('short_dense_count',mutation(first+12,207,'<I'),'trace row/epoch identity'),
        ('wrong_start',mutation(first+16,47.98,'<d'),'trace row/epoch identity'),('NaN_pose',mutation(first+32,float('nan'),'<d'),'nonfinite native map query'),
        ('Inf_time',mutation(first+24,float('inf'),'<d'),'nonfinite native map query'),('duplicate_time',mutation(first+24+32,0,'<d'),'full monotonic grid'),
        ('first_dense_time',mutation(first+24,.001,'<d'),'complete native/dense grid required'),
        ('short_dense_horizon',mutation(first+24+207*32,2.9995,'<d'),'complete native/dense grid required'),
        ('wrong_original_kind',mutation(original,2,'<I'),'trace row/epoch identity'),('wrong_original_count',mutation(original+12,208,'<I'),'trace row/epoch identity'),
        ('original_initial_zero',mutation(original+24,0.,'<d'),'complete native/dense grid required'),
        ('original_second_time',mutation(original+24+32,.201,'<d'),'original30 time grid required'),
        ('truncated',patch(len(blob)-1,1,b''),'exact byte budget'),('trailing',patch(len(blob),0,b'x'),'exact byte budget')]
    other=old_format.read_bytes();probes.append(('old_stage_format',(other,{'mode':'frozen_other_format','schema':'RMCOSTQ1','input_sha256':hashlib.sha256(other).hexdigest()}),'exact byte budget'))
    for name,(data,recipe),reason in probes:
        path=output/(name+'.bin');path.write_bytes(data);(output/(name+'.mutation.json')).write_text(json.dumps(recipe,indent=2)+'\n')
        result=invoke(path,output/(name+'.jsonl'),name)
        checks[name+'_native_rejected']=result.returncode!=0 and reason.encode() in result.stdout and not (output/(name+'.jsonl')).exists()
        try:read_input(path)
        except ValueError as error:
            checks[name+'_reader_rejected']=True;(output/(name+'.reader.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_reader_rejected']=False
    before=positive.read_bytes();result=invoke(prepared/'queries.bin',positive,'existing_output')
    checks['existing_output_protected']=result.returncode!=0 and b'refusing to overwrite' in result.stdout and positive.read_bytes()==before
    rows=[json.loads(line) for line in cpp.read_text().splitlines()];cpp_sha=hashlib.sha256(cpp.read_bytes()).hexdigest()
    def output_probe(name,line,replacement,exception):
        changed=list(rows);changed[line]=replacement;path=output/(name+'.jsonl')
        path.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in changed))
        recipe={'mode':'replace_JSON_row_then_normalize_all_lines','base_CPP_sha256':cpp_sha,'line_index':line,'replacement':replacement,'mutated_CPP_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        (output/(name+'.mutation.json')).write_text(json.dumps(recipe,indent=2)+'\n')
        try:result=verify(prepared,path)
        except (ValueError,KeyError,StopIteration) as error:
            checks[name+'_rejected']=exception;(output/(name+'.verification.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:
            checks[name+'_rejected']=not exception and result['verdict']=='FAILED';(output/(name+'.verification.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    output_probe('claimed_actual_critic',0,{**rows[0],'actual_CostCritic_state':'known'},True)
    changed=json.loads(json.dumps(rows[1]));changed['pose'][0]=math.nextafter(changed['pose'][0],math.inf);output_probe('changed_pose',1,changed,True)
    output_probe('duplicate_query',2,rows[1],True)
    output_probe('boolean_row',1,{**rows[1],'path_row':False},True)
    output_probe('changed_boundary_cost',1,{**rows[1],'native_footprint_cost':rows[1]['native_footprint_cost']+1},False)
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'mutation_storage':'Original prepared/native bytes plus archived lossless recipes and mutated SHA256 reconstruct every retained probe input/output; full ephemeral files also retained in trial directory.',
        'scope':'Compiled original raw30/dense schema and independent query integrity, not complete critic state, continuous clearance or execution.'}
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['executable','prepared','cpp','old_format','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=run_tests(*[getattr(args,n) for n in ['executable','prepared','cpp','old_format','output']])
    print(json.dumps(result),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
