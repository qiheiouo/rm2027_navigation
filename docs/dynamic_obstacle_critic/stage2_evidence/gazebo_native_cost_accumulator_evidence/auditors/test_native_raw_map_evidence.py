#!/usr/bin/env python3
"""Reject same-path cost, map-predicate and provenance tampering of raw reports."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
from verify_native_raw_map_report import verify


def run_tests(source,witness,raw_archive,prepared,cpp,report_path,subset_path,output):
    output.mkdir();report=json.loads(report_path.read_text());subset=json.loads(subset_path.read_text())
    result=verify(source,witness,raw_archive,prepared,cpp,report,subset)
    (output/'positive.json').write_text(json.dumps(result,indent=2)+'\n');checks={'positive':result['verdict']=='PASS'}
    def probe(name,change):
        r=json.loads(json.dumps(report));s=json.loads(json.dumps(subset));change(r,s)
        (output/(name+'.report.json')).write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
        (output/(name+'.subset.json')).write_text(json.dumps(s,indent=2,allow_nan=False)+'\n')
        try:verify(source,witness,raw_archive,prepared,cpp,r,s)
        except ValueError as error:
            checks[name+'_rejected']=True;(output/(name+'.rejection.json')).write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_rejected']=False
    probe('risk_one_ULP',lambda r,s:r['rows'][73].update(actual_dynamic_risk=math.nextafter(r['rows'][73]['actual_dynamic_risk'],math.inf)))
    probe('weight_one_ULP',lambda r,s:r['rows'][185].update(actual_native_weight=math.nextafter(r['rows'][185]['actual_native_weight'],math.inf)))
    probe('original_cost_sum_changed',lambda r,s:r['rows'][73].update(original_seven_and_StaticStopping_cost_sum_before_dynamic=r['rows'][73]['original_seven_and_StaticStopping_cost_sum_before_dynamic']+.01))
    probe('CPP_boundary_stat_changed',lambda r,s:r['rows'][185]['epochs']['actual_score_clock']['dense_SDK_queries'].update(maximum_native_footprint_cost=201))
    probe('forged_raw_vector_pass',lambda r,s:r['rows'][82]['epochs']['actual_score_clock'].update(raw203_conservative_vector_gate_pass=True))
    probe('invented_SG_acceptance',lambda r,s:r['rows'][185].update(raw_path_is_an_accepted_SG_control=True))
    probe('invented_actual_critic',lambda r,s:r['rows'][73].update(actual_CostCritic_individual_increment_or_collision='known'))
    probe('rational_mass_changed',lambda r,s:r['epoch_summaries']['actual_score_clock']['partition'][0].update(actual_weight_mass=r['epoch_summaries']['actual_score_clock']['partition'][0]['actual_weight_mass']+1e-8))
    probe('summary_count_changed',lambda r,s:r['epoch_summaries']['actual_score_clock'].update(whole_polygon_vector_pass_rows=91))
    probe('shrink_exact_cell_scope',lambda r,s:s.update(verified_original_raw_rows=[73,127,185]))
    data=bytearray((prepared/'queries.bin').read_bytes());value=struct.unpack_from('<d',data,10172)[0];new=math.nextafter(value,math.inf);struct.pack_into('<d',data,10172,new)
    meta=json.loads((prepared/'input_identity.json').read_text());meta['input_sha256']=hashlib.sha256(data).hexdigest();r=json.loads(json.dumps(report));r['input_sha256']=meta['input_sha256']
    changed=output/'coordinated_pose';changed.mkdir();(changed/'queries.bin').write_bytes(data);(changed/'input_identity.json').write_text(json.dumps(meta,indent=2)+'\n')
    (changed/'report.json').write_text(json.dumps(r,indent=2)+'\n');(changed/'mutation.json').write_text(json.dumps({'mode':'binary_patch','base_sha256':hashlib.sha256((prepared/'queries.bin').read_bytes()).hexdigest(),'offset':10172,'delete_bytes':8,'insert_hex':struct.pack('<d',new).hex(),'mutated_input_sha256':meta['input_sha256']},indent=2)+'\n')
    try:verify(source,witness,raw_archive,changed,cpp,r,subset)
    except ValueError as error:
        checks['coordinated_pose_hash_report_rejected']=True;(changed/'rejection.json').write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
    else:checks['coordinated_pose_hash_report_rejected']=False
    before=(output/'positive.json').read_bytes()
    process=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('verify_native_raw_map_report.py')),*map(str,[source,witness,raw_archive,prepared,cpp,report_path,subset_path,output/'positive.json'])],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (output/'existing_output.log').write_bytes(process.stdout)
    checks['existing_output_protected']=process.returncode!=0 and b'refusing to overwrite' in process.stdout and (output/'positive.json').read_bytes()==before
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Same original raw costs/weights and query provenance; reject forged map/SG acceptance and preserve exact-subset scope.'}
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_archive','prepared','cpp','report','subset','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=run_tests(*[getattr(args,n) for n in ['source','witness','raw_archive','prepared','cpp','report','subset','output']])
    print(json.dumps(result),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
