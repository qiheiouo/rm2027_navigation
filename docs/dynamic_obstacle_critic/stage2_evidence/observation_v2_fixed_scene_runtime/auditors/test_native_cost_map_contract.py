#!/usr/bin/env python3
"""Reject coordinated query identities and distinct raw/SG provenance changes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from analyze_native_cost_map_contract import analyze


def run_tests(source,witness,stages,mass,prepared,cpp,synthetic,report,output):
    output.mkdir();genuine=analyze(source,witness,stages,mass,prepared,cpp,synthetic)
    positive=output/'positive.json';positive.write_text(json.dumps(genuine,indent=2,allow_nan=False)+'\n')
    checks={'positive_byte_exact':positive.read_bytes()==report.read_bytes()};blob=(prepared/'queries.bin').read_bytes()
    identity=json.loads((prepared/'input_identity.json').read_text())
    def probe(name,change):
        data=bytearray(blob);meta=json.loads(json.dumps(identity));change(data,meta)
        meta['input_sha256']=hashlib.sha256(data).hexdigest();root=output/name;root.mkdir()
        (root/'queries.bin').write_bytes(data);(root/'input_identity.json').write_text(json.dumps(meta,indent=2)+'\n')
        try:analyze(source,witness,stages,mass,root,cpp,synthetic)
        except ValueError as error:
            checks[name+'_rejected']=True;(root/'rejection.json').write_text(json.dumps({'reason':str(error)},indent=2)+'\n')
        else:checks[name+'_rejected']=False
    def map_change(data,meta):
        data[140]=255;meta['map_raw_bytes_sha256']=hashlib.sha256(data[140:10140]).hexdigest()
    def footprint_change(data,meta):
        value=math.nextafter(meta['padded_footprint'][0][0],math.inf);struct.pack_into('<d',data,76,value);meta['padded_footprint'][0][0]=value
    probe('coordinated_map',map_change);probe('coordinated_footprint',footprint_change)
    def pose_change(data,meta):
        value=struct.unpack_from('<d',data,10172)[0];struct.pack_into('<d',data,10172,math.nextafter(value,math.inf))
    probe('coordinated_pose_and_hash',pose_change)
    probe('reference_identity',lambda d,m:m['reference_manifests'].__setitem__('source','0'*64))
    probe('stage_identity',lambda d,m:m['traces'][0].update(stage='actual_history_after_SG'))
    probe('distinct_raw_SG_identity',lambda d,m:m['traces'][8].update(witness_row=0))
    probe('epoch_identity',lambda d,m:m['traces'][0].update(epoch_time=47.98))
    probe('raw_subset_identity',lambda d,m:m['individual_raw_rows'].__setitem__(0,1))
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Coordinated provenance/hash probes preserve original full gates and distinguish raw costs from individual SG paths.'}
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','stages','mass','prepared','cpp','synthetic','report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=run_tests(*[getattr(args,n) for n in ['source','witness','stages','mass','prepared','cpp','synthetic','report','output']])
    print(json.dumps(result),flush=True);raise SystemExit(0 if result['verdict']=='PASS' else 1)
