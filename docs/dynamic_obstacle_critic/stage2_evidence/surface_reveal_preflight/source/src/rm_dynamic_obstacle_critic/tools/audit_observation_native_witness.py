#!/usr/bin/env python3
"""Explicit v2 wrapper around full native geometry, preserving old v1 reports."""
import argparse
import hashlib
import json
from pathlib import Path
from analyze_native_witness import analyze
from audit_surface_reveal import validate


def audit(root,witness,identity_file,sdk_report,mode):
    manifest=validate(root)
    identity=json.loads(identity_file.read_text())
    selected=json.loads((root/'archive_identity.json').read_text())
    if selected['mode']!=mode or identity['cycles']!=identity['source_cycles']:
        raise ValueError('selected mode and complete source cycles required')
    sdk=json.loads(sdk_report.read_text())
    joined=json.loads((root/'dynamic_consumption_audit.json').read_text())
    cost=json.loads((root/'cost_accumulator_full_chain.json').read_text())
    if identity['source_manifest_sha256']!=manifest or sdk['verdict']!='PASS' or not sdk['checks'] or not all(sdk['checks'].values()):
        raise ValueError('complete independent SDK evidence required')
    if joined['verdict']!='EXACT CONSUMPTION/NATIVE JOIN PASS' or cost['verdict']!='EXACT OBSERVED COST/RISK/NATIVE WEIGHTS/SG CHAIN PASS':
        raise ValueError('actual dynamic/cost chain required')
    if joined['native_batches']!=identity['cycles'] or cost['cycles']!=identity['cycles']:
        raise ValueError('full source-cycle evidence required')
    for name in ('risk_exact','sdk_final_output_exact','weights_output_exact','all_witness_integrity','witness_complete'):
        if sdk['checks'].get(mode+' '+name) is not True:raise ValueError('selected mode SDK check missing '+name)
    result=analyze(root,witness,identity_file)
    result['evidence_schema']='observation_native_witness/v1'
    result['source_manifest_sha256']=manifest
    result['witness_payload_sha256']=hashlib.sha256(witness.read_bytes()).hexdigest()
    result['prepared_input_sha256']=identity['input_sha256']
    result['independent_sdk_report_sha256']=hashlib.sha256(sdk_report.read_bytes()).hexdigest()
    result['actual_consumed_public_context']='captured v2 fields and actual double risk; full raw paths/costs/control exact'
    result['limits']=('Conditional planar rollout of the SDK aggregate after SG and ten independently processed fixed proposals. '
        'Actual native MPPI returned command is bit-exact; guard pass/brake output and physical execution remain separate. '
        'Full 3s paths use source pose epoch and future physical truth labels only offline, measured twist values preserved. '
        'Source pose/capture/score epochs differ; map source and odometry subscriber source timestamp remain unavailable. '
        'Captured frozen raw203 context cannot certify future costmaps. e/B and complete future CV support not established. '
        'Raw 300 candidate paths are reproduced exactly by SDK but their full geometry is a separate pending analysis. '
        'No closed-loop/real-hardware acceptance or sampler/optimizer/SG changes.')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for field in ('archive','witness','identity','sdk_report','output'):parser.add_argument(field,type=Path)
    parser.add_argument('--mode',choices=('visible','nominal'),required=True);args=parser.parse_args()
    result=audit(args.archive,args.witness,args.identity,args.sdk_report,args.mode)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('records','first_witnesses')}))
