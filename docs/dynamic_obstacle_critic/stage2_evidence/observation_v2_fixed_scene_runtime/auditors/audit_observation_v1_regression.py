#!/usr/bin/env python3
"""Reexecute the fixed v1 risk trace and three radius-related historical audits."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from audit_consumed_cv_support import audit
from audit_contact_witness import analyze
from prepare_dynamic_consumption_replay import prepare
from replay_observation_anchor_preflight import verify

SOURCE_MANIFEST = 'a2adeed5ddd006a00c95a6926824b0b5a56adb1b04ff4f3ecb34fd3ec97ec0a5'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('new_output',type=Path)
    parser.add_argument('replay_helper',type=Path);args=parser.parse_args();source=args.archive.resolve()
    if hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()!=SOURCE_MANIFEST:
        raise ValueError('registered v1 archive identity required')
    verify(source);args.new_output.mkdir(parents=True,exist_ok=False);out=args.new_output;checks={}
    identity=prepare(source/'dynamic_scores',out/'prepared')
    checks['347_input_identity_byte_exact']=(out/'prepared/input_identity.json').read_bytes()==(source/'dynamic_replay/input_identity.json').read_bytes()
    with (out/'risk.log').open('x') as log:
        subprocess.run([str(args.replay_helper),str(out/'prepared/input.bin'),str(out/'risk.jsonl')],stdout=log,stderr=subprocess.STDOUT,check=True)
    checks['347_risk_output_byte_exact']=(out/'risk.jsonl').read_bytes()==gzip.open(source/'dynamic_replay/native_risk.jsonl.gz','rb').read()
    report=audit(source)
    (out/'consumed_cv_support_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    checks['consumed_support_report_byte_exact']=(out/'consumed_cv_support_audit.json').read_bytes()==(source/'consumed_cv_support_audit.json').read_bytes()
    report=analyze(source,True)
    (out/'mechanical_contact_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    checks['contact_report_byte_exact']=(out/'mechanical_contact_audit.json').read_bytes()==(source/'mechanical_contact_audit.json').read_bytes()
    receipt=out/'receipt';receipt.mkdir()
    for relative in ['execution.json','profile.yaml','gazebo_poses.jsonl.gz','observations.jsonl.gz']:
        shutil.copyfile(source/relative,receipt/relative)
    with (out/'prediction.log').open('x') as log:
        subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('audit_prediction.py')),str(receipt)],
                       stdout=log,stderr=subprocess.STDOUT,check=True)
    checks['prediction_report_byte_exact']=(receipt/'prediction_audit.json').read_bytes()==(source/'prediction_audit.json').read_bytes()
    result=dict(status='PASS' if all(checks.values()) else 'FAILED',checks=checks,
                verified_dependency_manifest=SOURCE_MANIFEST,input_sha256=identity['input_sha256'],
                risk_sha256=hashlib.sha256((out/'risk.jsonl').read_bytes()).hexdigest(),
                helper_sha256=hashlib.sha256(args.replay_helper.read_bytes()).hexdigest(),
                scope='Reexecuted historical v1 trace and default-radius reports; original failed physical/task gates stay failed.')
    (out/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(json.dumps(result,sort_keys=True))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
