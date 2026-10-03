#!/usr/bin/env python3
"""Verified archive replay; new SDK execution is an explicit external input.

All original raw records and failed gates remain immutable in the archive.
The original runtime cost-audit formatter regenerates its historical report;
post-trial tools verify dynamic attribution and the full recorded output chain.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def validate(archive):
    manifest=json.loads((archive/'manifest.json').read_text())
    for relative,digest in manifest['files'].items():
        path=(archive/relative).resolve()
        if not path.is_relative_to(archive) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('archive file identity mismatch '+relative)
    return manifest


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--native-outputs',type=Path);args=parser.parse_args()
    archive=args.archive.resolve();manifest=validate(archive)
    shutil.copytree(archive,args.output);out=args.output.resolve();tools=out/'auditors'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR=str(out/'mpl_config'))
    checks={};policy=json.loads((out/'replay_policy.json').read_text())
    def run(script,arguments,name,source=None):
        with (out/(name+'.replay.log')).open('x') as log:
            subprocess.run([sys.executable,'-B',str((source or tools)/script),*map(str,arguments)],
                env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    def regenerate(script,report,arguments,source=None):
        (out/report).unlink();run(script,arguments,report,source)
        checks[report]=(out/report).read_bytes()==(archive/report).read_bytes()
    def decode(relative,target,external=None):
        data=external.read_bytes() if external is not None else gzip.open(out/relative,'rb').read()
        with target.open('xb') as stream:stream.write(data)
        if external is not None:checks[relative]=data==gzip.open(archive/relative,'rb').read()
    risk=out/'replayed_dynamic_risk.jsonl'
    decode('dynamic_replay/native_risk.jsonl.gz',risk,args.native_outputs/'dynamic_risk.jsonl' if args.native_outputs else None)
    optimizer=out/'replayed_optimizer';optimizer.mkdir()
    for name in ('sdk_final_output','zero_history_output','weights_output','uniform_output'):
        decode('native_optimizer/'+name+'.jsonl.gz',optimizer/(name+'.jsonl'),args.native_outputs/(name+'.jsonl') if args.native_outputs else None)
    dynamic_input=out/'regenerated_dynamic_input'
    run('prepare_dynamic_consumption_replay.py',[out/'dynamic_scores',dynamic_input],'prepare_dynamic')
    checks['dynamic_replay/input_identity.json']=(dynamic_input/'input_identity.json').read_bytes()==(archive/'dynamic_replay/input_identity.json').read_bytes()
    native_input=out/'regenerated_optimizer_input'
    run('prepare_native_optimizer_replay.py',[out,native_input,'--association-window',str(policy['association_window_seconds'])],'prepare_optimizer')
    checks['native_optimizer/schedule.json']=(native_input/'schedule.json').read_bytes()==(archive/'native_optimizer/schedule.json').read_bytes()
    shutil.copyfile(native_input/'input.bin',optimizer/'input.bin')
    shutil.copyfile(native_input/'schedule.json',optimizer/'schedule.json')
    run('compare_native_optimizer_replay.py',[optimizer/'schedule.json',optimizer/'sdk_final_output.jsonl',optimizer/'positive_comparison.json'],'compare_native')
    run('compare_native_optimizer_replay.py',[optimizer/'schedule.json',optimizer/'zero_history_output.jsonl',optimizer/'zero_history_comparison.json'],'compare_zero_history')
    for name in ('positive_comparison.json','zero_history_comparison.json'):
        checks['native_optimizer/'+name]=(optimizer/name).read_bytes()==(archive/'native_optimizer'/name).read_bytes()
    regenerate('audit_cost_accumulator.py','cost_accumulator_audit.json',[out,out/'cost_accumulator_audit.json'],out/'runtime_source/tools')
    regenerate('audit_dynamic_consumption.py','dynamic_consumption_audit.json',[out,risk,out/'dynamic_consumption_audit.json','--cost-accumulator-evidence'])
    regenerate('verify_cost_accumulator_chain.py','cost_accumulator_full_chain.json',[out,optimizer,risk,out/'cost_accumulator_full_chain.json'])
    jobs=[('analyze_trial.py','analysis.json',[]),('audit_raw_costmap_gate.py','raw_costmap_audit.json',[]),
        ('audit_mechanical_footprint.py','mechanical_footprint_audit.json',['--include-static']),
        ('audit_native_snapshots.py','native_snapshot_audit.json',[]),('audit_native_commands.py','native_command_audit.json',[]),
        ('audit_guard_rejections.py','guard_audit.json',[]),('audit_stopping_objective.py','stopping_audit.json',[]),
        ('audit_prediction.py','prediction_audit.json',[]),('audit_scan_geometry.py','scan_geometry_audit.json',[]),
        ('audit_trial_timing.py','timing_audit.json',[]),('audit_contact_witness.py','mechanical_contact_audit.json',['--mechanical-events'])]
    for script,report,flags in jobs:regenerate(script,report,[out,*flags])
    for script,report in [('audit_native_velocity_contract.py','velocity_contract_audit.json'),
                         ('audit_effective_native_velocity.py','effective_velocity_audit.json'),
                         ('audit_dynamic_velocity_epochs.py','dynamic_velocity_epochs.json'),
                         ('audit_consumed_cv_support.py','consumed_cv_support_audit.json')]:
        regenerate(script,report,[out,out/report])
    regenerate('audit_raw_transitions.py','raw_transition_audit.json',[out,'--output',out/'raw_transition_audit.json'])
    for script,arguments,name in [('plot_cost_accumulator.py',[out],'cost_accumulator'),
        ('plot_dynamic_consumption.py',[out,optimizer/'weights_output.jsonl'],'dynamic_consumption'),
        ('plot_effective_native_velocity.py',[out],'effective_velocity'),
        ('plot_mechanical_contract_trial.py',[out,'--title',policy['plot_title'],'--fit-near-geometry'],'mechanical_contract_witness')]:
        run(script,arguments,name)
        for extension in ('png','svg'):
            checks[name+'.'+extension]=(out/(name+'.'+extension)).read_bytes()==(archive/(name+'.'+extension)).read_bytes()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,'count':len(checks),
        'manifest_files_verified':len(manifest['files']),
        'native_outputs_independently_reexecuted':bool(args.native_outputs),
        'scope':'Byte-exact reports/figures/inputs and optional freshly reexecuted SDK outputs. Original physical/raw203/task failures and unassigned pairs remain.'}
    (out/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    if not all(checks.values()):raise SystemExit(1)


if __name__=='__main__':main()
