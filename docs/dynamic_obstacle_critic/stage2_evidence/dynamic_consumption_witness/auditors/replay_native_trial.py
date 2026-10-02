#!/usr/bin/env python3
"""Replay frozen reports and SG comparison evidence in a verified private copy.

This regenerates the native replay input and independently checks saved C++
outputs. Re-executing C++ against the pinned Humble ELF is a separate step;
--native-outputs verifies those independently generated outputs too.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--native-outputs',type=Path);args=parser.parse_args()
    archive=args.archive.resolve();manifest=json.loads((archive/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        path=(archive/name).resolve()
        if not path.is_relative_to(archive) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('archive identity mismatch: '+name)
    shutil.copytree(archive,args.output)
    output=args.output.resolve();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR=str(output/'mpl_config'))
    replay_policy=json.loads((archive/'replay_policy.json').read_text()) if (archive/'replay_policy.json').exists() else {}
    plot_title=replay_policy.get('plot_title','Native cycle evidence trial: safety / task FAILED')
    jobs=[('analyze_trial.py','analysis.json',[]),('audit_raw_costmap_gate.py','raw_costmap_audit.json',[]),
          ('audit_guard_rejections.py','guard_audit.json',[]),('audit_stopping_objective.py','stopping_audit.json',[]),
          ('audit_prediction.py','prediction_audit.json',[]),('audit_scan_geometry.py','scan_geometry_audit.json',[]),
          ('audit_mechanical_footprint.py','mechanical_footprint_audit.json',['--include-static']),
          ('audit_contact_witness.py','contact_audit.json',[]),
          ('audit_contact_witness.py','mechanical_contact_audit.json',['--mechanical-events']),
          ('audit_trial_timing.py','timing_audit.json',[]),('audit_native_snapshots.py','native_snapshot_audit.json',[]),
          ('audit_native_commands.py','native_command_audit.json',[]),
          ('audit_raw_transitions.py','raw_transition_audit.json',['--output',str(output/'raw_transition_audit.json')]),
          ('plot_mechanical_contract_trial.py','mechanical_contract_witness.png',
           ['--title',plot_title,'--fit-near-geometry'])]
    results={}
    def run(script,arguments,name):
        with (output/(name+'.replay.log')).open('w') as log:
            subprocess.run([sys.executable,'-B',str(output/'auditors'/script),*map(str,arguments)],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    for script,report,flags in jobs:
        run(script,[output,*flags],report)
        results[report]=(output/report).read_bytes()==(archive/report).read_bytes()
    results['mechanical_contract_witness.svg']=(output/'mechanical_contract_witness.svg').read_bytes()==(archive/'mechanical_contract_witness.svg').read_bytes()
    if (archive/'effective_velocity_audit.json').exists():
        for script,report in [('audit_native_velocity_contract.py','velocity_contract_audit.json'),
                              ('audit_effective_native_velocity.py','effective_velocity_audit.json')]:
            (output/report).unlink() # Exclusive report writers; private copy only.
            run(script,[output,output/report],report)
            results[report]=(output/report).read_bytes()==(archive/report).read_bytes()
        run('plot_effective_native_velocity.py',[output],'effective_velocity_figure')
        for extension in ['png','svg']:
            name='effective_velocity.'+extension
            results[name]=(output/name).read_bytes()==(archive/name).read_bytes()
    if (archive/'dynamic_consumption_audit.json').exists():
        consumption_input=output/'replayed_dynamic_consumption'
        run('prepare_dynamic_consumption_replay.py',[output/'dynamic_scores',consumption_input],'prepare_dynamic_consumption')
        results['dynamic_input_identity.json']=(consumption_input/'input_identity.json').read_bytes()==(archive/'dynamic_replay/input_identity.json').read_bytes()
        import gzip
        risk=output/'replayed_dynamic_risk.jsonl'
        if args.native_outputs:
            risk.write_bytes((args.native_outputs/'dynamic_risk.jsonl').read_bytes())
            results['dynamic_risk.jsonl']=risk.read_bytes()==gzip.open(archive/'dynamic_replay/native_risk.jsonl.gz','rb').read()
        else:risk.write_bytes(gzip.open(archive/'dynamic_replay/native_risk.jsonl.gz','rb').read())
        for script,report,arguments in [
            ('audit_dynamic_consumption.py','dynamic_consumption_audit.json',[output,risk,output/'dynamic_consumption_audit.json']),
            ('audit_dynamic_velocity_epochs.py','dynamic_velocity_epochs.json',[output,output/'dynamic_velocity_epochs.json']),
            ('audit_consumed_cv_support.py','consumed_cv_support_audit.json',[output,output/'consumed_cv_support_audit.json'])]:
            (output/report).unlink();run(script,arguments,report)
            results[report]=(output/report).read_bytes()==(archive/report).read_bytes()
        run('plot_dynamic_consumption.py',[output,output/'native_optimizer/weights_output.jsonl.gz'],'dynamic_consumption_figure')
        for extension in ['png','svg']:
            name='dynamic_consumption.'+extension
            results[name]=(output/name).read_bytes()==(archive/name).read_bytes()
        if args.native_outputs:
            for name in ['weights_output.jsonl','uniform_output.jsonl']:
                results[name]=(args.native_outputs/name).read_bytes()==gzip.open(archive/'native_optimizer'/(name+'.gz'),'rb').read()
    prepared=output/'replayed_optimizer'
    association_flags=(['--association-window',str(replay_policy['association_window_seconds'])]
                       if 'association_window_seconds' in replay_policy else [])
    run('prepare_native_optimizer_replay.py',[output,prepared,*association_flags],'prepare_native_optimizer')
    results['schedule.json']=(prepared/'schedule.json').read_bytes()==(archive/'native_optimizer/schedule.json').read_bytes()
    for name,file in [('positive','sdk_final_output.jsonl'),('zero_history','zero_history_output.jsonl'),
                      ('omitted_reset','omitted_reset_output.jsonl'),('initial_scalar','initial_scalar_output.jsonl')]:
        candidate=output/'native_optimizer'/(file+'.gz')
        if not candidate.exists():
            if name=='initial_scalar' or (name=='omitted_reset' and replay_policy.get('no_noninitial_reset')):continue
            raise ValueError('required native replay evidence missing: '+file)
        if args.native_outputs and name!='initial_scalar':candidate=args.native_outputs/file
        report=prepared/(name+'_comparison.json')
        run('compare_native_optimizer_replay.py',[prepared/'schedule.json',candidate,report],name+'_comparison')
        results[name+'_comparison.json']=report.read_bytes()==(archive/'native_optimizer'/(name+'_comparison.json')).read_bytes()
    result={'result':'PASS' if all(results.values()) else 'FAILED','reports_and_figures_byte_exact':results,
            'source_data':'verified frozen gzip archive; output is a private copy',
            'native_cpp_outputs':'externally re-executed' if args.native_outputs else 'saved frozen evidence only',
            'native_input_regenerated':True,'limits':'Replay integrity does not alter the FAILED physical verdict or prove safe control/coverage.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    return 0 if result['result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
