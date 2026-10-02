#!/usr/bin/env python3
"""Replay frozen witness analyses without altering either source archive."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def verify(root):
    manifest=json.loads((root/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('archive hash mismatch: '+name)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--source-archive',type=Path);args=parser.parse_args()
    archive=args.archive.resolve();source=(args.source_archive or archive.parent/'gazebo_native_cycle_evidence').resolve()
    verify(archive);verify(source)
    provenance=json.loads((archive/'provenance.json').read_text())
    if hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()!=provenance['source_manifest_sha256']:
        raise ValueError('referenced native source archive identity')
    shutil.copytree(archive,args.output);output=args.output.resolve()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script,arguments,name):
        with (output/(name+'.log')).open('w') as log:
            subprocess.run([sys.executable,'-B',str(output/'auditors'/script),*map(str,arguments)],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    def saved(path):return gzip.open(path,'rb').read() if path.suffix=='.gz' else path.read_bytes()
    comparisons={}
    for name,ordinals in [('full',[]),('selected_cycle_223',['--source-ordinals','223'])]:
        prepared=output/(name+'_prepared')
        run('prepare_native_witness.py',[source,prepared,*ordinals],name+'_prepare')
        comparisons[name+'/input_identity.json']=(prepared/'input_identity.json').read_bytes()==(archive/name/'input_identity.json').read_bytes()
        report=output/(name+'_analysis_replayed.json')
        run('analyze_native_witness.py',[source,output/name/'witnesses.bin.gz',prepared/'input_identity.json',report],name+'_analysis')
        expected=archive/name/('analysis_final.json.gz' if name=='full' else 'analysis.json.gz')
        comparisons[name+'/analysis']=report.read_bytes()==saved(expected)
    velocity=output/'velocity_audit_replayed.json'
    run('audit_native_velocity_contract.py',[source,velocity],'velocity_audit')
    comparisons['velocity_contract_audit_final.json']=velocity.read_bytes()==(archive/'full/velocity_contract_audit_final.json').read_bytes()
    assessment=output/'contract_assessment_replayed.json'
    run('assess_native_witness_contract.py',[output/'full_analysis_replayed.json',output/'selected_cycle_223_analysis_replayed.json',
        velocity,output/'odom_contract_probe_configured/readback.json',assessment],'contract_assessment')
    comparisons['contract_assessment.json']=assessment.read_bytes()==(archive/'full/contract_assessment.json').read_bytes()
    run('plot_native_velocity_contract.py',[velocity,output/'velocity_contract'],'velocity_plot')
    for suffix in ['png','svg']:
        comparisons['velocity_contract.'+suffix]=(output/('velocity_contract.'+suffix)).read_bytes()==(archive/'full'/('velocity_contract.'+suffix)).read_bytes()
    result={'result':'PASS' if all(comparisons.values()) else 'FAILED','byte_exact':comparisons,
            'source_and_witness_manifests_verified':True,'input_regenerated':True,
            'cpp_output_scope':'saved actual C++ outputs; native re-execution separate',
            'physical_witness':'NOT ESTABLISHED — effective native velocity input contract FAILED',
            'limits':'Configured controller DDS readback is frozen evidence; this Python replay does not re-launch controller or execute native C++.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    return 0 if result['result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
