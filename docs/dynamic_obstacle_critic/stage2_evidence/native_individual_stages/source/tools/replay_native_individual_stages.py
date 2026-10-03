#!/usr/bin/env python3
"""Rebuild registered individual-stage inputs and fixture attribution from archives."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from prepare_processed_cv import verify


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    for name in ['stages','witness_binary','witness_json','verification']:parser.add_argument('--native-'+name.replace('_','-'),type=Path)
    args=parser.parse_args();supplied=[args.native_stages,args.native_witness_binary,args.native_witness_json,args.native_verification]
    if any(supplied) and not all(supplied):raise ValueError('all four native artifacts required for supplied execution comparison')
    archive=args.archive.resolve();verify(archive);provenance=json.loads((archive/'provenance.json').read_text());refs=[]
    for name in ['source','witness','raw_map','mass']:
        root=(archive/provenance[name+'_archive']).resolve();verify(root)
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=provenance[name+'_manifest_sha256']:raise ValueError('stage reference manifest changed')
        refs.append(root)
    shutil.copytree(archive,args.output);output=args.output.resolve();scripts=output/'source/tools'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(scripts),MPLCONFIGDIR=str(output/'mpl_config'))
    def run(name,arguments,label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable,'-B',str(scripts/name),*map(str,arguments)],env=env,stdout=stream,stderr=subprocess.STDOUT,check=True)
    prepared=output/'recomputed_prepared';run('prepare_native_witness.py',[refs[0],prepared,'--source-ordinals','298'],'prepare')
    paths=[]
    for arg,name in [(args.native_stages,'stages.bin'),(args.native_witness_binary,'witness.bin'),(args.native_witness_json,'witness.jsonl')]:
        p=output/('replayed_'+name);p.write_bytes(arg.read_bytes() if arg else gzip.open(archive/(name+'.gz'),'rb').read());paths.append(p)
    stages,binary,js=paths;analysis=output/'recomputed_analysis'
    run('analyze_native_individual_stages.py',[*refs,prepared,binary,js,stages,analysis],'analysis')
    figures=output/'recomputed_figures';run('plot_native_individual_stages.py',[refs[0],stages,analysis/'report.json',analysis/'raw185_scalar.json',figures],'figures')
    checks={}
    for name in ['input.bin','input_identity.json']:checks['prepared/'+name]=(prepared/name).read_bytes()==gzip.open(archive/'prepared'/(name+'.gz'),'rb').read() if name.endswith('.bin') else (prepared/name).read_bytes()==(archive/'prepared'/name).read_bytes()
    for name in ['report.json','raw185_scalar.json']:checks[name]=(analysis/name).read_bytes()==gzip.open(archive/'analysis'/(name+'.gz'),'rb').read()
    for name in ['native_individual_stages.png','native_individual_stages.svg']:checks[name]=(figures/name).read_bytes()==(archive/'figures'/name).read_bytes()
    if all(supplied):
        for p,name in zip(paths,['stages.bin','witness.bin','witness.jsonl']):checks['supplied_native/'+name]=p.read_bytes()==gzip.open(archive/(name+'.gz'),'rb').read()
        checks['supplied_native/verification.json']=args.native_verification.read_bytes()==(archive/'verification/tests/verification.json').read_bytes()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
        'native_source':'new supplied compiled artifacts' if all(supplied) else 'frozen native proof only',
        'scope':'Registered source input, all original raw/311 baseline joins, conservative stage fixture labels and raw185 exact cells. No runtime control or complete critic state certificate.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
