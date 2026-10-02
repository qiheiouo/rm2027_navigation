#!/usr/bin/env python3
"""Independently rebuild full individual gates, rational masses and raw subset."""
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
    parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    archive=args.archive.resolve();verify(archive);provenance=json.loads((archive/'provenance.json').read_text());refs={}
    names=['source','witness','processed','factors','aggregation','stages','raw']
    for name in names:
        root=(archive/provenance[name+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=provenance[name+'_manifest_sha256']:raise ValueError('referenced '+name+' manifest changed')
        verify(root);refs[name]=root
    shutil.copytree(archive,args.output);output=args.output.resolve();tools=output/'source/tools'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(tools),MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script,arguments,label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable,'-B',str(tools/script),*map(str,arguments)],stdout=stream,stderr=subprocess.STDOUT,env=env,check=True)
    roots=[refs[n] for n in names];report=output/'recomputed_report.json';scalar=output/'recomputed_scalar.json';verification=output/'recomputed_verification.json'
    run('audit_native_factor_safe_mass.py',[*roots,report,scalar],'audit')
    run('verify_native_factor_safe_mass.py',[*roots,report,scalar,verification],'rational')
    run('test_native_factor_safe_mass.py',[*roots,report,scalar,output/'recomputed_tests'],'negative')
    run('plot_native_factor_safe_mass.py',[report,output/'recomputed_figures'],'figures')
    checks={}
    for path,name in [(report,'report.json.gz'),(scalar,'scalar_verification.json.gz'),(verification,'verification.json')]:
        frozen=gzip.open(archive/name,'rb').read() if name.endswith('.gz') else (archive/name).read_bytes()
        checks[name]=path.read_bytes()==frozen
    checks['tests/verification.json']=(output/'recomputed_tests/verification.json').read_bytes()==(archive/'verification/tests/verification.json').read_bytes()
    for name in ['native_factor_safe_mass.png','native_factor_safe_mass.svg']:
        checks[name]=(output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
        'scope':'Independently recomputed full300 individual gates and 43-row scalar subset, rational mass and figures; original FAILED/NOT ESTABLISHED remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
