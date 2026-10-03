#!/usr/bin/env python3
"""Independently rebuild pure support-contract queries and rerun geometry/CLI."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from replay_processed_cv import verify


def geometry_summary(path):
    content=path.read_text()
    if not re.search(r'Ran 8 tests in ',content) or not content.rstrip().endswith('OK'):
        raise ValueError('complete eight geometry tests did not pass')
    row=next(line[line.index('{'):] for line in content.splitlines() if '"independent_irregular_cases"' in line)
    return {'verdict':'PASS','unit_tests':8,**json.loads(row)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();archive=args.archive.resolve();verify(archive)
    provenance=json.loads((archive/'provenance.json').read_text())
    source=(archive/provenance['source_archive']).resolve()
    if hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()!=provenance['source_manifest_sha256']:
        raise ValueError('physical source manifest mismatch')
    verify(source);shutil.copytree(archive,args.output)
    output=args.output.resolve();tools=output/'source/tools'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(tools))
    def run(script,arguments,label):
        log=output/(label+'.replay.log')
        with log.open('x') as stream:
            subprocess.run([sys.executable,'-B',str(tools/script),*map(str,arguments)],stdout=stream,stderr=subprocess.STDOUT,env=env,check=True)
        return log
    prepared=output/'recomputed_input'
    run('prepare_conditional_cv_queries.py',[source,prepared],'prepare')
    checks={p.name:(prepared/p.name).read_bytes()==p.read_bytes() for p in sorted((archive/'prepared').iterdir())}
    geometry=geometry_summary(run('test_conditional_cv_support.py',[],'geometry'))
    (output/'recomputed_geometry_verification.json').write_text(json.dumps(geometry,indent=2)+'\n')
    checks['geometry_verification.json']=(output/'recomputed_geometry_verification.json').read_bytes()==(archive/'geometry_verification.json').read_bytes()
    cli=output/'recomputed_cli'
    run('test_conditional_cv_cli.py',[prepared,cli],'cli')
    for name in ['verification.json','actual_unknown.json','synthetic_conditional.json']:
        checks['cli/'+name]=(cli/name).read_bytes()==(archive/'verification/cli_tests'/name).read_bytes()
    report={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
            'geometry_tests_reexecuted':8,'CLI_checks_reexecuted':14,
            'scope':'Conditional mathematics and unknown contract only; no physical bounds or deployment acceptance.'}
    (output/'replay_verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
