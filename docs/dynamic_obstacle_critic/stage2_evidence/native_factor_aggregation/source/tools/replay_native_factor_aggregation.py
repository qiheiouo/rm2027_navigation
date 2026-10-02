#!/usr/bin/env python3
"""Rebuild frozen fixed-context hypotheses, full gates and independent cells."""
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
    for name in ['native-paths','native-json','score-cpp']:parser.add_argument('--'+name,type=Path)
    args=parser.parse_args()
    overrides=[args.native_paths,args.native_json,args.score_cpp]
    if any(overrides) and not all(overrides):raise ValueError('all three independently re-executed CPP artifacts required together')
    archive=args.archive.resolve();verify(archive);provenance=json.loads((archive/'provenance.json').read_text());references={}
    for name in ['source','witness','factors']:
        root=(archive/provenance[name+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=provenance[name+'_manifest_sha256']:
            raise ValueError('referenced '+name+' manifest mismatch')
        verify(root);references[name]=root
    shutil.copytree(archive,args.output);output=args.output.resolve();tools=output/'source/tools';checks={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(tools),MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script,arguments,label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable,'-B',str(tools/script),*map(str,arguments)],stdout=stream,stderr=subprocess.STDOUT,env=env,check=True)
    prepared=output/'recomputed_input'
    run('prepare_native_factor_aggregation.py',[references['source'],references['witness'],references['factors'],prepared],'prepare')
    checks['aggregation_input.bin']=(prepared/'aggregation_input.bin').read_bytes()==gzip.open(archive/'prepared/aggregation_input.bin.gz','rb').read()
    checks['input_identity.json']=(prepared/'input_identity.json').read_bytes()==(archive/'prepared/input_identity.json').read_bytes()
    paths=args.native_paths.resolve() if args.native_paths else archive/'native_paths.bin'
    if args.native_json:
        native_json=args.native_json.resolve();score=args.score_cpp.resolve()
        checks['independent_CPP_native_paths']=paths.read_bytes()==(archive/'native_paths.bin').read_bytes()
        checks['independent_CPP_native_json']=native_json.read_bytes()==gzip.open(archive/'native_output.jsonl.gz','rb').read()
        checks['independent_CPP_score_output']=score.read_bytes()==gzip.open(archive/'scores.jsonl.gz','rb').read()
    else:
        native_json=output/'frozen_native_output.jsonl';score=output/'frozen_scores.jsonl'
        native_json.write_bytes(gzip.open(archive/'native_output.jsonl.gz','rb').read());score.write_bytes(gzip.open(archive/'scores.jsonl.gz','rb').read())
    scoring=output/'recomputed_scoring'
    run('prepare_native_factor_scoring.py',[references['factors'],paths,scoring],'scoring')
    for name in ['actual_input.bin','factor_input.bin','input_identity.json']:
        checks['scoring/'+name]=(scoring/name).read_bytes()==(archive/'scoring'/name).read_bytes()
    analysis=output/'recomputed_analysis.json';scalar=output/'recomputed_scalar.json';raw=output/'recomputed_raw_scalar.json'
    run('analyze_native_factor_aggregation.py',[references['source'],references['witness'],references['factors'],prepared,paths,native_json,score,analysis,scalar],'analysis')
    checks['analysis.json']=analysis.read_bytes()==(archive/'analysis.json').read_bytes()
    checks['scalar_verification.json']=scalar.read_bytes()==(archive/'scalar_verification.json').read_bytes()
    run('verify_native_factor_raw.py',[references['source'],paths,analysis,raw],'raw_scalar')
    checks['raw_scalar_verification.json']=raw.read_bytes()==(archive/'raw_scalar_verification.json').read_bytes()
    run('test_native_factor_raw_evidence.py',[references['source'],paths,analysis,output/'recomputed_raw_negative'],'raw_negative')
    checks['raw_negative_verification.json']=(output/'recomputed_raw_negative/verification.json').read_bytes()==(archive/'verification/raw_negative/verification.json').read_bytes()
    run('test_native_factor_io.py',[paths,output/'recomputed_io_tests'],'io')
    checks['io_verification.json']=(output/'recomputed_io_tests/verification.json').read_bytes()==(archive/'verification/io_tests/verification.json').read_bytes()
    run('plot_native_factor_aggregation.py',[references['source'],paths,analysis,raw,output/'recomputed_figures'],'figures')
    for name in ['native_factor_aggregation.png','native_factor_aggregation.svg']:
        checks[name]=(output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    report={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
            'CPP_scope':'independently re-executed' if args.native_json else 'frozen evaluated artifacts',
            'scope':'Conditional four-path numerical/gate/raw-cell integrity; all original FAILED/NOT ESTABLISHED and inadmissible-oracle limits remain.'}
    (output/'replay_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
