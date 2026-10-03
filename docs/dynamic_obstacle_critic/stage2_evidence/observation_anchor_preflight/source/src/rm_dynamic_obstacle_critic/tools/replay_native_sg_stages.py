#!/usr/bin/env python3
"""Reconstruct frozen native context and all eight full-horizon stage labels."""
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
    args=parser.parse_args();overrides=[args.native_paths,args.native_json,args.score_cpp]
    if any(overrides) and not all(overrides):raise ValueError('all three independent CPP artifacts required together')
    archive=args.archive.resolve();verify(archive);provenance=json.loads((archive/'provenance.json').read_text());refs={}
    for name in ['source','witness','factors','aggregation']:
        root=(archive/provenance[name+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=provenance[name+'_manifest_sha256']:
            raise ValueError('referenced '+name+' manifest mismatch')
        verify(root);refs[name]=root
    shutil.copytree(archive,args.output);output=args.output.resolve();tools=output/'source/tools';checks={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(tools),MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script,arguments,label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable,'-B',str(tools/script),*map(str,arguments)],stdout=stream,stderr=subprocess.STDOUT,env=env,check=True)
    context=output/'recomputed_native_context'
    run('prepare_native_factor_aggregation.py',[refs['source'],refs['witness'],refs['factors'],context],'native_context')
    for name in ['aggregation_input.bin','input_identity.json']:
        reference=refs['aggregation']/'prepared'/(name+('.gz' if name.endswith('.bin') else ''))
        frozen=gzip.open(reference,'rb').read() if reference.suffix=='.gz' else reference.read_bytes()
        checks['context/'+name]=(context/name).read_bytes()==frozen
    paths=args.native_paths.resolve() if args.native_paths else archive/'stage_paths.bin'
    if args.native_json:
        native=args.native_json.resolve();score=args.score_cpp.resolve()
        checks['independent_CPP_stage_paths']=paths.read_bytes()==(archive/'stage_paths.bin').read_bytes()
        checks['independent_CPP_stage_json']=native.read_bytes()==gzip.open(archive/'native_output.jsonl.gz','rb').read()
        checks['independent_CPP_stage_scores']=score.read_bytes()==gzip.open(archive/'scores.jsonl.gz','rb').read()
    else:
        native=output/'frozen_native.jsonl';score=output/'frozen_scores.jsonl'
        native.write_bytes(gzip.open(archive/'native_output.jsonl.gz','rb').read());score.write_bytes(gzip.open(archive/'scores.jsonl.gz','rb').read())
    prepared=output/'recomputed_scoring'
    run('prepare_native_stage_scoring.py',[refs['factors'],refs['aggregation'],paths,prepared],'scoring')
    for name in ['actual_input.bin','factor_input.bin','input_identity.json']:
        checks['scoring/'+name]=(prepared/name).read_bytes()==(archive/'scoring'/name).read_bytes()
    reports=[output/name for name in ['recomputed_analysis.json','recomputed_scalar.json','recomputed_raw.json']]
    run('analyze_native_sg_stages.py',[refs['source'],refs['aggregation'],prepared,paths,native,score,*reports],'analysis')
    for path,name in zip(reports,['analysis.json','scalar_verification.json','raw_scalar_verification.json']):
        checks[name]=path.read_bytes()==(archive/name).read_bytes()
    run('test_native_sg_stage_io.py',[paths,refs['aggregation']/'native_paths.bin',output/'recomputed_io_tests'],'io')
    checks['io_verification.json']=(output/'recomputed_io_tests/verification.json').read_bytes()==(archive/'verification/io_tests/verification.json').read_bytes()
    run('test_native_sg_stage_evidence.py',[refs['source'],refs['aggregation'],prepared,paths,native,score,reports[0],output/'recomputed_evidence_tests'],'evidence')
    checks['evidence_verification.json']=(output/'recomputed_evidence_tests/verification.json').read_bytes()==(archive/'verification/evidence_tests/verification.json').read_bytes()
    run('plot_native_sg_stages.py',[refs['source'],paths,reports[0],reports[2],output/'recomputed_figures'],'figures')
    for name in ['native_sg_stages.png','native_sg_stages.svg']:
        checks[name]=(output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
        'CPP_scope':'independently re-executed' if args.native_json else 'frozen stage artifacts',
        'scope':'Fixed-context stage integrity; raw contact already exists before SG; original FAILED/NOT ESTABLISHED remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
