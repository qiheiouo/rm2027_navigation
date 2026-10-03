#!/usr/bin/env python3
"""Rebuild original raw grids and all declared vector/scalar/cost evidence."""
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
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--cpp-output',type=Path);parser.add_argument('--native-test-verification',type=Path)
    parser.add_argument('--mutation-verifier',type=Path,help='Explicit corrected verifier for an immutable archive; preserve and record both source identities.')
    args=parser.parse_args()
    archive=args.archive.resolve();verify(archive);prov=json.loads((archive/'provenance.json').read_text());refs=[]
    for name in ['source','witness','raw']:
        root=(archive/prov[name+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=prov[name+'_manifest_sha256']:raise ValueError('reference changed: '+name)
        verify(root);refs.append(root)
    shutil.copytree(archive,args.output);output=args.output.resolve();scripts=output/'source/tools'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(scripts),MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script,arguments,label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable,'-B',str(scripts/script),*map(str,arguments)],stdout=stream,stderr=subprocess.STDOUT,env=env,check=True)
    prepared=output/'recomputed_prepared';run('prepare_native_raw_map_queries.py',[*refs,prepared],'prepare')
    cpp=output/'replayed_native_queries.jsonl';cpp.write_bytes(args.cpp_output.read_bytes() if args.cpp_output else gzip.open(archive/'native_queries.jsonl.gz','rb').read())
    scalar=output/'recomputed_scalar.json';report=output/'recomputed_report.json';subset=output/'recomputed_subset.json';verification=output/'recomputed_verification.json'
    run('verify_native_raw_map_queries.py',[prepared,cpp,scalar],'scalar')
    run('analyze_native_raw_map_contract.py',[*refs,prepared,cpp,report],'analysis')
    run('verify_native_raw_map_subset.py',[*refs,report,subset],'cell_subset')
    run('verify_native_raw_map_report.py',[*refs,prepared,cpp,report,subset,verification],'bits_rational')
    run('test_native_raw_map_evidence.py',[*refs,prepared,cpp,report,subset,output/'recomputed_tests'],'evidence_probes')
    run('plot_native_raw_map_contract.py',[report,subset,cpp,output/'recomputed_figures'],'figures')
    mutation_source=None
    if args.mutation_verifier:
        frozen=scripts/'verify_raw_map_mutation_recipes.py'
        amended=output/'amended_tools/verify_raw_map_mutation_recipes.py';amended.parent.mkdir()
        shutil.copyfile(args.mutation_verifier,amended)
        mutation_source={'frozen_sha256':hashlib.sha256(frozen.read_bytes()).hexdigest(),
            'amended_sha256':hashlib.sha256(amended.read_bytes()).hexdigest(),
            'copied_source':'amended_tools/verify_raw_map_mutation_recipes.py'}
        mutation_script=amended
    else:mutation_script=Path('verify_raw_map_mutation_recipes.py')
    recipes=output/'recomputed_recipes.json';run(mutation_script,[archive,recipes],'mutation_recipes')
    checks={}
    for name in ['queries.bin','input_identity.json']:checks['prepared/'+name]=(prepared/name).read_bytes()==(archive/'prepared'/name).read_bytes()
    for path,name in [(scalar,'scalar_verification.json'),(report,'report.json'),(subset,'cell_subset.json')]:checks[name]=path.read_bytes()==gzip.open(archive/(name+'.gz'),'rb').read()
    checks['verification.json']=verification.read_bytes()==(archive/'verification.json').read_bytes()
    checks['evidence_probes']=(output/'recomputed_tests/verification.json').read_bytes()==(archive/'verification/evidence_tests/verification.json').read_bytes()
    for name in ['native_raw_map_contract.png','native_raw_map_contract.svg']:checks[name]=(output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    recipe_result=json.loads(recipes.read_text())
    recipe_pass=recipe_result['verdict']=='PASS' and len(recipe_result['checks'])==38 and all(recipe_result['checks'].values())
    if args.cpp_output:checks['new_actual_native_queries']=cpp.read_bytes()==gzip.open(archive/'native_queries.jsonl.gz','rb').read()
    if args.native_test_verification:checks['new_actual_native_probes']=args.native_test_verification.read_bytes()==(archive/'verification/native_tests/verification.json').read_bytes()
    result={'verdict':'PASS' if all(checks.values()) and recipe_pass else 'FAILED','byte_exact':checks,
        'mutation_recipes':{'verdict':'PASS' if recipe_pass else 'FAILED','hash_checks':len(recipe_result['checks'])},
        'mutation_verifier_amendment':mutation_source,
        'native_query_source':'new supplied SDK output' if args.cpp_output else 'original frozen SDK output',
        'native_input_probes':'new supplied compiled result' if args.native_test_verification else 'frozen proof only',
        'scope':'Original raw30/dense input, all300 conservative vector gates, ten exact cell paths, actual bits/rational partitions/probes/figures/recipes. No complete critic state or control acceptance.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) and recipe_pass else 1


if __name__=='__main__':raise SystemExit(main())
