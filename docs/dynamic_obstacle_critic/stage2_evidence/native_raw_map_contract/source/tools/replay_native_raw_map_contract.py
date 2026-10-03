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
    parser.add_argument('--cpp-output',type=Path);parser.add_argument('--native-test-verification',type=Path);args=parser.parse_args()
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
    recipes=output/'recomputed_recipes.json';run('verify_raw_map_mutation_recipes.py',[archive,recipes],'mutation_recipes')
    checks={}
    for name in ['queries.bin','input_identity.json']:checks['prepared/'+name]=(prepared/name).read_bytes()==(archive/'prepared'/name).read_bytes()
    for path,name in [(scalar,'scalar_verification.json'),(report,'report.json'),(subset,'cell_subset.json')]:checks[name]=path.read_bytes()==gzip.open(archive/(name+'.gz'),'rb').read()
    checks['verification.json']=verification.read_bytes()==(archive/'verification.json').read_bytes()
    checks['evidence_probes']=(output/'recomputed_tests/verification.json').read_bytes()==(archive/'verification/evidence_tests/verification.json').read_bytes()
    for name in ['native_raw_map_contract.png','native_raw_map_contract.svg']:checks[name]=(output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    checks['mutation_recipes']=json.loads(recipes.read_text())['verdict']=='PASS'
    if args.cpp_output:checks['new_actual_native_queries']=cpp.read_bytes()==gzip.open(archive/'native_queries.jsonl.gz','rb').read()
    if args.native_test_verification:checks['new_actual_native_probes']=args.native_test_verification.read_bytes()==(archive/'verification/native_tests/verification.json').read_bytes()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','byte_exact':checks,
        'native_query_source':'new supplied SDK output' if args.cpp_output else 'original frozen SDK output',
        'native_input_probes':'new supplied compiled result' if args.native_test_verification else 'frozen proof only',
        'scope':'Original raw30/dense input, all300 conservative vector gates, ten exact cell paths, actual bits/rational partitions/probes/figures/recipes. No complete critic state or control acceptance.'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
