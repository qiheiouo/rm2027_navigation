#!/usr/bin/env python3
"""Rebuild frozen offline factor inputs, truth checks, scores and figures."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from replay_processed_cv import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cpp-output', type=Path)
    args = parser.parse_args()
    archive = args.archive.resolve()
    verify(archive)
    provenance = json.loads((archive/'provenance.json').read_text())
    references = {}
    for label in ['source', 'witness', 'raw_rollout', 'processed']:
        root = (archive/provenance[label+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() != provenance[label+'_manifest_sha256']:
            raise ValueError(label+' manifest identity mismatch')
        verify(root)
        references[label] = root
    shutil.copytree(archive, args.output)
    output = args.output.resolve()
    tools = output/'source/tools'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=str(tools), MPLCONFIGDIR=str(output/'mpl_config'))
    def run(script, arguments, label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable, '-B', str(tools/script), *map(str, arguments)],
                           stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)
    processed = output/'recomputed_processed'
    prepared = output/'recomputed_input'
    run('prepare_processed_cv.py', [references['source'], references['witness'], processed], 'processed')
    run('prepare_cv_support_factors.py', [references['source'], references['witness'], processed, prepared], 'prepare')
    checks = {}
    for name in ['input_identity.json', 'actual_input/input_identity.json']:
        checks['processed/'+name] = (processed/name).read_bytes()==(references['processed']/'prepared'/name).read_bytes()
    for name in ['actual_input/input.bin', 'processed_paths.bin']:
        checks['processed/'+name] = (processed/name).read_bytes()==gzip.open(references['processed']/'prepared'/(name+'.gz'), 'rb').read()
    checks['input_identity.json'] = (prepared/'input_identity.json').read_bytes()==(archive/'prepared/input_identity.json').read_bytes()
    for name in ['actual_input.bin', 'factor_input.bin']:
        checks[name] = (prepared/name).read_bytes()==gzip.open(archive/'prepared'/(name+'.gz'), 'rb').read()
    truth = output/'recomputed_truth.json'
    run('verify_cv_factor_truth.py', [references['source'], prepared, truth], 'truth')
    checks['truth_verification.json'] = truth.read_bytes()==(archive/'truth_verification.json').read_bytes()
    run('test_cv_factor_truth.py', [references['source'], prepared, output/'recomputed_truth_tests'], 'truth_negative')
    checks['truth_negative_verification.json'] = (output/'recomputed_truth_tests/verification.json').read_bytes()==(archive/'verification/truth_tests/verification.json').read_bytes()
    frozen_cpp = gzip.open(archive/'factors.jsonl.gz', 'rb').read()
    cpp = args.cpp_output.resolve() if args.cpp_output else output/'frozen_factors.jsonl'
    if args.cpp_output:
        checks['independent_CPP_factor_output'] = cpp.read_bytes()==frozen_cpp
    else:
        cpp.write_bytes(frozen_cpp)
    analysis = output/'recomputed_analysis.json'
    run('analyze_cv_support_factors.py', [references['source'], references['raw_rollout'], references['processed'], prepared, cpp, analysis], 'analysis')
    checks['analysis.json'] = analysis.read_bytes()==gzip.open(archive/'analysis.json.gz', 'rb').read()
    run('plot_cv_support_factors.py', [analysis, prepared/'input_identity.json', output/'recomputed_figures'], 'figures')
    for name in ['cv_support_factors.png', 'cv_support_factors.svg']:
        checks[name] = (output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    result = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'byte_exact': checks,
              'cpp_scope': 'independently re-executed' if args.cpp_output else 'frozen evaluated output',
              'scope': 'Offline factor integrity only; all original FAILED/NOT ESTABLISHED and inadmissible-oracle limits remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
