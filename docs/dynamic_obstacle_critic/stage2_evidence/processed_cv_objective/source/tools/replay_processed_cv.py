#!/usr/bin/env python3
"""Recompute explicit processed-path CV evidence and conditional geometry."""
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
    for name, digest in json.loads((root/'manifest.json').read_text())['files'].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('archive identity mismatch: '+name)


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
    for label in ['source', 'witness', 'raw_rollout']:
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
    prepared = output/'recomputed_input'
    run('prepare_processed_cv.py', [references['source'], references['witness'], prepared], 'prepare')
    checks = {}
    for name in ['input_identity.json', 'actual_input/input_identity.json']:
        checks[name] = (prepared/name).read_bytes()==(archive/'prepared'/name).read_bytes()
    for name in ['actual_input/input.bin', 'processed_paths.bin']:
        checks[name] = (prepared/name).read_bytes()==gzip.open(archive/'prepared'/(name+'.gz'), 'rb').read()
    cpp = args.cpp_output.resolve() if args.cpp_output else archive/'processed_cv.jsonl'
    if args.cpp_output:
        checks['independent_CPP_processed_output'] = cpp.read_bytes()==(archive/'processed_cv.jsonl').read_bytes()
    analysis, scalar = output/'recomputed_analysis.json', output/'recomputed_scalar.json'
    run('analyze_processed_cv.py', [references['source'], references['witness'], prepared, cpp, analysis, scalar], 'analysis')
    checks['analysis.json'] = analysis.read_bytes()==(archive/'analysis.json').read_bytes()
    checks['scalar_verification.json'] = scalar.read_bytes()==(archive/'scalar_verification.json').read_bytes()
    run('plot_processed_cv.py', [analysis, output/'recomputed_figures'], 'figures')
    for name in ['processed_cv.png', 'processed_cv.svg']:
        checks[name] = (output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    legacy = output/'legacy_raw_scalar.json'
    run('verify_raw_rollout_geometry.py', [references['source'], references['raw_rollout']/'analysis.json', legacy], 'legacy_scalar')
    checks['legacy_raw_scalar_default'] = legacy.read_bytes()==(references['raw_rollout']/'scalar_verification.json').read_bytes()
    result = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'byte_exact': checks,
              'cpp_scope': 'independently re-executed' if args.cpp_output else 'frozen evaluated output',
              'scope': 'Processed input/geometry/score integrity; all existing FAILED/NOT ESTABLISHED and dynamic-only limits remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if result['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
