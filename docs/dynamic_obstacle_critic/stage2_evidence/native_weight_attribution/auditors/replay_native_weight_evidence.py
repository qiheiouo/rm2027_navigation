#!/usr/bin/env python3
"""Verify source archives and reproduce frozen weight/label analysis privately."""
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
    for name, digest in json.loads((root / 'manifest.json').read_text())['files'].items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('archive identity mismatch: ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--native-outputs', type=Path)
    args = parser.parse_args()
    archive = args.archive.resolve()
    verify(archive)
    metadata = json.loads((archive / 'provenance.json').read_text())
    sources = {}
    for name, identity in metadata['source_archives'].items():
        source = (archive / identity['relative_path']).resolve()
        if hashlib.sha256((source / 'manifest.json').read_bytes()).hexdigest() != identity['manifest_sha256']:
            raise ValueError('source manifest mismatch: ' + name)
        verify(source)
        sources[name] = source
    shutil.copytree(archive, args.output)
    output = args.output.resolve()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', MPLCONFIGDIR=str(output / 'mpl_config'))
    results = {}

    def run(script, arguments, label):
        with (output / (label + '.replay.log')).open('x') as stream:
            subprocess.run([sys.executable, '-B', str(output / 'auditors' / script),
                            *map(str, arguments)], stdout=stream, stderr=subprocess.STDOUT,
                           env=env, check=True)

    prepared = output / 'native_input'
    run('prepare_native_optimizer_replay.py', [sources['physical'], prepared, '--association-window', '.05'], 'prepare')
    results['native_input_sha256'] = hashlib.sha256((prepared / 'input.bin').read_bytes()).hexdigest() == metadata['input_sha256']
    baseline = sources['physical'] / 'native_optimizer/sdk_final_output.jsonl.gz'
    labels = sources['witness'] / 'selected/analysis.json.gz'
    for label in ['native', 'uniform']:
        audit = archive / (label + '_output.jsonl.gz')
        if args.native_outputs:
            audit = args.native_outputs / (label + '_output.jsonl')
            results[label + '_cpp_output'] = audit.read_bytes() == gzip.open(archive / (label + '_output.jsonl.gz'), 'rb').read()
        report = output / (label + '_analysis.replayed.json')
        run('analyze_native_weights.py', [audit, baseline, labels, report], label + '_analysis')
        results[label + '_analysis.json'] = report.read_bytes() == (archive / (label + '_analysis.json')).read_bytes()
    native = args.native_outputs / 'native_output.jsonl' if args.native_outputs else archive / 'native_output.jsonl.gz'
    run('plot_native_weights.py', [native, output / 'native_analysis.replayed.json', labels,
                                   output / 'native_weight_distribution'], 'figure')
    for extension in ['png', 'svg']:
        name = 'native_weight_distribution.' + extension
        results[name] = (output / name).read_bytes() == (archive / name).read_bytes()
    result = {'verdict': 'PASS' if all(results.values()) else 'FAILED', 'byte_exact': results,
              'scope': 'source identity, regenerated native input, weight analysis and figures; physical FAILED remains',
              'native_cpp_outputs': 'independently re-executed' if args.native_outputs else 'saved evidence only'}
    (output / 'replay_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
