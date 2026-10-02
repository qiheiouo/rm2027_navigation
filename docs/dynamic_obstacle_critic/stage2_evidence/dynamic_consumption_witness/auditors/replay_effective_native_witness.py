#!/usr/bin/env python3
"""Verify and reproduce measured-input witness evidence in a private directory."""
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
    provenance = json.loads((archive / 'provenance.json').read_text())
    source = (archive / provenance['source_archive']).resolve()
    if hashlib.sha256((source / 'manifest.json').read_bytes()).hexdigest() != provenance['source_manifest_sha256']:
        raise ValueError('source manifest identity mismatch')
    verify(source)
    shutil.copytree(archive, args.output)
    output = args.output.resolve()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    results = {}

    def run(script, arguments, label):
        with (output / (label + '.replay.log')).open('x') as stream:
            subprocess.run([sys.executable, '-B', str(output / 'auditors' / script),
                            *map(str, arguments)], stdout=stream, stderr=subprocess.STDOUT,
                           env=env, check=True)

    for label, ordinals in [('full', []), ('selected', ['--source-ordinals', str(provenance['selected_source_ordinal'])])]:
        prepared = output / (label + '_input')
        run('prepare_native_witness.py', [source, prepared, *ordinals], label + '_prepare')
        results[label + '_input_identity.json'] = (prepared / 'input_identity.json').read_bytes() == (archive / label / 'input_identity.json').read_bytes()
        report = output / (label + '_analysis.json')
        run('analyze_native_witness.py', [source, archive / label / 'witnesses.bin.gz',
                                        prepared / 'input_identity.json', report], label + '_analysis')
        results[label + '_analysis.json'] = report.read_bytes() == gzip.open(archive / label / 'analysis.json.gz', 'rb').read()
    selection = output / 'selected_cycle.json'
    run('select_native_witness_cycle.py', [source, output / 'full_analysis.json', selection], 'selection')
    results['selected_cycle.json'] = selection.read_bytes() == (archive / 'full/selected_cycle.json').read_bytes()
    assessment = output / 'contract_assessment.json'
    run('assess_effective_native_witness.py', [output / 'full_analysis.json', output / 'selected_analysis.json',
                                             source / 'effective_velocity_audit.json', assessment], 'assessment')
    results['contract_assessment.json'] = assessment.read_bytes() == (archive / 'full/contract_assessment.json').read_bytes()
    if (archive/'selected/components.json').exists():
        for script,name,arguments in [
            ('analyze_native_weights.py','weights.json',[source/'native_optimizer/weights_output.jsonl.gz',
                source/'native_optimizer/sdk_final_output.jsonl.gz',output/'selected_analysis.json',output/'weights.json']),
            ('analyze_dynamic_cost_labels.py','components.json',[source,source/'native_optimizer/weights_output.jsonl.gz',
                source/'native_optimizer/sdk_final_output.jsonl.gz',output/'selected_analysis.json',output/'components.json'])]:
            run(script,arguments,name)
            results[name]=(output/name).read_bytes()==(archive/'selected'/name).read_bytes()
    if args.native_outputs:
        for label, prefix in [('full', ''), ('selected', 'selected_')]:
            for name in ['witnesses.bin', 'witnesses.jsonl']:
                results[prefix + name] = (args.native_outputs / (prefix + name)).read_bytes() == gzip.open(archive / label / (name + '.gz'), 'rb').read()
    result = {'verdict': 'PASS' if all(results.values()) else 'FAILED', 'byte_exact': results,
              'native_cpp_outputs': 'independently re-executed' if args.native_outputs else 'frozen evidence only',
              'scope': 'manifest, input, analysis and conditional assessment integrity; original physical FAILED remains'}
    (output / 'replay_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
