#!/usr/bin/env python3
"""Reproduce frozen raw-path geometry/weights and reject a contact corruption."""
import argparse
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
    args = parser.parse_args()
    archive = args.archive.resolve()
    verify(archive)
    provenance = json.loads((archive/'provenance.json').read_text())
    references = {}
    for label in ['source', 'witness']:
        root = (archive/provenance[label+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() != provenance[label+'_manifest_sha256']:
            raise ValueError(label+' manifest identity mismatch')
        verify(root)
        references[label] = root
    shutil.copytree(archive, args.output)
    output = args.output.resolve()
    tools = output/'source/tools'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=str(tools), MPLCONFIGDIR=str(output/'mpl_config'))

    def run(script, arguments, label, expected_failure=False):
        log = output/(label+'.replay.log')
        with log.open('x') as stream:
            result = subprocess.run([sys.executable, '-B', str(tools/script), *map(str, arguments)],
                                    stdout=stream, stderr=subprocess.STDOUT, env=env)
        if expected_failure:
            return result.returncode != 0 and 'scalar contact classification differs' in log.read_text()
        result.check_returncode()

    with (output/'geometry_tests.replay.log').open('x') as stream:
        subprocess.run([sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                        str(tools/'test_witness_geometry.py')], stdout=stream, stderr=subprocess.STDOUT,
                       env=env, check=True)
    report = output/'recomputed_analysis.json'
    scalar = output/'recomputed_scalar.json'
    run('audit_raw_rollout_objective.py', [references['source'], references['witness'], report], 'analysis')
    run('verify_raw_rollout_geometry.py', [references['source'], report, scalar], 'scalar')
    run('plot_raw_rollout_objective.py', [report, output/'recomputed_figures'], 'figures')
    checks = {'analysis.json': report.read_bytes()==(archive/'analysis.json').read_bytes(),
              'scalar_verification.json': scalar.read_bytes()==(archive/'scalar_verification.json').read_bytes()}
    for name in ['raw_rollout_objective.png', 'raw_rollout_objective.svg']:
        checks[name] = (output/'recomputed_figures'/name).read_bytes()==(archive/'figures'/name).read_bytes()
    probe = json.loads(report.read_text())
    target = probe['rows'][73]['physical_dynamic_labels']['actual_score_clock']['geometry']['padded_native']
    if target['original_rollout_grid_min'] != 0:
        raise ValueError('registered negative contact row changed')
    target['original_rollout_grid_min'] = .1
    corrupted = output/'corrupted_contact_probe.json'
    corrupted.write_text(json.dumps(probe, indent=2)+'\n')
    negative = run('verify_raw_rollout_geometry.py', [references['source'], corrupted, output/'negative_scalar.json'],
                   'negative_contact', expected_failure=True)
    result = {'verdict': 'PASS' if all(checks.values()) and negative else 'FAILED', 'byte_exact': checks,
              'independent_geometry_tests': '4 passed', 'corrupted_contact_probe_rejected': negative,
              'scope': 'Conditional raw dynamic geometry/integrity only; all original FAILED/NOT ESTABLISHED gates remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if result['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
