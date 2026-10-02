#!/usr/bin/env python3
"""Independently reproduce an extent-prior archive using its frozen tools."""
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
    for label in ['source', 'manual_review']:
        root = (archive/provenance[label+'_archive']).resolve()
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() != provenance[label+'_manifest_sha256']:
            raise ValueError(label+' manifest identity mismatch')
        verify(root)
        references[label] = root
    shutil.copytree(archive, args.output)
    output = args.output.resolve()
    tools = output/'source/tools'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', MPLCONFIGDIR=str(output/'mpl_config'))

    def run(script, arguments, label):
        with (output/(label+'.replay.log')).open('x') as stream:
            subprocess.run([sys.executable, '-B', str(tools/script), *map(str, arguments)],
                           stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)

    run('test_robot_extent_prior.py', [], 'geometry_tests')
    report = output/'recomputed_analysis.json'
    run('audit_ground_extent_prior.py', [references['source'], output/'source/config/ground_robot_extent_prior_offline.yaml', report], 'analysis')
    run('plot_ground_extent_prior.py', [report, output/'recomputed_figures'], 'figures')
    legacy = output/'recomputed_legacy_cv.json'
    run('audit_consumed_cv_support.py', [references['source'], legacy], 'legacy_default')
    checks = {'analysis.json': report.read_bytes() == (archive/'analysis.json').read_bytes(),
              'legacy_CV_default': legacy.read_bytes() == (references['source']/'consumed_cv_support_audit.json').read_bytes()}
    for name in ['ground_extent_prior.png', 'ground_extent_prior.svg']:
        checks[name] = (output/'recomputed_figures'/name).read_bytes() == (archive/'figures'/name).read_bytes()
    result = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'byte_exact': checks,
              'geometry_tests': '8 passed', 'scope': 'Offline geometry/reference integrity only; original physical and task verdicts remain.'}
    (output/'replay_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
