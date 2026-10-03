#!/usr/bin/env python3
"""Replay the interface preflight from frozen source on an installed Humble SDK.

Build frozen source first with the documented /repo and /out mounts. This tool
checks installed source/ELF identity, reexecutes tests, and verifies semantic CDR
bytes; unspecified padding, timing and guard callback counts are not byte gates.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


def verify(root):
    manifest = json.loads((root/'manifest.json').read_text())
    actual = {str(path.relative_to(root)) for path in root.rglob('*') if path.is_file()
              and path != root/'manifest.json'}
    if actual != set(manifest['files']):
        raise ValueError('archive file set mismatch')
    for name, digest in manifest['files'].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('archive identity mismatch: '+name)


def xml_counts(path):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == 'testsuite' else list(root.iter('testsuite'))
    result = {key: sum(int(suite.get(key, '0')) for suite in suites)
              for key in ('tests', 'errors', 'failures', 'skipped')}
    if result['errors'] or result['failures'] or result['skipped']:
        raise ValueError('incomplete tests: '+str(path))
    return result['tests']


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('archive', type=Path)
    parser.add_argument('new_output', type=Path); parser.add_argument('build', type=Path)
    parser.add_argument('install', type=Path); args = parser.parse_args()
    archive = args.archive.resolve(); verify(archive)
    args.new_output.mkdir(parents=True, exist_ok=False); output = args.new_output.resolve()
    source = archive/'source/src'
    tracker = source/'rm_dynamic_obstacle_tracking'
    tools = source/'rm_dynamic_obstacle_critic/tools'
    provenance = json.loads((archive/'provenance.json').read_text())
    checks = {}
    import rm_dynamic_obstacle_tracking.core as installed_core
    import rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node as installed_node
    for module, name in [(installed_core, 'core.py'), (installed_node, 'dynamic_obstacle_tracker_node.py')]:
        checks['installed_'+name] = Path(module.__file__).read_bytes() == (
            tracker/'rm_dynamic_obstacle_tracking'/name).read_bytes()
    for name, expected in provenance['runtime_elf_sha256'].items():
        path = args.install/name
        checks['runtime_elf:'+name] = hashlib.sha256(path.read_bytes()).hexdigest() == expected
    if not all(checks.values()): raise ValueError('installed frozen source or runtime ELF mismatch')
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    env['PYTHONPATH'] = str(tracker)+':'+str(tools)+':'+env.get('PYTHONPATH', '')
    def run(command, label):
        with (output/(label+'.log')).open('x') as stream:
            subprocess.run(list(map(str, command)), stdout=stream, stderr=subprocess.STDOUT,
                           env=env, check=True)
    run([sys.executable, '-B', tracker/'tools/audit_observation_anchor_core.py',
        archive/'reference/core_41e96399.py', output/'default_core_regression.json'], 'core')
    checks['default_core_report_byte_exact'] = (output/'default_core_regression.json').read_bytes() == (
        archive/'verification/default_core_regression.json').read_bytes()
    run([sys.executable, '-B', tracker/'tools/audit_default_prediction_wire.py',
        archive/'reference/node_41e96399.py', output/'default_wire_regression.json'], 'wire')
    new_wire = json.loads((output/'default_wire_regression.json').read_text())
    old_wire = json.loads((archive/'verification/default_wire_regression.json').read_text())
    for key in ('status', 'canonical_wire_trace_sha256', 'frames', 'coasting_tracks',
                'truncated_frames', 'serialized_fields', 'structural_padding_bytes_examined'):
        checks['wire_semantic:'+key] = new_wire[key] == old_wire[key]
    run([sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', tracker/'test',
         '--basetemp='+str(output/'ros_fixtures'), '--junitxml='+str(output/'tracker.xml')], 'tracker')
    checks['tracker_test_count'] = xml_counts(output/'tracker.xml') == provenance['tracker_tests']
    def observations(root):
        return {json.loads(path.read_text())['mode']: json.loads(path.read_text())
                for path in root.rglob('observations.json')}
    saved = observations(archive/'verification/ros_fixtures')
    replayed = observations(output/'ros_fixtures')
    checks['dds_modes'] = set(saved) == set(replayed) == {'filtered', 'last_observation_cv'}
    for mode in saved:
        checks['dds_observations:'+mode] = replayed[mode] == saved[mode]
    for name in ('test_conditional_cv_support.py', 'test_conditional_observation_support.py'):
        run([sys.executable, '-B', tools/name], name)
        content = (output/(name+'.log')).read_text()
        expected = 8 if name == 'test_conditional_cv_support.py' else 6
        checks['math:'+name] = f'Ran {expected} tests in ' in content and content.rstrip().endswith('OK')
    for script, directory, marker in [('test_guard_runtime.py', 'default_guard', 'verdict'),
                                      ('test_observation_anchor_guard.py', 'observation_guard', 'status')]:
        run([sys.executable, '-B', tools/script, output/directory], script)
        report = json.loads((output/directory/'summary.json').read_text())
        checks[directory] = report[marker] == 'PASS' and len(report['cases']) == 7 and all(
            row['passed'] for row in report['cases'])
    # Execute the four newly built native GTest binaries, not saved XML.
    native_count = 0
    for name in ('test_model', 'test_plugin', 'test_cost_accumulator_evidence', 'test_cost_accumulator_plugin'):
        run([args.build/'rm_dynamic_obstacle_critic'/name, '--gtest_output=xml:'+str(output/(name+'.xml'))], name)
        native_count += xml_counts(output/(name+'.xml'))
    checks['native_test_count'] = native_count == provenance['native_tests']
    report = dict(schema=1, status='PASS' if all(checks.values()) else 'FAILED', checks=checks,
        archive_manifest_sha256=hashlib.sha256((archive/'manifest.json').read_bytes()).hexdigest(),
        scope='Reexecuted interface/preflight tests only. CDR padding, callback counts and test timings are not byte gates; physical/task failures are unchanged.')
    (output/'replay_verification.json').write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report, sort_keys=True)); return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
