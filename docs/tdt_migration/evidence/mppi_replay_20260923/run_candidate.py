#!/usr/bin/env python3
"""Isolated one-variable controller odom routing pilot; never edits old profiles."""
from pathlib import Path
import hashlib, importlib.util, json, subprocess, sys
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = ROOT / 'build/tdt_p2b/runs/dynamic_odom_routing_pilot_v1'
PREV = ROOT / 'docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922'
BASE = ROOT / 'build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2'

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p, value):
    with p.open('x') as f:
        json.dump(value, f, indent=2)
        f.write('\n')

def prepare():
    RUNS.mkdir(exist_ok=False)
    (RUNS/'profiles').mkdir()
    baseline = json.loads((BASE/'inputs.json').read_text())
    candidate = dict(baseline)
    candidate['source_commit'] = subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip()
    candidate['control_change'] = {'only_semantic_profile_delta':'controller_server.ros__parameters.odom_topic: /odometry/lio',
                                   'reason':'Nav2 1.1.20 controller_server default odom lacks publisher in fixture'}
    candidate['source_profiles'] = baseline['profiles']
    candidate['profiles'] = {}
    candidate['candidate_tool_sha256'] = sha(Path(__file__))
    candidate['scope'] = 'one-variable diagnostic pilot, fixed phase; not dynamic acceptance or deployment default'
    for name, digest in baseline['profiles'].items():
        source = BASE/'profiles'/f'{name}.yaml'
        assert sha(source) == digest
        before = source.read_text()
        token = 'controller_server:\n  ros__parameters:\n'
        assert before.count(token) == 1
        after = before.replace(token, token+'    odom_topic: /odometry/lio\n')
        a, b = yaml.safe_load(before), yaml.safe_load(after)
        assert 'odom_topic' not in a['controller_server']['ros__parameters']
        assert b['controller_server']['ros__parameters'].pop('odom_topic') == '/odometry/lio'
        assert a == b, 'unexpected semantic profile delta'
        dest = RUNS/'profiles'/f'{name}.yaml'
        dest.write_text(after)
        candidate['profiles'][name] = sha(dest)
    write(RUNS/'inputs.json', candidate)
    print(json.dumps({'series':str(RUNS),'source_profiles':candidate['source_profiles'],
                      'candidate_profiles':candidate['profiles']},indent=2))

def run(name):
    candidate = json.loads((RUNS/'inputs.json').read_text())
    assert candidate['candidate_tool_sha256'] == sha(Path(__file__))
    spec = importlib.util.spec_from_file_location('previous_run_experiment', PREV/'run_experiment.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.RUNS = RUNS
    return module.run(name)

if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'prepare': prepare()
    elif len(sys.argv) == 3 and sys.argv[1] == 'run': sys.exit(run(sys.argv[2]))
    else: raise SystemExit('usage: run_candidate.py prepare | run tdt_astar|tdt_qp')
