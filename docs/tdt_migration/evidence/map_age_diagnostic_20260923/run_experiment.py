#!/usr/bin/env python3
"""New isolated diagnostic series: odom-corrected profile plus costmap timestamp trace."""
from pathlib import Path
import hashlib, importlib.util, json, subprocess, sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / 'build/tdt_p2b'
RUNS = WORK / 'runs/dynamic_map_age_pilot_v1'
BASE = WORK / 'runs/dynamic_odom_routing_pilot_v1'
PREV = ROOT / 'docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922'
LIB = WORK / 'map_age_diagnostic_v1/install/nav2_costmap_2d/lib'

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p, obj):
    with p.open('x') as f:
        json.dump(obj, f, indent=2)
        f.write('\n')

def prepare():
    assert sha(WORK/'mppi_cycle_diagnostic_v1/navigation2-1.1.20.tar.gz') == 'c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6'
    assert LIB.is_dir()
    RUNS.mkdir(exist_ok=False)
    (RUNS/'profiles').mkdir()
    old = json.loads((BASE/'inputs.json').read_text())
    new = dict(old)
    new['source_commit'] = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    new['scope'] = 'fixed-phase map-update-age diagnostic; no acceptance matrix or deployment claim'
    new['map_age_instrumentation'] = {'upstream':'nav2_costmap_2d 1.1.20',
        'source_inputs_sha256':sha(HERE/'build_inputs.json'),
        'only_runtime_delta':'passive independent-prefix costmap trace'}
    new['diagnostic_tools'] = dict(old['diagnostic_tools'])
    for p in HERE.iterdir():
        if p.suffix in ('.py','.sh','.hpp','.patch'):
            new['diagnostic_tools'][str(p.relative_to(ROOT))] = sha(p)
    new['diagnostic_libraries'] = dict(old['diagnostic_libraries'])
    libs = list(LIB.glob('libnav2_costmap_2d*.so'))
    assert len(libs) >= 2, libs
    for p in libs:
        new['diagnostic_libraries'][str(p.relative_to(ROOT))] = sha(p)
    new['profiles'] = {}
    new['source_profiles'] = old['profiles']
    for name, digest in old['profiles'].items():
        source = BASE/'profiles'/f'{name}.yaml'
        assert sha(source) == digest
        dest = RUNS/'profiles'/source.name
        dest.write_bytes(source.read_bytes())
        new['profiles'][name] = sha(dest)
    write(RUNS/'inputs.json',new)
    print(json.dumps({'series':str(RUNS),'profiles':new['profiles'],'costmap_libraries':len(libs)},indent=2))

def run(name):
    if name not in ('tdt_astar','tdt_qp'): raise ValueError(name)
    m = json.loads((RUNS/'inputs.json').read_text())
    assert m['map_age_instrumentation']['source_inputs_sha256'] == sha(HERE/'build_inputs.json')
    spec = importlib.util.spec_from_file_location('cycle_run_experiment', PREV/'run_experiment.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.RUNS = RUNS
    module.HERE = HERE
    return module.run(name)

if __name__ == '__main__':
    if len(sys.argv)==2 and sys.argv[1]=='prepare':prepare()
    elif len(sys.argv)==3 and sys.argv[1]=='run':sys.exit(run(sys.argv[2]))
    else:raise SystemExit('usage: run_experiment.py prepare | run tdt_astar|tdt_qp')
