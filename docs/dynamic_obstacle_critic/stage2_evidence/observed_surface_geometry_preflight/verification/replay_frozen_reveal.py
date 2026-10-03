import json, os, subprocess, sys
from pathlib import Path

repo = Path(__file__).resolve().parents[2]
archive = repo / 'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight'
tools = archive / 'source/src/rm_dynamic_obstacle_critic/tools'
output = Path(__file__).parent / 'frozen_reveal_reexecution'
output.mkdir(exist_ok=False)
checks = {}
for mode in ('visible', 'nominal'):
    trial = archive.parent / ('gazebo_observation_v2_' + mode)
    for label, script in [('reveal', 'audit_surface_reveal.py'), ('brake', 'audit_brake_response.py')]:
        name = mode + ('' if label == 'reveal' else '_brake')
        with (output / (name + '.log')).open('x') as log:
            subprocess.run([sys.executable, '-B', str(tools / script), str(trial), str(output / (name + '.json'))], stdout=log, stderr=subprocess.STDOUT, check=True)
        checks[name] = (archive / 'initial_execution' / (name + '.json')).read_bytes() == (output / (name + '.json')).read_bytes()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', MPLCONFIGDIR=str(output / 'mpl_config'))
    with (output / (mode + '_plot.log')).open('x') as log:
        subprocess.run([sys.executable, '-B', str(tools / 'plot_surface_reveal.py'), str(output / (mode + '.json')), str(output / (mode + '_reveal')), '--title', f'V2 {mode}: sensor reveal labels; actual braking/replanning margin UNKNOWN'], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    for ext in ('png', 'svg'):
        checks[mode + '_plot_' + ext] = (archive / 'initial_execution' / (mode + '_reveal.' + ext)).read_bytes() == (output / (mode + '_reveal.' + ext)).read_bytes()
report = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'checks': checks, 'scope': 'Fresh execution from frozen 0d7dd31c source; guaranteed real stopping/response remains UNKNOWN'}
(output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report))
if not all(checks.values()):
    raise SystemExit(1)
