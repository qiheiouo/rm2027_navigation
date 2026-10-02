import ast
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
import matplotlib
import numpy
import yaml
import pytest

repo = Path('/home/qihei/rm2027_navigation')
base = Path('/tmp/rm_dynamic_critic_v1')
out = repo/'docs/dynamic_obstacle_critic/stage2_evidence/raw_rollout_objective'
out.mkdir()

def copy(source, target, compress=False):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as writer:
                writer.write(source.read_bytes())
    else:
        shutil.copyfile(source, target)

tools = repo/'src/rm_dynamic_obstacle_critic/tools'
pending = ['audit_raw_rollout_objective', 'verify_raw_rollout_geometry', 'plot_raw_rollout_objective',
           'replay_raw_rollout_objective', 'test_witness_geometry']
included = set()
while pending:
    name = pending.pop()
    if name in included:
        continue
    included.add(name)
    path = tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        names = [node.module] if isinstance(node, ast.ImportFrom) else [n.name for n in node.names] if isinstance(node, ast.Import) else []
        for module in names:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():
                pending.append(module.split('.')[0])
    copy(path, out/'source/tools'/path.name)
copy(base/'raw_rollout_objective_analysis_complete.json', out/'analysis.json')
copy(base/'raw_rollout_objective_scalar_verification.json', out/'scalar_verification.json')
for path in (base/'raw_rollout_objective_figures_final').iterdir():
    copy(path, out/'figures'/path.name)
for path in (base/'raw_rollout_objective_figures').iterdir():
    copy(path, out/'verification/initial_figures'/path.name)
for name in ['raw_rollout_objective_analysis.log', 'raw_rollout_objective_analysis_complete.log',
             'raw_rollout_objective_geometry_tests.log', 'raw_rollout_objective_scalar_verification.log',
             'raw_rollout_objective_plot.log', 'raw_rollout_objective_plot_final.log',
             'raw_rollout_objective_corrupted_contact.log']:
    copy(base/name, out/'verification'/name)
copy(base/'raw_rollout_objective_corrupted_contact.json', out/'verification/corrupted_contact.json.gz', True)
copy(Path(__file__), out/'verification/freeze_raw_rollout_objective.py')
source = out.parent/'gazebo_dynamic_consumption_evidence'
witness = out.parent/'dynamic_consumption_witness'
provenance = {'source_archive': '../gazebo_dynamic_consumption_evidence',
    'source_manifest_sha256': hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
    'witness_archive': '../dynamic_consumption_witness',
    'witness_manifest_sha256': hashlib.sha256((witness/'manifest.json').read_bytes()).hexdigest(),
    'runtime_source_checkpoint': '9f73d72', 'preregistration_checkpoint': '96bbf4c',
    'native_source_ordinal': 298, 'raw_candidates': 300,
    'scope': 'Original raw-path dynamic model/physical geometry, offline fixture labels only.',
    'runtime_parameters_changed': False, 'tf_or_ros_interfaces_changed': False,
    'public_message_changes': 'Comments only; all non-comment fields/types verified unchanged.',
    'original_task_and_raw203_verdict': 'FAILED', 'full_trace_velocity_gate': 'FAILED',
    'full_witness_assessment': 'NOT ESTABLISHED',
    'visual_review': 'Final PNG inspected; initial annotation spill fixed and prior figures retained.',
    'negative_probe': 'row73 padded grid distance changed from 0 to .1; independent scalar verifier rejects contact classification mismatch.'}
(out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python': sys.version, 'matplotlib': matplotlib.__version__,
    'numpy': numpy.__version__, 'pyyaml': yaml.__version__, 'pytest': pytest.__version__,
    'scalar_vector_geometry_comparison_only': 2e-12}, indent=2)+'\n')
files = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
         for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope': 'Same raw path model/physical mismatch; no safety or optimizer certification.', 'files': files}, indent=2)+'\n')
print(json.dumps({'files': len(files), 'tool_modules': sorted(included),
                  'bytes': sum(path.stat().st_size for path in out.rglob('*') if path.is_file()),
                  'manifest_sha256': hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}))
