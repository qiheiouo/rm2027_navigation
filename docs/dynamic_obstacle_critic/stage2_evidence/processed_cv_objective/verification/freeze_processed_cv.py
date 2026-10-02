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

repo = Path('/home/qihei/rm2027_navigation')
base = Path('/tmp/rm_dynamic_critic_v1')
out = repo/'docs/dynamic_obstacle_critic/stage2_evidence/processed_cv_objective'
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
pending = ['prepare_processed_cv', 'analyze_processed_cv', 'plot_processed_cv',
           'replay_processed_cv', 'test_processed_cv_evaluator']
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
for name in ['dynamic_replay_input.hpp', 'replay_dynamic_consumption.cpp', 'evaluate_processed_cv.cpp']:
    copy(tools/name, out/'source/tools'/name)
for name in ['model.hpp', 'geometry.hpp']:
    copy(repo/'src/rm_dynamic_obstacle_critic/include/rm_dynamic_obstacle_critic'/name, out/'source/include/rm_dynamic_obstacle_critic'/name)
copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt', out/'source/CMakeLists.txt')
for name in ['input_identity.json', 'actual_input/input_identity.json']:
    copy(base/'processed_cv_input'/name, out/'prepared'/name)
for name in ['processed_paths.bin', 'actual_input/input.bin']:
    copy(base/'processed_cv_input'/name, out/'prepared'/(name+'.gz'), True)
copy(base/'processed_cv_analysis.json', out/'analysis.json')
copy(base/'processed_cv_scalar_verification.json', out/'scalar_verification.json')
copy(base/'processed_cv_evaluator_tests/processed_cv.jsonl', out/'processed_cv.jsonl')
for path in (base/'processed_cv_figures_final').iterdir():
    copy(path, out/'figures'/path.name)
for path in (base/'processed_cv_figures').iterdir():
    copy(path, out/'verification/initial_figures'/path.name)
for name in ['processed_cv_offline_build.log', 'processed_cv_prepare.log', 'processed_cv_analysis.log',
             'processed_cv_plot.log', 'processed_cv_plot_final.log', 'processed_cv_evaluator_tests.log',
             'processed_cv_runtime_before.txt', 'processed_cv_runtime_after.txt',
             'processed_cv_default_prepare_regression.log', 'processed_cv_default_prepare_verification.json',
             'processed_cv_legacy_scalar_regression.log', 'processed_cv_compatibility_verification.json']:
    copy(base/name, out/'verification'/name)
for name in ['evaluate_processed_cv', 'replay_dynamic_consumption']:
    copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/'flags.make', out/'verification'/(name+'_flags.make'))
tests = base/'processed_cv_evaluator_tests'
for path in tests.iterdir():
    if path.suffix in ['.log', '.json']:
        copy(path, out/'verification/evaluator_tests'/path.name)
    elif path.suffix == '.bin':
        copy(path, out/'verification/evaluator_tests'/(path.name+'.gz'), True)
for folder in (base/'dynamic_consumption_fixture_replay').iterdir():
    if folder.is_dir():
        for path in folder.iterdir():
            copy(path, out/'verification/original_fixture_inputs'/folder.name/(path.name+'.gz'), True)
full = tests/'full_raw_regression.jsonl'
(out/'verification/full_raw_replay_identity.json').write_text(json.dumps({'source': 'referenced physical archive dynamic_replay/native_risk.jsonl.gz',
    'reexecuted_sha256': hashlib.sha256(full.read_bytes()).hexdigest(),
    'frozen_decompressed_sha256': hashlib.sha256(gzip.open(out.parent/'gazebo_dynamic_consumption_evidence/dynamic_replay/native_risk.jsonl.gz', 'rb').read()).hexdigest(),
    'whole_output_byte_exact': True, 'raw_risk_and_after_rows': 104400}, indent=2)+'\n')
copy(Path(__file__), out/'verification/freeze_processed_cv.py')
provenance = {'runtime_source_checkpoint': '9f73d72', 'preregistration_checkpoint': '42d5213',
    'native_source_ordinal': 298, 'processed_rows': 311, 'runtime_parameters_changed': False,
    'tf_or_ros_interfaces_changed': False, 'original_task_raw203_and_age_verdict': 'FAILED',
    'full_witness_assessment': 'NOT ESTABLISHED', 'visual_review': 'Final PNG inspected; number labels have white background; earlier figure retained.',
    'limits': 'Explicit offline CV hypotheses, reconstructed SDK paths and conditional physical dynamic labels only.'}
for label, folder in [('source', 'gazebo_dynamic_consumption_evidence'), ('witness', 'dynamic_consumption_witness'), ('raw_rollout', 'raw_rollout_objective')]:
    provenance[label+'_archive'] = '../'+folder
    provenance[label+'_manifest_sha256'] = hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python_analysis': sys.version, 'matplotlib': matplotlib.__version__,
    'numpy': numpy.__version__, 'pyyaml': yaml.__version__, 'cpp_environment': 'existing rm2027_navigation:humble image; network none; no ROS initialization',
    'runtime_five_ELF_hashes_unchanged': True}, indent=2)+'\n')
files = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope': 'Processed CV integrity and conditional model/physical support mismatch; no acceptance.', 'files': files}, indent=2)+'\n')
print(json.dumps({'files': len(files), 'bytes': sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),
                  'manifest_sha256': hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(), 'python_tool_modules': sorted(included)}))
