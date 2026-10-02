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
out = repo/'docs/dynamic_obstacle_critic/stage2_evidence/cv_support_factors'
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
pending = ['prepare_cv_support_factors', 'analyze_cv_support_factors', 'plot_cv_support_factors',
           'verify_cv_factor_truth', 'test_cv_factor_truth', 'replay_cv_support_factors', 'test_cv_support_factors']
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
for name in ['dynamic_replay_input.hpp', 'evaluate_cv_support_factors.cpp']:
    copy(tools/name, out/'source/tools'/name)
for name in ['model.hpp', 'geometry.hpp']:
    copy(repo/'src/rm_dynamic_obstacle_critic/include/rm_dynamic_obstacle_critic'/name, out/'source/include/rm_dynamic_obstacle_critic'/name)
copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt', out/'source/CMakeLists.txt')
copy(base/'cv_support_factor_input/input_identity.json', out/'prepared/input_identity.json')
for name in ['actual_input.bin', 'factor_input.bin']:
    copy(base/'cv_support_factor_input'/name, out/'prepared'/(name+'.gz'), True)
copy(base/'cv_support_factor_analysis.json', out/'analysis.json.gz', True)
copy(base/'cv_support_factor_truth_verification_final.json', out/'truth_verification.json')
copy(base/'cv_support_factor_evaluator_tests_complete/factors.jsonl', out/'factors.jsonl.gz', True)
for folder, target in [('cv_support_factor_figures_final', 'figures'), ('cv_support_factor_figures', 'verification/initial_figures')]:
    for path in (base/folder).iterdir():
        copy(path, out/target/path.name)
for name in ['cv_support_factor_build.log', 'cv_support_factor_prepare.log', 'cv_support_factor_analysis.log',
             'cv_support_factor_plot.log', 'cv_support_factor_plot_final.log', 'cv_support_factor_evaluator_tests.log',
             'cv_support_factor_evaluator_tests_complete.log', 'cv_support_factor_truth_verification.log',
             'cv_support_factor_truth_verification_final.log', 'cv_support_factor_truth_tests.log',
             'cv_support_factor_binaries_before.txt', 'cv_support_factor_binaries_after.txt']:
    copy(base/name, out/'verification'/name)
copy(base/'cv_support_factor_evaluator_tests/verification.json', out/'verification/initial_evaluator_verification.json')
copy(base/'cv_support_factor_truth_verification.json', out/'verification/initial_truth_verification.json')
copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles/evaluate_cv_support_factors.dir/flags.make', out/'verification/evaluate_cv_support_factors_flags.make')
for path in (base/'cv_support_factor_evaluator_tests_complete').iterdir():
    if path.name=='factors.jsonl':
        continue
    compress = path.suffix in ['.bin', '.jsonl']
    copy(path, out/'verification/evaluator_tests'/(path.name+('.gz' if compress else '')), compress)
for path in (base/'cv_support_factor_truth_tests').rglob('*'):
    if path.is_file():
        compress = path.suffix=='.bin'
        target = str(path.relative_to(base/'cv_support_factor_truth_tests'))+('.gz' if compress else '')
        copy(path, out/'verification/truth_tests'/target, compress)
copy(Path(__file__), out/'verification/freeze_cv_support_factors.py')
provenance = {'runtime_source_checkpoint': '9f73d72', 'preregistration_checkpoint': '7b11884',
    'native_source_ordinal': 298, 'factor_count': 4, 'raw_rows_per_factor': 300, 'processed_rows_per_factor': 311,
    'runtime_parameters_changed': False, 'tf_or_ros_interfaces_changed': False,
    'original_task_raw203_and_age_verdict': 'FAILED', 'full_witness_assessment': 'NOT ESTABLISHED',
    'physical_motion_bound': 'unknown',
    'visual_review': 'Final PNG inspected; shortened multiline labels; initial figure retained.',
    'limits': 'Fixed one-cycle offline oracle diagnostics only; original weights are descriptive; no new control or online prediction.'}
for label, folder in [('source', 'gazebo_dynamic_consumption_evidence'), ('witness', 'dynamic_consumption_witness'),
                      ('raw_rollout', 'raw_rollout_objective'), ('processed', 'processed_cv_objective')]:
    provenance[label+'_archive'] = '../'+folder
    provenance[label+'_manifest_sha256'] = hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python_analysis': sys.version, 'matplotlib': matplotlib.__version__,
    'numpy': numpy.__version__, 'pyyaml': yaml.__version__, 'cpp_environment': 'existing rm2027_navigation:humble image; network none; no ROS initialization',
    'runtime_five_and_two_prior_offline_ELF_hashes_unchanged': True,
    'factor_evaluator_sha256': '934bd10451cd029910a03599207c0df308785051dd25c073c4a3c4609623ec89'}, indent=2)+'\n')
files = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope': 'Conditional geometry/local-CV/future-circle factors; no deployment acceptance.', 'files': files}, indent=2)+'\n')
print(json.dumps({'files': len(files), 'bytes': sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),
                  'manifest_sha256': hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(), 'python_tool_modules': sorted(included)}))
