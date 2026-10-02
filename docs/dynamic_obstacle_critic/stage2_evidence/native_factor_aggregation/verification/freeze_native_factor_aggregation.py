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

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_factor_aggregation';out.mkdir()
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(source.read_bytes())
    else:shutil.copyfile(source,target)
tools=repo/'src/rm_dynamic_obstacle_critic/tools'
pending=['prepare_native_factor_aggregation','prepare_native_factor_scoring','analyze_native_factor_aggregation',
         'verify_native_factor_raw','plot_native_factor_aggregation','test_native_factor_aggregation',
         'test_cv_aggregate_factors','test_native_factor_io','test_native_factor_raw_evidence','replay_native_factor_aggregation',
         'test_cv_support_factors']
included=set()
while pending:
    name=pending.pop()
    if name in included:continue
    included.add(name);path=tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        modules=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
        for module in modules:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
    copy(path,out/'source/tools'/path.name)
for name in ['evaluate_native_factor_aggregation.cpp','evaluate_cv_support_factors.cpp','dynamic_replay_input.hpp']:
    copy(tools/name,out/'source/tools'/name)
for name in ['model.hpp','geometry.hpp']:
    copy(repo/'src/rm_dynamic_obstacle_critic/include/rm_dynamic_obstacle_critic'/name,out/'source/include/rm_dynamic_obstacle_critic'/name)
copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt',out/'source/CMakeLists.txt')
copy(base/'native_factor_aggregation_input/input_identity.json',out/'prepared/input_identity.json')
copy(base/'native_factor_aggregation_input/aggregation_input.bin',out/'prepared/aggregation_input.bin.gz',True)
for path in (base/'native_factor_scoring_input').iterdir():copy(path,out/'scoring'/path.name)
copy(base/'native_factor_aggregation_evaluator_tests/positive_paths.bin',out/'native_paths.bin')
copy(base/'native_factor_aggregation_evaluator_tests/positive.jsonl',out/'native_output.jsonl.gz',True)
copy(base/'native_factor_scoring_evaluator_tests/scores.jsonl',out/'scores.jsonl.gz',True)
for source,target in [('native_factor_aggregation_analysis.json','analysis.json'),('native_factor_aggregation_scalar.json','scalar_verification.json'),
                      ('native_factor_aggregation_raw_scalar.json','raw_scalar_verification.json')]:copy(base/source,out/target)
for folder,target in [('native_factor_aggregation_figures_final','figures'),('native_factor_aggregation_figures','verification/initial_figures')]:
    for path in (base/folder).iterdir():copy(path,out/target/path.name)
for folder,target,skip in [('native_factor_aggregation_evaluator_tests','native_tests',{'positive_paths.bin','positive.jsonl'}),
                         ('native_factor_scoring_evaluator_tests','score_tests',{'scores.jsonl'}),
                         ('native_factor_aggregation_io_tests','io_tests',set()),('native_factor_aggregation_raw_negative','raw_negative',set())]:
    for path in (base/folder).iterdir():
        if path.name in skip:continue
        compress=path.suffix in ['.bin','.jsonl']
        copy(path,out/'verification'/target/(path.name+('.gz' if compress else '')),compress)
logs=['native_factor_aggregation_build.log','native_factor_aggregation_build_final.log','native_factor_aggregation_prepare.log',
      'native_factor_aggregation_prepare_final.log','native_factor_aggregation_prepare_failed_source.py','native_factor_aggregation_execute.log',
      'native_factor_aggregation_analysis.log','native_factor_aggregation_raw_scalar.log','native_factor_aggregation_raw_negative.log',
      'native_factor_aggregation_evaluator_tests.log','native_factor_aggregation_io_tests.log','native_factor_scoring_evaluator_tests.log',
      'native_factor_scoring_prepare.log','native_factor_scoring_execute.log','native_factor_default_cv_regression.log',
      'native_factor_aggregation_plot.log','native_factor_aggregation_plot_final.log','native_factor_aggregation_binaries_after.txt',
      'native_factor_aggregation_binaries_final.txt']
for name in logs:copy(base/name,out/'verification'/name)
copy(base/'native_factor_aggregation_paths.bin',out/'verification/initial_CPP/native_paths.bin')
copy(base/'native_factor_aggregation_output.jsonl',out/'verification/initial_CPP/native_output.jsonl.gz',True)
initial=(base/'native_factor_aggregation_output.jsonl').read_bytes().splitlines()[1:]
final=(base/'native_factor_aggregation_evaluator_tests/positive.jsonl').read_bytes().splitlines()[1:]
if initial!=final or (base/'native_factor_aggregation_paths.bin').read_bytes()!=(out/'native_paths.bin').read_bytes():raise ValueError('library path metadata addition changed numeric results')
prior=(base/'conditional_cv_support_binaries.txt').read_text()+'5147a13366c960d8c9623ef0995c2eb80d283a5d150428601be4d53baf3a59f5  /out/build/rm_dynamic_obstacle_critic/replay_native_optimizer\n'+'d798fbd5050a943a6e9a5d4fb5d2e4cc0d2883891723a5e94b72f3c808abec01  /out/build/rm_dynamic_obstacle_critic/native_safe_control_witness\n'
if not (base/'native_factor_aggregation_binaries_final.txt').read_text().startswith(prior):raise ValueError('prior ten ELF identities changed')
copy(base/'native_factor_default_cv_regression/verification.json',out/'verification/default311_verification.json')
default=base/'native_factor_default_cv_regression';frozen=out.parent/'cv_support_factors/verification/evaluator_tests'
default_checks={}
for path in default.iterdir():
    if path.name=='factors.jsonl':reference=gzip.open(out.parent/'cv_support_factors/factors.jsonl.gz','rb').read()
    elif path.suffix in ['.bin','.jsonl']:reference=gzip.open(frozen/(path.name+'.gz'),'rb').read()
    else:reference=(frozen/path.name).read_bytes()
    default_checks[path.name]=path.read_bytes()==reference
if not all(default_checks.values()):raise ValueError('default311 probe/output bytes changed')
(out/'verification/default311_complete_artifact_identity.json').write_text(json.dumps({'verdict':'PASS','byte_exact':default_checks},indent=2)+'\n')
for name in ['evaluate_native_factor_aggregation','evaluate_cv_aggregate_factors','evaluate_cv_support_factors']:
    copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/'flags.make',out/'verification'/(name+'_flags.make'))
provenance={'preregistration_checkpoint':'a78fecf','native_source_ordinal':298,'factor_count':4,
    'runtime_and_prior_ten_ELFs_unchanged':True,'runtime_or_TF_or_public_interfaces_changed':False,
    'original_task_raw203_and_age':'FAILED','full_witness_assessment':'NOT ESTABLISHED','full_conditional_safe_progress_factors':0,
    'physical_support_bounds':'unknown','initial_to_final_CPP_numeric_lines_exact':True,
    'visual_review':'Final PNG inspected; negative labels staggered and legend clear of padded pose; initial figure retained.',
    'limits':'Fixed-context independent oracle cost interventions only; future oracle passes dynamic geometry but contacts frozen raw216 cell; no execution or deployment acceptance.'}
for label,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('factors','cv_support_factors')]:
    provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,'matplotlib':matplotlib.__version__,
    'CPP_environment':'existing rm2027_navigation:humble; network none; no ROS init','native_ELF_sha256':'0b8cb8ff2fe57bc91641f2bbd34d11cc0c274722f6b709a4696a4d877e706529',
    'pure_four_path_ELF_sha256':'c254dd8e0a7d68488bbce751b8b97d1647057afd4bd1719dfda4f3841829c79f'},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_native_factor_aggregation.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Fixed-context native aggregation factors, retained raw203 failure; no acceptance.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'source_python_modules':sorted(included)}))
