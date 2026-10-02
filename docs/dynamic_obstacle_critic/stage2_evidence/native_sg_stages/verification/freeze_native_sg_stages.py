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
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_sg_stages';out.mkdir()
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(source.read_bytes())
    else:shutil.copyfile(source,target)
tools=repo/'src/rm_dynamic_obstacle_critic/tools'
pending=['prepare_native_factor_aggregation','prepare_native_stage_scoring','analyze_native_sg_stages',
         'plot_native_sg_stages','test_native_sg_stage_io','test_native_sg_stage_evidence','replay_native_sg_stages',
         'test_native_factor_aggregation','test_cv_aggregate_factors']
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
for path in (base/'native_sg_stage_scoring_input').iterdir():copy(path,out/'scoring'/path.name)
for source,target in [('native_sg_stage_paths.bin','stage_paths.bin'),('native_sg_stage_native.jsonl','native_output.jsonl.gz'),
                      ('native_sg_stage_scores.jsonl','scores.jsonl.gz')]:copy(base/source,out/target,target.endswith('.gz'))
for initial,test in [('native_sg_stage_paths.bin','native_sg_stage_evaluator_tests/positive_paths.bin'),
                     ('native_sg_stage_native.jsonl','native_sg_stage_evaluator_tests/positive.jsonl'),
                     ('native_sg_stage_scores.jsonl','native_sg_stage_score_tests/scores.jsonl')]:
    if (base/initial).read_bytes()!=(base/test).read_bytes():raise ValueError('initial/test numerical artifact mismatch')
for source,target in [('native_sg_stage_analysis.json','analysis.json'),('native_sg_stage_scalar.json','scalar_verification.json'),
                      ('native_sg_stage_raw_scalar.json','raw_scalar_verification.json')]:copy(base/source,out/target)
for folder,target in [('native_sg_stage_figures_final','figures'),('native_sg_stage_figures','verification/initial_figures')]:
    for path in (base/folder).iterdir():copy(path,out/target/path.name)
for folder,target,skip,count in [('native_sg_stage_evaluator_tests','native_tests',{'positive_paths.bin','positive.jsonl'},31),
                         ('native_sg_stage_score_tests','score_tests',{'scores.jsonl'},19),
                         ('native_sg_stage_io_tests','io_tests',set(),12),('native_sg_stage_evidence_tests','evidence_tests',set(),5)]:
    report=json.loads((base/folder/'verification.json').read_text())
    if report['verdict']!='PASS' or len(report['checks'])!=count:raise ValueError('expected executable/probe checks missing')
    for path in (base/folder).rglob('*'):
        if not path.is_file() or path.name in skip:continue
        compress=path.suffix in ['.bin','.jsonl'];relative=path.relative_to(base/folder)
        copy(path,out/'verification'/target/(str(relative)+('.gz' if compress else '')),compress)
logs=['native_sg_stage_build.log','native_sg_stage_execute.log','native_sg_stage_score_execute.log',
      'native_sg_stage_scoring_prepare.log','native_sg_stage_analysis.log','native_sg_stage_native_tests.log',
      'native_sg_stage_score_tests.log','native_sg_stage_io_tests.log','native_sg_stage_evidence_tests.log',
      'native_sg_stage_current_four_geometry.log','native_sg_stage_current_four_raw.log',
      'native_sg_stage_default_regression_identity.json','native_sg_stage_plot.log','native_sg_stage_plot_corrected.log',
      'native_sg_stage_plot_final.log','native_sg_stage_plot_failed_source.py','native_sg_stage_binaries_initial.txt',
      'native_sg_stage_binaries_final.txt','native_sg_stage_binaries_verification.log']
for name in logs:copy(base/name,out/'verification'/name)
reg=json.loads((base/'native_sg_stage_default_regression_identity.json').read_text())
if reg['verdict']!='PASS' or len(reg['byte_exact'])!=86:raise ValueError('default regression bytes not established')
for folder,target in [('native_sg_stage_default_native_regression','default_native_verification.json'),
                      ('native_sg_stage_default_score_regression','default_score_verification.json')]:copy(base/folder/'verification.json',out/'verification'/target)
if not (base/'native_sg_stage_binaries_final.txt').read_text().startswith((base/'native_factor_aggregation_binaries_final.txt').read_text()):raise ValueError('prior twelve ELF identity changed')
for name in ['evaluate_native_sg_stages','evaluate_cv_stage_factors']:
    copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/'flags.make',out/'verification'/(name+'_flags.make'))
provenance={'preregistration_checkpoint':'3d66a19','native_source_ordinal':298,'stage_paths':8,
    'four_after_full_labels_exact':True,'runtime_or_TF_or_public_interfaces_changed':False,'prior_twelve_ELFs_unchanged':True,
    'full_conditional_safe_progress_before_and_after':0,'original_task_raw203_and_age':'FAILED','full_witness_assessment':'NOT ESTABLISHED',
    'physical_support_bounds':'unknown','visual_review':'Final PNG inspected, paired negative labels staggered; first overlapping figure and byte conversion failure retained.',
    'limits':'Fixed-context model hypotheses only. Future-circle raw216 sampled contact at +2.982s exists both before and after SG; no global SG causal or deployment conclusion.'}
for label,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('factors','cv_support_factors'),('aggregation','native_factor_aggregation')]:
    provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,'matplotlib':matplotlib.__version__,
    'CPP_environment':'existing rm2027_navigation:humble; network none; no ROS init','stage_native_ELF_sha256':'7f8380bb6ecfb35292a08050af93b4840f9219196043ac5911431005104eb6f9',
    'pure_eight_path_ELF_sha256':'3b60baa88eb8627c14e7506c999e69f192ba616479c877a64c4a8aeffc43a6f3'},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_native_sg_stages.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Fixed-context stage paths; original failures and bounds unknown retained.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'source_modules':len(included)}))
