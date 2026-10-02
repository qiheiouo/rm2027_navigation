import ast
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy
import yaml

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
tools=repo/'src/rm_dynamic_obstacle_critic/tools'
sys.path.insert(0,str(tools))
from replay_conditional_cv_support import geometry_summary
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/conditional_cv_support'
out.mkdir()
def copy(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    shutil.copyfile(source,target)
pending=['conditional_cv_support','evaluate_conditional_cv_support','prepare_conditional_cv_queries',
         'test_conditional_cv_support','test_conditional_cv_cli','replay_conditional_cv_support']
included=set()
while pending:
    name=pending.pop()
    if name in included:continue
    included.add(name);path=tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        names=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
        for module in names:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
    copy(path,out/'source/tools'/path.name)
for name in ['cv_support_assumptions_offline.yaml','ground_robot_extent_prior_offline.yaml']:
    copy(repo/'src/rm_dynamic_obstacle_critic/config'/name,out/'source/config'/name)
for path in (base/'conditional_cv_support_input').iterdir():copy(path,out/'prepared'/path.name)
for path in (base/'conditional_cv_support_cli_tests').iterdir():copy(path,out/'verification/cli_tests'/path.name)
for name in ['conditional_cv_support_tests.log','conditional_cv_support_tests_final.log',
             'conditional_cv_support_prepare.log','conditional_cv_support_prepare_final.log',
             'conditional_cv_support_prepare_failed_source.py','conditional_cv_support_cli_tests.log','conditional_cv_support_binaries.txt']:
    copy(base/name,out/'verification'/name)
copy(base/'cv_support_factor_binaries_after.txt',out/'verification/prior_seven_binaries.txt')
expected=(base/'cv_support_factor_binaries_after.txt').read_text()+ '934bd10451cd029910a03599207c0df308785051dd25c073c4a3c4609623ec89  /out/build/rm_dynamic_obstacle_critic/evaluate_cv_support_factors\n'
if (base/'conditional_cv_support_binaries.txt').read_text()!=expected:raise ValueError('prior eight ELF hashes changed')
(out/'geometry_verification.json').write_text(json.dumps(geometry_summary(base/'conditional_cv_support_tests_final.log'),indent=2)+'\n')
source=out.parent/'gazebo_dynamic_consumption_evidence'
(out/'provenance.json').write_text(json.dumps({'preregistration_checkpoint':'2a97d92','source_archive':'../gazebo_dynamic_consumption_evidence',
    'source_manifest_sha256':hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
    'actual_query_source_ordinal':298,'default_all_support_conditions':'unknown','runtime_five_and_three_offline_ELFs_unchanged':True,
    'runtime_or_TF_or_public_message_changed':False,'original_task_raw203_and_age_verdict':'FAILED','full_witness_assessment':'NOT ESTABLISHED',
    'limits':'Pure conditional set support theorem; synthetic assumptions are not physical evidence; no online profile.'},indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,
    'random_seed':20261002,'no_ROS_initialization':True},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_conditional_cv_support.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Conditional support mathematics/unknown contract only.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),
                  'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'python_tool_modules':sorted(included)}))
