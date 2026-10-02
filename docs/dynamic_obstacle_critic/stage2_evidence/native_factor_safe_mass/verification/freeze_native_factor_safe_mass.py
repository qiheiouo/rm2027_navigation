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
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_factor_safe_mass';out.mkdir()
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(source.read_bytes())
    else:shutil.copyfile(source,target)
tools=repo/'src/rm_dynamic_obstacle_critic/tools';pending=['audit_native_factor_safe_mass','verify_native_factor_safe_mass',
    'test_native_factor_safe_mass','plot_native_factor_safe_mass','replay_native_factor_safe_mass'];included=set()
while pending:
    name=pending.pop()
    if name in included:continue
    included.add(name);path=tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        modules=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
        for module in modules:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
    copy(path,out/'source/tools'/path.name)
for source,target in [('native_factor_safe_mass_report.json','report.json.gz'),('native_factor_safe_mass_scalar.json','scalar_verification.json.gz')]:copy(base/source,out/target,True)
copy(base/'native_factor_safe_mass_tests_final/positive.json',out/'verification.json')
if (base/'native_factor_safe_mass_tests_final/positive.json').read_bytes()!=(base/'native_factor_safe_mass_verification.json').read_bytes():raise ValueError('rational checker amendments changed result')
result=json.loads((base/'native_factor_safe_mass_tests_final/verification.json').read_text())
if result['verdict']!='PASS' or len(result['checks'])!=13:raise ValueError('all thirteen integrity/CLI probes required')
for path in (base/'native_factor_safe_mass_tests_final').iterdir():
    compress=path.suffix in ['.json','.jsonl'] and path.name not in ['positive.json','verification.json']
    copy(path,out/'verification/tests'/(path.name+('.gz' if compress else '')),compress)
for path in (base/'native_factor_safe_mass_figures').iterdir():copy(path,out/'figures'/path.name)
for name in ['native_factor_safe_mass_execute.log','native_factor_safe_mass_verification.log','native_factor_safe_mass_tests.log',
             'native_factor_safe_mass_tests_final.log','native_factor_safe_mass_plot.log','native_factor_safe_mass_binaries.txt','native_factor_safe_mass_binaries.log']:
    copy(base/name,out/'verification'/name)
copy(base/'native_factor_safe_mass_tests/verification.json',out/'verification/initial_eleven_probes_verification.json')
if (base/'native_factor_safe_mass_binaries.txt').read_bytes()!=(base/'native_sg_stage_binaries_final.txt').read_bytes():raise ValueError('prior fourteen ELFs changed')
report=json.loads((base/'native_factor_safe_mass_report.json').read_text());scalar=json.loads((base/'native_factor_safe_mass_scalar.json').read_text())
if not report['all311_source_pose_full_geometry_and_gates_exact'] or scalar['verdict']!='PASS' or len(scalar['verified_sampler_raw_rows'])!=43:raise ValueError('full311/source39/43 scalar proofs missing')
provenance={'preregistration_checkpoint':'ef3c93e','native_source_ordinal':298,'sampler_raw_rows':300,'individual_SG_offset':11,
    'runtime_or_TF_or_public_interfaces_changed':False,'prior_fourteen_ELFs_unchanged':True,'full_primary_safe_individuals':38,
    'full_source_pose_safe_individuals':39,'independent_raw_scalar_subset_rows':43,'full_aggregate_safe_progress_factors':0,
    'original_task_raw203_and_age':'FAILED','full_witness_assessment':'NOT ESTABLISHED','physical_support_bounds':'unknown',
    'visual_review':'PNG inspected: paired weight bars and row63 positive sampled gap versus negative primary interval lower are clear.',
    'limits':'Descriptive native raw-cost weights versus distinct individual SG paths; no selection, convexity or execution acceptance.'}
for label,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('processed','processed_cv_objective'),
                     ('factors','cv_support_factors'),('aggregation','native_factor_aggregation'),('stages','native_sg_stages'),('raw','raw_rollout_objective')]:
    provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,'matplotlib':matplotlib.__version__,
    'CPP_execution':'No new compilation or native execution needed; original fourteen frozen ELFs read-only verified.'},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_native_factor_safe_mass.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Safe-individual mass and raw/SG type integrity; all aggregate failures retained.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'source_modules':len(included)}))
