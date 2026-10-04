import hashlib,json,subprocess,tarfile,shutil
from pathlib import Path
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004')
original=Path('/home/qihei/rm2027_navigation');root=work/'build/temporal_mpc_timing_20261004'
names=['head_on_b0_01','head_on_portfolio_01']
def git(cwd,*args):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def source_files(name):
 with tarfile.open(root/name/'source_snapshot.tar.gz') as tar:
  return {m.name:hashlib.sha256(tar.extractfile(m).read()).hexdigest() for m in tar.getmembers() if m.isfile()}
files={n:source_files(n) for n in names};identities={n:json.loads((root/n/'runtime_identity.json').read_text()) for n in names}
reference=files[names[0]]
core_names=['temporal_mpc/realtime_qp.py','temporal_mpc/candidates.py','temporal_mpc/contracts.py','temporal_mpc/geometry.py','temporal_mpc/dynamics.py','temporal_mpc/local_reference.py','temporal_mpc/frontend.py','temporal_mpc/execution_guard.py','temporal_mpc/selection.py','ros2/rm_temporal_mpc_controller/include/stopping.hpp','integration/execution_guard_node.py','integration/selector_node.py','gazebo/run.sh','gazebo/prepare_scene.py']
core={}
for p in core_names:
 relative='experiments/temporal_mpc/'+p
 assert (work/relative).is_file()
 before=subprocess.check_output(['git','show','ca89b527:'+relative],cwd=work)
 core[relative]={'before_sha256':hashlib.sha256(before).hexdigest(),'current_sha256':digest(work/relative)}
def validate_body(text):return text[text.index('  bool validate('):text.index('  void request(')]
cpp='experiments/temporal_mpc/ros2/rm_temporal_mpc_controller/src/controller.cpp'
before=subprocess.check_output(['git','show','ca89b527:'+cpp],cwd=work,text=True)
native_validate_unchanged=validate_body(before)==validate_body((work/cpp).read_text())
differences={p:{'physical':sha,'current':digest(work/p) if (work/p).is_file() else None} for p,sha in reference.items() if not (work/p).is_file() or digest(work/p)!=sha}
nav2=json.loads((root/'nav2_portfolio_01/runtime_identity.json').read_text())
result={'source_hashes':files,'physical_sources_identical':files[names[0]]==files[names[1]],
 'current_sources_match_physical_snapshot':not differences,'postrun_source_differences':differences,
 'current_runtime_sources_match_physical_snapshot':set(differences)<= {'experiments/temporal_mpc/README.md','experiments/temporal_mpc/gazebo/README.md'},
 'same_prestart_binaries_and_dependencies':all((v['binaries'],v['dependencies'])==(identities[names[0]]['binaries'],identities[names[0]]['dependencies']) for v in identities.values()),
 'all_identities_prestart':all(v['captured_before_process_start'] for v in identities.values()),
 'nav2_engineering_binaries_and_dependencies_match_physical':(nav2['binaries'],nav2['dependencies'])==(identities[names[0]]['binaries'],identities[names[0]]['dependencies']),
 'control_core_files':core,'control_core_unchanged_since_ca89b527':all(v['before_sha256']==v['current_sha256'] for v in core.values()),
 'native_validate_body_unchanged_since_ca89b527':native_validate_unchanged,
 'current_branch':git(work,'branch','--show-current'),'main_base':git(work,'merge-base','HEAD','main'),
 'formal_src_diff':git(work,'diff','main','--name-only','--','src'),'post_physics_runtime_changes':False,
 'scope':'Timing instrumentation changes worker/native request and health reporting; valid request gates and control core remain equivalent. Invalid stamp diagnostics now retain nullable identity instead of restamping after catch. Same physical run sources/binaries/dependencies, no tuning or physical reruns. Post-run documentation may differ from snapshots.'}
assert all(result[k] for k in ['physical_sources_identical','current_runtime_sources_match_physical_snapshot','same_prestart_binaries_and_dependencies','all_identities_prestart','nav2_engineering_binaries_and_dependencies_match_physical','control_core_unchanged_since_ca89b527','native_validate_body_unchanged_since_ca89b527'])
assert not result['formal_src_diff']
(root/'source_identity.json').write_text(json.dumps(result,indent=2)+'\n')
(root/'binaries').mkdir(exist_ok=True)
for relative,item in identities[names[0]]['binaries'].items():
 p=work/relative;assert digest(p)==item['sha256'];shutil.copy2(p,root/'binaries'/p.name)
base=json.loads(Path('/tmp/temporal_mpc_preserved_20261004.json').read_text());preserved={}
for relative,item in base['files'].items():
 p=original/relative;observed={'size':p.stat().st_size,'sha256':digest(p) if item['sha256'] is not None else None}
 preserved[relative]={'expected':item,'observed':observed,'pass':item==observed}
refs={ref:{'expected':value,'observed':git(original,'rev-parse',ref)} for ref,value in base['refs'].items()}
for item in refs.values():item['pass']=item['expected']==item['observed']
preservation={'files':preserved,'refs':refs,'all_preserved':all(v['pass'] for v in preserved.values()) and all(v['pass'] for v in refs.values()),'large_original_core':'size checked; no huge core read or changed'}
assert preservation['all_preserved'];(root/'preservation_check.json').write_text(json.dumps(preservation,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ['source_hashes','control_core_files']},indent=2))
print('original state preserved',preservation['all_preserved'])
