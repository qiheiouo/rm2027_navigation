from pathlib import Path
import hashlib,json,shutil,sys
repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_native_individual_stages');parent=repo/'docs/dynamic_obstacle_critic/stage2_evidence';out=parent/'native_individual_stages_replay'
sys.path.insert(0,str(repo/'src/rm_dynamic_obstacle_critic/tools'))
from prepare_processed_cv import verify
r=json.loads((base/'replay/replay_verification.json').read_text());assert r['verdict']=='PASS' and len(r['byte_exact'])==10 and all(r['byte_exact'].values())
refs={};count=0
for name in ['native_individual_stages','gazebo_dynamic_consumption_evidence','dynamic_consumption_witness','native_raw_map_contract','native_factor_safe_mass']:
 p=parent/name;verify(p.resolve());n=len(json.loads((p/'manifest.json').read_text())['files']);count+=n;refs[name]={'files':n,'manifest_sha256':hashlib.sha256((p/'manifest.json').read_bytes()).hexdigest()}
assert (base/'tests/verification.json').read_bytes()==(base/'independent_tests/verification.json').read_bytes()
out.mkdir()
for p in sorted((base/'replay').glob('*.replay.log')):shutil.copyfile(p,out/p.name)
for src,name in [('replay/replay_verification.json','replay_verification.json'),('independent_tests/verification.json','native63_verification.json'),('independent_prepare.log','native_input_prepare.log'),('independent_sdk_and_tests.log','native_execution_and_tests.log'),('replay.log','driver.log')]:shutil.copyfile(base/src,out/name)
shutil.copyfile(Path(__file__),out/'freeze_replay.py')
provenance={'date':'2026-10-03 Asia/Shanghai','references':refs,'reference_file_count':count,'byte_exact_checks':10,'native63_checks_reexecuted':True,'original_archive_modified':False,'default_witness_ELF_unchanged':True,'old311_binary_JSON_and_raw_stage_bytes_preserved':True,'exact_cell_scope':'raw185 three stage paths only; all300 map labels conservative','physical_support_bounds':'unknown','actual_CostCritic_state':'unknown','original_task_raw203_age':'FAILED','composite':'NOT ESTABLISHED','runtime_profiles_TF_or_public_contracts_changed':False}
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.iterdir()) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Independent actual SDK execution and source-only individual-stage replay, with unchanged archived input/output identities.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes_excluding_manifest':sum((out/f).stat().st_size for f in files),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'reference_files':count}),flush=True)
