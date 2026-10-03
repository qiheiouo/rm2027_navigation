from pathlib import Path
import difflib,gzip,hashlib,json,shutil,sys
repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_native_raw_recovery');archives=repo/'docs/dynamic_obstacle_critic/stage2_evidence'
original=archives/'native_raw_map_contract';out=archives/'native_raw_map_contract_replay'
sys.path.insert(0,str(repo/'src/rm_dynamic_obstacle_critic/tools'))
from prepare_processed_cv import verify
refs={};file_count=0
for name in ['native_raw_map_contract','native_cost_map_contract','gazebo_dynamic_consumption_evidence','dynamic_consumption_witness','raw_rollout_objective','native_factor_aggregation','native_sg_stages']:
    root=archives/name;verify(root);m=root/'manifest.json';count=len(json.loads(m.read_text())['files']);file_count+=count
    refs[name]={'manifest_sha256':hashlib.sha256(m.read_bytes()).hexdigest(),'files':count}
r=json.loads((base/'replay/replay_verification.json').read_text())
assert r['verdict']=='PASS' and len(r['byte_exact'])==11 and all(r['byte_exact'].values())
assert r['mutation_recipes']=={'verdict':'PASS','hash_checks':38}
checks={}
for filename,key in [('new_native_tests/verification.json','raw72'),('default_native_tests/verification.json','default79')]:
    v=json.loads((base/filename).read_text());assert v['verdict']=='PASS' and len(v['checks'])==(72 if key=='raw72' else 79)
    expected=original/'verification/native_tests/verification.json' if key=='raw72' else archives/'native_cost_map_contract/verification/native_tests/verification.json'
    checks[key+'_verification_byte_exact']=(base/filename).read_bytes()==expected.read_bytes()
checks['default21165_native_output_byte_exact']=(base/'default_native_queries.jsonl').read_bytes()==gzip.open(archives/'native_cost_map_contract/native_queries.jsonl.gz','rb').read()
checks['default_synthetic_output_byte_exact']=(base/'default_native_tests/synthetic.jsonl').read_bytes()==(archives/'native_cost_map_contract/native_synthetic.jsonl').read_bytes()
assert all(checks.values())
ids=json.loads((base/'rebuilt_identities.json').read_text())
expected={'native_cost_map_queries':'bb45f51c0c9e7c956d3e967608743032cf12237a22fb5bbecc489357ca28ec39','native_raw_map_queries':'43575158ddb202f4824e4fe0108c5bc720107a0ae93435b3297044b5a98bae17','libnav2_costmap_2d_core.so':'d90906d3f7c461b540562707c17d01a35df02f19b11c8bbbda3ca351a7e4dab1','libmppi_controller.so':'fc259b61103fe1cc8ce6e5fa517460074aecf4b8d5c1075c50fad0ab0eaff88b','libmppi_critics.so':'cb4f73ca0ac36be85c37a7bb1eafcca94120d8300d7c290c52a35f89fe4f31ef'}
assert ids==expected
out.mkdir()
def copy(src,rel):
    dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
for name in ['replay_verification.json','recomputed_recipes.json']:
    copy(base/'replay'/name,name)
for p in sorted((base/'replay').glob('*.replay.log')):copy(p,'logs/'+p.name)
for name in ['original_recipe_failure.log','corrected_recipes.log','build.log','prepare.log','native_execution_and_tests.log','default_execution_and_tests.log','default_tests_corrected.log','replay.log','build_from_archived_map_commands.py','rebuilt_identities.json']:
    copy(base/name,'recovery/'+name)
copy(base/'new_native_tests/verification.json','native72_verification.json');copy(base/'default_native_tests/verification.json','default79_verification.json')
for name in ['replay_native_raw_map_contract.py','verify_raw_map_mutation_recipes.py']:
    current=repo/'src/rm_dynamic_obstacle_critic/tools'/name;copy(current,'amended_tools/'+name)
    old=(original/'source/tools'/name).read_text().splitlines(keepends=True);new=current.read_text().splitlines(keepends=True)
    (out/'amended_tools'/(name+'.diff')).write_text(''.join(difflib.unified_diff(old,new,fromfile='frozen/'+name,tofile='amended/'+name)))
copy(Path(__file__),'freeze_replay.py')
provenance={'date':'2026-10-03 Asia/Shanghai','references':refs,'reference_file_count':file_count,'byte_exact_replay_checks':11,'mutation_recipe_hash_checks':38,'recovery_regression_checks':checks,
    'rebuilt_helpers_and_reread_SDKs_match':True,'missing_old_temporary_ELFs':'Historical identities preserved in original archive; not claimed reread after temporary directory vanished.',
    'original_archive_mutated':False,'amendment':'Explicit mutation-verifier source adds coordinated_pose/mutation.json discovery. Current replay driver stores both verifier identities and separates recipe hashes from byte-exact checks.',
    'preserved_failures':['Original frozen verifier found only 37 recipes and rejected missing coordinated entry.','First recovery default test invocation omitted mandatory format arguments and exited 2; corrected invocation exited 0.'],
    'full_old_ephemeral_negative_files':'Lost with old temporary directory; reconstructed from complete frozen bases and 38 lossless recipes, then new compiled probes executed. Frozen provenance is retained as a statement at freeze time.',
    'runtime_TF_profiles_and_public_contracts_changed':False,'actual_CostCritic_state_and_cell_source':'unknown','original_task_raw203_age':'FAILED','composite':'NOT ESTABLISHED','physical_support_bounds':'unknown'}
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Independent raw map replay with actual rebuilt SDK tools; immutable original evidence, explicit source amendment and lost-temp recovery.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes_excluding_manifest':sum((out/f).stat().st_size for f in files),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'references':len(refs),'reference_files':file_count}),flush=True)
