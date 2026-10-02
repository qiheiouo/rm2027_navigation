import gzip,hashlib,json,shutil,subprocess,yaml
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');temp=Path('/tmp/rm_dynamic_critic_v1')
trial=temp/'gazebo_dynamic_consumption_evidence_retry';out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_dynamic_consumption_evidence'
out.mkdir();checkpoint='9f73d72';package=repo/'src/rm_dynamic_obstacle_critic'
source=json.loads((trial/'source_identity.json').read_text())
for relative,digest in source.items():
 data=subprocess.check_output(['git','show',checkpoint+':src/rm_dynamic_obstacle_critic/'+relative],cwd=repo)
 assert hashlib.sha256(data).hexdigest()==digest,relative
 target=out/'runtime_source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
previous=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_controller_odom_contract'
current_binary=json.loads((trial/'binary_identity.json').read_text());old_binary=json.loads((previous/'binary_identity.json').read_text())
changed=[p for p,v in current_binary.items() if old_binary.get(p)!=v]
assert len(changed)==1 and changed[0].endswith('librm_dynamic_obstacle_critic.so')
for kind in ['installed','scene']:
 data=json.loads((trial/(kind+'_input_identity.json')).read_text());assert data==json.loads((previous/(kind+'_input_identity.json')).read_text())
 for name,digest in data.get('sha256',data).items():assert hashlib.sha256((trial/(kind+'_inputs')/name).read_bytes()).hexdigest()==digest
actual=yaml.safe_load((trial/'profile.yaml').read_text());old=yaml.safe_load((previous/'profile.yaml').read_text())
dynamic=actual['controller_server']['ros__parameters']['FollowPath']['DynamicObstacleCritic']
assert dynamic.pop('consumption_evidence_directory')=='/out/gazebo_dynamic_consumption_evidence_retry/dynamic_scores'
assert dynamic.pop('consumption_evidence_max_records')==400
for config in [actual,old]:config['controller_server']['ros__parameters']['FollowPath']['NativeCycleSnapshotCritic'].pop('output_directory')
assert actual==old

def copy(src,dest,compress=False):
 dest.parent.mkdir(parents=True,exist_ok=True)
 if compress:
  with src.open('rb') as source,dest.open('xb') as sink:
   with gzip.GzipFile(filename='',mode='wb',fileobj=sink,mtime=0) as gz:shutil.copyfileobj(source,gz)
 else:shutil.copyfile(src,dest)
for path in trial.iterdir():
 if path.is_file():
  compressed=path.suffix=='.jsonl' or path.name=='launch.log'
  copy(path,out/(path.name+'.gz' if compressed else path.name),compressed)
for name in ['scene_inputs','installed_inputs']:shutil.copytree(trial/name,out/name)
for name in ['native_cycles','dynamic_scores']:
 for path in (trial/name).iterdir():
  if path.is_file():copy(path,out/name/(path.name+'.gz' if path.suffix=='.bin' else path.name),path.suffix=='.bin')
copy(temp/'gazebo_dynamic_consumption_evidence_retry_driver.log',out/'driver.log')
shutil.copytree(temp/'gazebo_dynamic_consumption_evidence',out/'initial_prelaunch_failure')
copy(temp/'gazebo_dynamic_consumption_evidence_driver.log',out/'initial_prelaunch_failure/driver.log')
shutil.copytree(temp/'dynamic_consumption_initial_figures',out/'verification/initial_figures')
for path in (package/'tools').iterdir():
 if path.suffix in ['.py','.cpp']:copy(path,out/'auditors'/path.name)
optimizer=temp/'dynamic_consumption_optimizer'
for name in ['schedule.json','positive_comparison.json','zero_history_comparison.json']:copy(optimizer/name,out/'native_optimizer'/name)
for name in ['sdk_final_output.jsonl','zero_history_output.jsonl','weights_output.jsonl','uniform_output.jsonl']:copy(optimizer/name,out/'native_optimizer'/(name+'.gz'),True)
copy(temp/'dynamic_consumption_actual_input/input_identity.json',out/'dynamic_replay/input_identity.json')
copy(temp/'dynamic_consumption_actual_risk.jsonl',out/'dynamic_replay/native_risk.jsonl.gz',True)
for path in temp.glob('dynamic_consumption*.log'):copy(path,out/'verification'/path.name)
for name in ['rm_dynamic_obstacle_critic','replay_dynamic_consumption','replay_native_optimizer']:
 copy(temp/f'build/rm_dynamic_obstacle_critic/CMakeFiles/{name}.dir/flags.make',out/'verification'/(name+'_flags.make'))
metadata={'runtime_source_checkpoint':checkpoint,'runtime_source_files_verified':len(source),'runtime_delta':'own default-off dynamic consumption evidence; four native/guard/observer ELFs unchanged','installed_scene_inputs_exact_previous':True,'profile_semantic_delta':'two evidence knobs and evidence destinations only','physical_task_raw_verdict':'FAILED','exact_dynamic_risk_candidates':348*300,'exact_dynamic_native_batch_joins':348,'effective_snapshot_velocity_verdict':'FAILED, source164 exceeds fixed150ms window','score_epoch_values':'348/348 exact canonical matches in unchanged window; actual callback stamp unavailable','optimizer_input_sha256':hashlib.sha256((optimizer/'input.bin').read_bytes()).hexdigest(),'dynamic_input_sha256':hashlib.sha256((temp/'dynamic_consumption_actual_input/input.bin').read_bytes()).hexdigest()}
(out/'archive_identity.json').write_text(json.dumps(metadata,indent=2)+'\n')
(out/'replay_policy.json').write_text(json.dumps({'plot_title':'Dynamic consumption evidence: task / raw203 FAILED','association_window_seconds':.05,'no_noninitial_reset':True},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_dynamic_consumption_trial.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'FAILED task/raw203 and one final-epoch speed provenance; exact native/consumption evidence only','source_checkpoint':checkpoint,'files':files},indent=2)+'\n')
print('archived',len(files),'files',sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),flush=True)
