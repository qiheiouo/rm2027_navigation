import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import yaml

repo=Path('/home/qihei/rm2027_navigation')
temp=Path('/tmp/rm_dynamic_critic_v1')
trial=temp/'gazebo_controller_odom_contract'
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_controller_odom_contract'
out.mkdir()
checkpoint='c775723'
package=repo/'src/rm_dynamic_obstacle_critic'
source=json.loads((trial/'source_identity.json').read_text())
for relative,digest in source.items():
 data=subprocess.check_output(['git','show',checkpoint+':src/rm_dynamic_obstacle_critic/'+relative],cwd=repo)
 assert hashlib.sha256(data).hexdigest()==digest,relative
 target=out/'runtime_source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
previous=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_native_cycle_evidence'
assert json.loads((trial/'binary_identity.json').read_text())==json.loads((previous/'binary_identity.json').read_text())
for kind in ['installed','scene']:
 data=json.loads((trial/(kind+'_input_identity.json')).read_text());hashes=data.get('sha256',data)
 assert data==json.loads((previous/(kind+'_input_identity.json')).read_text())
 for name,digest in hashes.items():assert hashlib.sha256((trial/(kind+'_inputs')/name).read_bytes()).hexdigest()==digest
actual=yaml.safe_load((trial/'profile.yaml').read_text());old=yaml.safe_load((previous/'profile.yaml').read_text())
assert actual['controller_server']['ros__parameters'].pop('odom_topic')=='/odometry/lio'
for config in [actual,old]:config['controller_server']['ros__parameters']['FollowPath']['NativeCycleSnapshotCritic'].pop('output_directory')
assert actual==old

def copy(src,dest,compress=False):
 dest.parent.mkdir(parents=True,exist_ok=True)
 if compress:
  with src.open('rb') as stream,dest.open('xb') as sink:
   with gzip.GzipFile(filename='',mode='wb',fileobj=sink,mtime=0) as gz:shutil.copyfileobj(stream,gz)
 else:shutil.copyfile(src,dest)

for path in trial.iterdir():
 if path.is_file():copy(path,out/(path.name+'.gz' if path.suffix=='.jsonl' or path.name=='launch.log' else path.name),path.suffix=='.jsonl' or path.name=='launch.log')
for name in ['scene_inputs','installed_inputs']:shutil.copytree(trial/name,out/name)
for path in (trial/'native_cycles').iterdir():
 if path.is_file():copy(path,out/'native_cycles'/(path.name+'.gz' if path.suffix=='.bin' else path.name),path.suffix=='.bin')
copy(temp/'gazebo_controller_odom_contract_driver.log',out/'driver.log')
for path in (package/'tools').iterdir():
 if path.suffix in ['.py','.cpp']:copy(path,out/'auditors'/path.name)
optimizer=temp/'controller_odom_optimizer_window50ms'
for name in ['schedule.json','positive_comparison.json','zero_history_comparison.json','omitted_reset_comparison.json']:
 copy(optimizer/name,out/'native_optimizer'/name)
for name in ['sdk_final_output.jsonl','zero_history_output.jsonl','omitted_reset_output.jsonl']:
 copy(optimizer/name,out/'native_optimizer'/(name+'.gz'),True)
for name in ['controller_odom_optimizer_prepare.log','controller_odom_optimizer_window50ms_prepare.log','controller_odom_optimizer_sdk_run.log','controller_odom_optimizer_negatives.log','controller_odom_legacy_effective_audit.json','controller_odom_legacy_optimizer_regression.log','controller_odom_comparison_tests.log','controller_odom_comparison_tests.xml','controller_odom_binary_identity.txt','controller_odom_pinned_header.txt','controller_odom_callback_disassembly_bounded.txt']:
 copy(temp/name,out/'verification'/name)
copy(temp/'controller_odom_optimizer/input.bin',out/'verification/failed_10ms_partial_input.bin.gz',True)
copy(temp/'controller_odom_legacy_optimizer_regression/schedule.json',out/'verification/legacy_schedule.json')
# Freeze a bounded original ELF excerpt, not unrelated enormous template symbols.
lines=(temp/'controller_odom_controller_disassembly.txt').read_text().splitlines()
start=next(i for i,line in enumerate(lines) if line.startswith('00000000000953f0 <'))
end=next(i for i,line in enumerate(lines[start+1:],start+1) if line.startswith('0000000000'))
(out/'verification/controller_compute_velocity_disassembly.txt').write_text('\n'.join(lines[start:end])+'\n')
metadata={'runtime_source_checkpoint':checkpoint,'runtime_source_files_verified':len(source),'runtime_five_ELFs_exact_previous':True,'installed_and_scene_inputs_exact_previous':True,'profile_semantic_delta':'controller_server.odom_topic=/odometry/lio; native output directory is an evidence destination','physical_verdict':'FAILED','effective_velocity_value_consistency':'PASS; actual consumer source stamp unavailable','observer_association':'explicit 50ms hypothesis after original 10ms failed at ordinal89; all350 tensors and commands bit exact','optimizer_input_sha256':hashlib.sha256((optimizer/'input.bin').read_bytes()).hexdigest(),'negative_omitted_reset_input_sha256':hashlib.sha256((optimizer/'omitted_reset_input.bin').read_bytes()).hexdigest()}
(out/'archive_identity.json').write_text(json.dumps(metadata,indent=2)+'\n')
(out/'replay_policy.json').write_text(json.dumps({'plot_title':'Controller odometry repair: physical safety / task FAILED','association_window_seconds':.05},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_controller_odom_trial.py')
files={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'FAILED physical trial with repaired route and recovered native speed values; not deployment or safe-control acceptance','source_checkpoint':checkpoint,'mechanical_include_static':True,'files':files},indent=2)+'\n')
print('archived',len(files),'files',sum(path.stat().st_size for path in out.rglob('*') if path.is_file()))
