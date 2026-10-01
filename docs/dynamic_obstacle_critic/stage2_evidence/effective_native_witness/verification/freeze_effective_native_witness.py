import gzip,hashlib,json,shutil
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/effective_native_witness';out.mkdir()
def copy(src,dst,compress=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 if compress:
  with dst.open('xb') as stream:
   with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as gz:gz.write(src.read_bytes())
 else:shutil.copyfile(src,dst)
for label,folder in [('full','controller_odom_witness'),('selected','controller_odom_witness_selected')]:
 for name in ['witnesses.bin','witnesses.jsonl','analysis.json']:copy(base/folder/name,out/label/(name+'.gz'),True)
 copy(base/(folder+'_input')/'input_identity.json',out/label/'input_identity.json')
for name in ['model_integrity.json','selected_cycle.json','source_mapping_verification.json','contract_assessment.json']:
 copy(base/'controller_odom_witness'/name,out/'full'/name)
for name in ['controller_odom_witness_prepare.log','controller_odom_witness_sdk_run.log','controller_odom_witness_analysis.log','controller_odom_witness_selected_prepare.log','controller_odom_witness_selected_sdk.log','controller_odom_witness_selected_analysis.log','controller_odom_cpp_reexecution.log','controller_odom_selected_cpp_reexecution.log','controller_odom_selector_legacy_regression.json','controller_odom_pose_epoch_audit.json','controller_odom_archive_replay.log','controller_odom_legacy_complete_regression.log']:
 copy(base/name,out/'verification'/name)
for name in ['reexecution_verification.json','binary_identity.txt']:copy(base/'controller_odom_cpp_reexecution'/name,out/'verification'/name)
for label,folder in [('new','controller_odom_archive_replay'),('legacy','controller_odom_legacy_complete_regression')]:
 copy(base/folder/'replay_verification.json',out/'verification'/(label+'_physical_replay_verification.json'))
for path in (repo/'src/rm_dynamic_obstacle_critic/tools').iterdir():
 if path.suffix in ['.py','.cpp']:copy(path,out/'auditors'/path.name)
copy(Path(__file__),out/'verification/freeze_effective_native_witness.py')
source=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_controller_odom_contract'
metadata={'source_archive':'../gazebo_controller_odom_contract','source_manifest_sha256':hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),'runtime_source_checkpoint':'c775723','protocol_registration_checkpoint':'0d13162','selected_source_ordinal':196,'conditional_witness_verdict':'CONDITIONAL MEASURED-INPUT WITNESSES','original_physical_trial':'FAILED','main_or_hardware_acceptance':False,'runtime_algorithms_changed':False,'selected_sampler_bounds_SG_safe_progress':239,'full_input_sha256':hashlib.sha256((base/'controller_odom_witness_input/input.bin').read_bytes()).hexdigest(),'selected_input_sha256':hashlib.sha256((base/'controller_odom_witness_selected_input/input.bin').read_bytes()).hexdigest()}
(out/'provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
files={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'conditional measured-input native full-horizon witnesses; physical safety/task FAILED, no sampler lack-of-coverage claim','files':files},indent=2)+'\n');print('archived',len(files),'files',sum(path.stat().st_size for path in out.rglob('*') if path.is_file()))
