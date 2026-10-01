import gzip,hashlib,json,shutil
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_weight_attribution';out.mkdir()
def copy(src,dst,compress=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 if compress:
  with dst.open('xb') as stream:
   with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as gz:gz.write(src.read_bytes())
 else:shutil.copyfile(src,dst)
for label,name in [('native','native_weight_final_output.jsonl'),('uniform','native_weight_uniform_output.jsonl')]:copy(base/name,out/(label+'_output.jsonl.gz'),True)
for label,name in [('native','native_weight_analysis.json'),('uniform','native_weight_uniform_analysis.json')]:copy(base/name,out/(label+'_analysis.json'))
for extension in ['png','svg']:copy(base/('native_weight_distribution.'+extension),out/('native_weight_distribution.'+extension))
for name in ['native_weight_build.log','native_weight_run.log','native_weight_default_run.log','native_weight_negative_build.log','native_weight_negative_run.log','native_weight_original_output_regression.json','native_weight_binary_identity.txt','native_weight_final_binary_identity.txt']:
 copy(base/name,out/'verification'/name)
copy(repo/'build/dynamic_research_device_20260930/upstream/navigation2-1.1.20/nav2_mppi_controller/src/optimizer.cpp',out/'verification/pinned_optimizer.cpp')
copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles/replay_native_optimizer.dir/flags.make',out/'verification/offline_flags.make')
for name in ['replay_native_optimizer.cpp','analyze_native_weights.py','plot_native_weights.py','prepare_native_optimizer_replay.py','compare_native_optimizer_replay.py','replay_native_weight_evidence.py','native_snapshot_io.py','trial_io.py']:
 copy(repo/'src/rm_dynamic_obstacle_critic/tools'/name,out/'auditors'/name)
copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt',out/'offline_CMakeLists.txt')
copy(Path(__file__),out/'verification/freeze_native_weight_evidence.py')
sources={}
for name,folder in [('physical','gazebo_controller_odom_contract'),('witness','effective_native_witness')]:
 sources[name]={'relative_path':'../'+folder,'manifest_sha256':hashlib.sha256((repo/'docs/dynamic_obstacle_critic/stage2_evidence'/folder/'manifest.json').read_bytes()).hexdigest()}
metadata={'registration_checkpoint':'a56b6a1','negative_registration_checkpoint':'695427c','source_archives':sources,'input_sha256':hashlib.sha256((base/'controller_odom_archive_replay/replayed_optimizer/input.bin').read_bytes()).hexdigest(),'runtime_algorithms_changed':False,'five_runtime_ELFs_exact_previous':True,'selected_source_ordinal':196,'native_bounded_means_bit_exact':350,'uniform_probe_bounded_means_bit_exact':0,'all_actual_SG_chains_unchanged':True,'original_physical_trial':'FAILED','cost_contribution_limitation':'native total costs only; exact Dynamic consumer/per-critic contribution unavailable','source_label_limitation':'individually constrained/SG counterfactual rows, not raw rollout or weighted output geometry'}
(out/'provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
files={str(path.relative_to(out)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'exact native total cost/softmax weights and conditional-label attribution; no physical acceptance or critic-specific causality','files':files},indent=2)+'\n');print('archived',len(files),'files',sum(path.stat().st_size for path in out.rglob('*') if path.is_file()))
