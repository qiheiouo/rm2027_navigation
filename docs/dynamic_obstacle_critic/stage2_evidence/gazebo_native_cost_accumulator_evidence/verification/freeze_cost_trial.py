"""Freeze full raw trial, sources, negative inputs and SDK build evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

repo=Path('/home/qihei/rm2027_navigation');temp=Path('/tmp/rm_native_cost_runtime')
trial=temp/'gazebo_cost_accumulator';unit=Path('/tmp/rm_native_cost_accumulator')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_native_cost_accumulator_evidence'
out.mkdir();checkpoint='9a0d3b5'


def copy(source,target,compressed=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if compressed:
        with source.open('rb') as s,target.open('xb') as d:
            with gzip.GzipFile(filename='',mode='wb',fileobj=d,mtime=0) as z:shutil.copyfileobj(s,z)
    else:shutil.copyfile(source,target)


source=json.loads((trial/'source_identity.json').read_text())
for relative,digest in source.items():
    data=subprocess.check_output(['git','show',checkpoint+':src/rm_dynamic_obstacle_critic/'+relative],cwd=repo)
    if hashlib.sha256(data).hexdigest()!=digest:raise ValueError('runtime source checkpoint mismatch '+relative)
    target=out/'runtime_source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
for path in sorted(trial.iterdir()):
    if path.is_file():
        compressed=path.suffix=='.jsonl' or path.name=='launch.log'
        copy(path,out/(path.name+('.gz' if compressed else '')),compressed)
for name in ('installed_inputs','scene_inputs'):shutil.copytree(trial/name,out/name)
for name in ('cost_before','cost_after','native_cycles','dynamic_scores'):
    for path in sorted((trial/name).iterdir()):
        if path.is_file():copy(path,out/name/(path.name+('.gz' if path.suffix=='.bin' else '')),path.suffix=='.bin')
for path in sorted((repo/'src/rm_dynamic_obstacle_critic/tools').iterdir()):
    if path.suffix in ('.py','.cpp'):copy(path,out/'auditors'/path.name)
for name in ('schedule.json','positive_comparison.json','zero_history_comparison.json'):
    copy(temp/'native_optimizer_replay'/name,out/'native_optimizer'/name)
for name in ('sdk_final_output','zero_history_output','weights_output','uniform_output'):
    copy(temp/'native_optimizer_replay'/(name+'.jsonl'),out/'native_optimizer'/(name+'.jsonl.gz'),True)
copy(temp/'actual_dynamic_replay/input_identity.json',out/'dynamic_replay/input_identity.json')
copy(temp/'actual_dynamic_risk.jsonl',out/'dynamic_replay/native_risk.jsonl.gz',True)
for path in sorted(temp.glob('*.log')):copy(path,out/'verification'/path.name)
copy(temp/'old_dynamic_audit_regression.json',out/'verification/old_dynamic_audit_regression.json')
for path in sorted(unit.glob('*.log')):copy(path,out/'verification/sdk_only'/path.name)
for name in ('plugin_tests','serializer_tests_token','reader_tests','reader_tests_final'):
    for path in sorted((unit/name).rglob('*')):
        if path.is_file():
            relative=path.relative_to(unit/name);compressed=path.suffix=='.bin'
            copy(path,out/'verification/sdk_only'/name/(str(relative)+('.gz' if compressed else '')),compressed)
for name in ('CMakeLists.txt','package.xml','cost_accumulator_critics.xml'):
    copy(unit/'offline'/name,out/'verification/sdk_only/build_source'/name)
for path in sorted((temp/'configure_tests').rglob('*')):
    if path.is_file():copy(path,out/'verification/configure_tests'/path.relative_to(temp/'configure_tests'))
for path in sorted((temp/'chain_negative_probes').rglob('*')):
    if path.is_file() and not path.is_symlink():
        relative=path.relative_to(temp/'chain_negative_probes')
        # Linked originals are referenced by identity; preserve all changed files.
        copy(path,out/'verification/chain_negative_probes'/(str(relative)+('.gz' if path.suffix=='.jsonl' else '')),path.suffix=='.jsonl')
for name in ('rm_dynamic_obstacle_critic','dynamic_safety_guard','native_command_observer','rm_native_cost_accumulator_observer',
             'replay_native_optimizer','replay_dynamic_consumption'):
    for leaf in ('flags.make','link.txt'):
        copy(temp/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/leaf,out/'verification/build'/(name+'_'+leaf))
copy(temp/'build/rm_dynamic_obstacle_critic/compile_commands.json',out/'verification/build/compile_commands.json')
for path in sorted((temp/'build/rm_dynamic_obstacle_critic/test_results').rglob('*')):
    if path.is_file():copy(path,out/'verification/test_results'/path.relative_to(temp/'build/rm_dynamic_obstacle_critic/test_results'))
copy(Path(__file__),out/'verification/freeze_cost_trial.py')
metadata={'runtime_source_checkpoint':checkpoint,'post_trial_auditor_checkpoint':'a96d526',
    'runtime_source_files_verified':len(source),'observer_ELF_sha256':'cf649563b93f7ebaae4b027385806b2c392245c4a749268090815be2f5d9ee95',
    'existing_five_runtime_ELFs_unchanged':True,'observed_cost_pairs':349,'complete_output_chains':347,
    'unassigned_cost_pair_ordinals':[94,95],'actual_risk_rows_exact':104100,'physical_raw203_task_verdict':'FAILED',
    'source_age_value_verdict':'347/347 in original150ms window; callback source stamp unavailable',
    'image_identity':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
    'negative_probe_source_hash_scope':'2892 enumerated input files unchanged; source_identity uses original absolute paths',
    'cost_audit_formatter':'runtime_source/tools/audit_cost_accumulator.py reproduces original two-sided report; newer auditor adds two descriptive missing-output lists',
    'replay_script_status':'captured source before first archive replay; its own SHA is in manifest'}
(out/'archive_identity.json').write_text(json.dumps(metadata,indent=2)+'\n')
(out/'replay_policy.json').write_text(json.dumps({'association_window_seconds':.05,
    'plot_title':'Cost accumulator trial: physical / raw203 / task FAILED','runtime_source_checkpoint':checkpoint,
    'scope':'Default replay uses captured SDK outputs. --native-outputs verifies separately reexecuted outputs, without changing failed gates.'},indent=2)+'\n')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
size=sum((out/name).stat().st_size for name in files)
(out/'manifest.json').write_text(json.dumps({'scope':'Actual rounded accumulator evidence with physical/raw203/task FAILED; unassigned outputs unknown',
    'runtime_source_checkpoint':checkpoint,'files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes_excluding_manifest':size,'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}),flush=True)
