import gzip, hashlib, json, shutil, subprocess
from pathlib import Path

repo=Path(__file__).resolve().parents[2];work=Path(__file__).parent
input_root=work/'actual_sdk_reexecution'
root=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_native_full_horizon_replay'
root.mkdir(exist_ok=False)

def copy(source,destination):
    if source.is_symlink() or not source.is_file():raise ValueError('regular file required')
    destination.parent.mkdir(parents=True,exist_ok=True)
    if source.suffix in ('.bin','.jsonl'):
        destination.with_name(destination.name+'.gz').write_bytes(gzip.compress(source.read_bytes(),mtime=0))
    else:shutil.copyfile(source,destination)

for p in sorted(input_root.rglob('*')):
    if p.is_file():copy(p,root/'execution'/p.relative_to(input_root))
for name in ('recover_sdk.py','finish_sdk.py','sdk_build.sh','sdk_build_driver.log','recover_sdk_driver.log','finish_sdk_driver.log'):
    copy(work/name,root/'verification'/name)
for name in ('recover_sdk_initial_source.py','recover_sdk_schema_repaired_source.py'):
    copy(work/'failures'/name,root/'verification/failures'/name)
copy(Path(__file__),root/'verification'/Path(__file__).name)
copy(work/'sdk_build/build.log',root/'verification/build.log')
for name in ('replay_observation_consumption','replay_native_optimizer','native_safe_control_witness'):
    copy(work/'sdk_build/build/rm_dynamic_obstacle_critic'/name,root/'helpers'/name)
    copy(work/'sdk_build/build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/'flags.make',root/'helpers'/(name+'.flags.make'))
copy(work/'sdk_build/build/rm_dynamic_obstacle_critic/CMakeCache.txt',root/'verification/CMakeCache.txt')
dependency=repo/'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight'
prior=json.loads((dependency/'initial_execution/checkpoint_summary.json').read_text())
checks={};summary={}
for mode in ('visible','nominal'):
    report=json.loads((input_root/mode/'witness/geometry.json').read_text())
    actual=[next(p for p in r['proposals'] if p['name']=='actual_aggregate') for r in report['records']]
    gates={k:sum(p['gates'][k] for p in actual) for k in actual[0]['gates']}
    summary[mode]={'cycles':report['cycles'],'aggregate_gate_counts':gates,'safe_context':report['counts']['actual_aggregate:safe_context'],'safe_progress':report['counts']['actual_aggregate:safe_progress'],
                   'witness_payload_sha256':report['witness_payload_sha256'],'prepared_input_sha256':report['prepared_input_sha256']}
    old=prior['modes'][mode]
    checks[mode+' prior checkpoint gate counts']=gates==old['actual_aggregate_full_3s_gate_counts'] and report['cycles']==old['all_native_cycles'] and summary[mode]['safe_context']==old['actual_aggregate_safe_context_count'] and summary[mode]['safe_progress']==old['actual_aggregate_safe_progress_count']
if not all(checks.values()):raise ValueError('prior summary differs')
identity={'base_checkpoint':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
          'sdk_image':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
          'audit_source_manifest_sha256':hashlib.sha256((dependency/'manifest.json').read_bytes()).hexdigest(),
          'build_source_manifest_sha256':hashlib.sha256((dependency.parent/'observation_diameter_preflight/manifest.json').read_bytes()).hexdigest(),
          'sdk_checks':json.loads((input_root/'sdk_reexecution.json').read_text()),'prior_checkpoint_comparison':checks,'summary':summary,
          'limits':'Fresh full replay after temporary output loss; prior full geometry files unavailable, only durable checkpoint counts compared. Native SDK/risk outputs exact against original archives. Conditional source-time 3s model geometry is not actual guard/execution or future map certification. Full raw300 geometry pending. Original trials FAILED; real e/B and response/braking bounds unknown.'}
(root/'recovery_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file()}
manifest={'files':files,'bytes':sum((root/k).stat().st_size for k in files)}
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':manifest['bytes'],'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),'prior_checkpoint_comparison':checks}))
