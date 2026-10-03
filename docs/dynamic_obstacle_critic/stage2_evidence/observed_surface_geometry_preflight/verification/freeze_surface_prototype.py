import gzip, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

repo=Path(__file__).resolve().parents[2]
work=Path(__file__).parent
root=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observed_surface_geometry_preflight'
root.mkdir(exist_ok=False)

def copy(source,destination):
    if not source.is_file() or source.is_symlink():raise ValueError('regular file required '+str(source))
    destination.parent.mkdir(parents=True,exist_ok=True)
    if source.suffix=='.jsonl':
        destination=destination.with_name(destination.name+'.gz')
        destination.write_bytes(gzip.compress(source.read_bytes(),mtime=0))
    else:shutil.copyfile(source,destination)

for p in sorted((repo/'src/rm_dynamic_obstacle_tracking').rglob('*')):
    if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts:
        copy(p,root/'source'/p.relative_to(repo))
for name in ('docs/dynamic_obstacle_critic/dynamic_surface_reveal_experiment.md','docs/dynamic_obstacle_critic/stage2_progress.md','docs/contracts/tf_contract_2027.md','docs/development_workflow.md'):
    copy(repo/name,root/'source'/name)
for name in ('surface_unit.log','surface_range_units.log','quality_visible.log','quality_nominal.log','quality_visible_range_labels.log','quality_nominal_range_labels.log','frozen_reveal_driver.log','replay_frozen_reveal.py'):
    copy(work/name,root/'verification'/name)
for name in ('tuple_index_source.py','tuple_index_failure.log'):
    copy(work/'failures'/name,root/'failures'/name)
for mode in ('visible','nominal'):
    for part in ('quality_'+mode,'quality_'+mode+'_range_labels'):
        for p in sorted((work/part).iterdir()):copy(p,root/'initial_execution'/part/p.name)
for p in sorted((work/'frozen_reveal_reexecution').iterdir()):
    if p.is_file():copy(p,root/'frozen_reveal_reexecution'/p.name)
copy(Path(__file__),root/'verification'/Path(__file__).name)

dependency=repo/'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight'
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join([str(root/'source/src/rm_dynamic_obstacle_tracking'),str(dependency/'source/src/rm_dynamic_obstacle_critic/tools')]))
checks={}
with (root/'verification/frozen_source_units.log').open('x') as log:
    subprocess.run([sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',str(root/'source/src/rm_dynamic_obstacle_tracking/test/test_observed_surface_geometry.py'),str(root/'source/src/rm_dynamic_obstacle_tracking/test/test_surface_range_labels.py')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
for mode in ('visible','nominal'):
    output=work/('frozen_quality_'+mode)
    with (root/'verification'/('frozen_quality_'+mode+'.log')).open('x') as log:
        subprocess.run([sys.executable,'-B',str(root/'source/src/rm_dynamic_obstacle_tracking/tools/audit_observed_surface_geometry.py'),str(root.parent/('gazebo_observation_v2_'+mode)),str(output)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    for p in sorted(output.iterdir()):
        checks[mode+' '+p.name]=(work/('quality_'+mode+'_range_labels')/p.name).read_bytes()==p.read_bytes()
        copy(p,root/'independent_reexecution'/mode/p.name)
report={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,'scope':'Frozen source reports and all source-indexed cluster records byte exact; actual tracker members remain unknown'}
(root/'independent_reexecution/comparison.json').write_text(json.dumps(report,indent=2)+'\n')
if not all(checks.values()):raise ValueError('frozen surface replay differs')
identity={'base_checkpoint':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
          'source_scope':'New offline prototype plus frozen current tracking package; no ROS node invocation or formal config change',
          'audit_dependency_manifest_sha256':hashlib.sha256((dependency/'manifest.json').read_bytes()).hexdigest(),
          'inputs':{mode:hashlib.sha256((root.parent/('gazebo_observation_v2_'+mode)/'manifest.json').read_bytes()).hexdigest() for mode in ('visible','nominal')},
          'unit_checks':21,'independent_cluster_report_checks':4,'independent_reveal_report_figure_checks':8,
          'limits':'Nominal chords, hidden geometry, future motion and real brake bounds uncertified. Source-time canonical replay is not actual internal TF/membership. No C trial. Failed tuple index and initial coarse labels preserved.'}
(root/'audit_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file()}
manifest={'files':files,'bytes':sum((root/k).stat().st_size for k in files)}
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':manifest['bytes'],'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),'comparison':report}))
