"""Single configuration factor: supply existing measured body velocity."""
import pathlib,json,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2];p=json.loads((root/'experiments/stvl_temporal_roots/protocol.json').read_text())
for cohort,specs in [('repair_odom',p['before_reproduction']),('heldout_odom',['narrow:D:61:aligned','narrow:C:61:aligned'])]:
 for spec in specs:
  if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk reserve')
  with (root/'build/roots_cache'/(cohort+'_'+spec.replace(':','_')+'.log')).open('w') as f:r=subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),cohort,spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
  path=root/'build/roots_cache'/cohort/'runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned');e=json.loads((path/'events.json').read_text());print(cohort,spec,e['finished'],e['result_status'],'contact',len(e['contacts']),flush=True)
  subprocess.run(['python3',str(root/'experiments/stvl_temporal_roots/compress_owned.py')],check=True)
  if e['goal_ns'] is None:raise RuntimeError('preserved startup failure')
print('bounded odometry-contract cohort complete; stop for review',flush=True)
