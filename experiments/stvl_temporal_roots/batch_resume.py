"""Continue fixed cases after preserving a startup-only failure."""
import pathlib,json,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2]
for cohort,spec in [('heldout_abi_retry','stale:C:61:aligned'),('heldout_abi','narrow:D:61:aligned'),('heldout_abi','narrow:C:61:aligned')]:
 if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk reserve')
 with (root/'build/roots_cache'/(cohort+'_'+spec.replace(':','_')+'.log')).open('w') as f:subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),cohort,spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
 path=root/'build/roots_cache'/cohort/'runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned');e=json.loads((path/'events.json').read_text());print(cohort,spec,e['finished'],e['result_status'],'contact',len(e['contacts']),flush=True)
 subprocess.run(['python3',str(root/'experiments/stvl_temporal_roots/compress_owned.py')],check=True)
 if e['goal_ns'] is None:raise RuntimeError('retry startup failed, preserve and stop')
