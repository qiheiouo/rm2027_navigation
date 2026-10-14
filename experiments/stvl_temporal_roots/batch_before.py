import json,pathlib,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2];p=json.loads((root/'experiments/stvl_temporal_roots/protocol.json').read_text())
for spec in p['before_reproduction']:
 if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk reserve')
 path=root/'build/roots_cache/before/runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned')
 if path.exists():raise RuntimeError('existing case; never overwrite '+str(path))
 with (root/'build/roots_cache'/('before_'+spec.replace(':','_')+'.log')).open('w') as f:r=subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),'before',spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
 if not (path/'events.json').exists():raise RuntimeError('incomplete before '+spec)
 e=json.loads((path/'events.json').read_text());print(spec,e['finished'],e['result_status'],'contact',len(e['contacts']),flush=True)
 if e['goal_ns'] is None:raise RuntimeError('startup classified separately')
print('targeted original reproduction complete',flush=True)
