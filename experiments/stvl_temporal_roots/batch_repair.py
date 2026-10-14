import pathlib,json,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2];p=json.loads((root/'experiments/stvl_temporal_roots/protocol.json').read_text())
for spec in p['before_reproduction']:
 if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk reserve')
 with (root/'build/roots_cache'/('repair_'+spec.replace(':','_')+'.log')).open('w') as f:r=subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),'repair',spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
 path=root/'build/roots_cache/repair/runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned');e=json.loads((path/'events.json').read_text());print(spec,e['finished'],e['result_status'],'contact',len(e['contacts']),flush=True)
 if e['goal_ns'] is None:raise RuntimeError('preserved startup failure')
print('single-factor paired repair cohort complete',flush=True)
