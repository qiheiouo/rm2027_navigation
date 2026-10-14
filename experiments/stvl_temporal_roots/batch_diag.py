import pathlib,subprocess,json
root=pathlib.Path(__file__).resolve().parents[2]
for spec in ['wait_release:C:29:aligned','stale:C:29:aligned','narrow:D:29:aligned']:
 with (root/'build/roots_cache'/('diag_'+spec.replace(':','_')+'.log')).open('w') as f:r=subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),'diag',spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
 p=root/'build/roots_cache/diag/runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned');e=json.loads((p/'events.json').read_text());print(spec,e['finished'],e['result_status'],len(e['contacts']),flush=True)
print('targeted same-score diagnostic cohort complete',flush=True)
