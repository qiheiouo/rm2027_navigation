import pathlib,json,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2];p=json.loads((root/'experiments/stvl_temporal_roots/protocol.json').read_text())
for spec in ['narrow:CAL:61:aligned']+p['heldout_cases']:
 if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk reserve')
 with (root/'build/roots_cache'/('heldout_'+spec.replace(':','_')+'.log')).open('w') as f:r=subprocess.run(['bash',str(root/'experiments/stvl_temporal_roots/run.sh'),'heldout_repair',spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
 path=root/'build/roots_cache/heldout_repair/runs'/('_'.join(spec.split(':')[:2])+f'_{int(spec.split(":")[2]):02d}_aligned');e=json.loads((path/'events.json').read_text());print(spec,e['finished'],e['result_status'],'contact',len(e['contacts']),flush=True)
 if e['goal_ns'] is None:raise RuntimeError('startup failed; preserved')
 if spec.startswith('narrow:CAL'):
  truth=[json.loads(l) for l in (path/'truth.jsonl').read_text().splitlines()];actor=[r for r in truth if r['model']=='moving_obstacle'];origin=e['goal_ns'];data={'t':[(r['source_ns']-origin)/1e9 for r in actor],'xy':[r['position'][:2] for r in actor]};assert data['t'][-1]>39 and len(data['t'])>1000
  (root/'build/roots_cache/assets/oracle_narrow_61.json').write_text(json.dumps(data,separators=(',',':'))+'\n')
print('untouched-seed finite verification complete',flush=True)
