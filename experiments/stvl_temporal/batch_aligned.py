"""One frozen time-reference intervention, original seeds and untouched holdout."""
import json,pathlib,subprocess,shutil
root=pathlib.Path(__file__).resolve().parents[2];cache=root/'build/stvl_cache';p=json.loads((root/'experiments/stvl_temporal/p3_protocol.json').read_text())
schedule=[]
for seed in p['repeat_seeds']+[p['untouched_holdout_seed']]:
 for scene in p['scenes']:
  if seed==47 and scene!='empty':schedule.append(f'{scene}:CAL:{seed}:aligned')
  for mode in (p['groups'] if seed!=29 else list(reversed(p['groups']))):schedule.append(f'{scene}:{mode}:{seed}:aligned')
(cache/'schedule_aligned.json').write_text(json.dumps(schedule,indent=2)+'\n')
for spec in schedule:
 scene,mode,seed,tag=spec.split(':');directory=cache/'runs'/f'{scene}_{mode}_{int(seed):02d}_{tag}'
 if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('disk protection line; stop writes')
 usage=sum(f.stat().st_size for f in cache.rglob('*') if f.is_file())
 if usage>900*1024**2:raise RuntimeError('cache budget; preserve evidence before another trial')
 if not directory.exists():
  with (cache/f'{scene}_{mode}_{seed}_aligned_runner.log').open('w') as log:result=subprocess.run(['bash',str(root/'experiments/stvl_temporal/run.sh'),spec],cwd=root,stdout=log,stderr=subprocess.STDOUT)
 if not (directory/'events.json').exists():raise RuntimeError('preserved incomplete trial '+spec)
 event=json.loads((directory/'events.json').read_text())
 print(spec,event['finished'],'action',event['result_status'],'contact',len(event['contacts']),'native_errors',len(event['native_mppi_errors']),flush=True)
 if event['goal_ns'] is None:raise RuntimeError('preserved startup; classify before resuming '+spec)
 if mode=='CAL':
  actor=[json.loads(l) for l in (directory/'truth.jsonl').read_text().splitlines() if json.loads(l)['model']=='moving_obstacle'];origin=event['goal_ns']
  data={'t':[(v['source_ns']-origin)/1e9 for v in actor],'xy':[v['position'][:2] for v in actor]};assert len(data['t'])>1000 and data['t'][-1]>39
  (cache/'assets'/f'oracle_{scene}_{seed}.json').write_text(json.dumps(data,separators=(',',':'))+'\n')
print('finite single-factor P3 complete',flush=True)
