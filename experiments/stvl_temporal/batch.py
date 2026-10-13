"""Fixed paired schedule; every started failure stays in its cohort."""
import json,pathlib,subprocess,time
root=pathlib.Path(__file__).resolve().parents[2];cache=root/'build/stvl_cache';protocol=json.loads((root/'experiments/stvl_temporal/protocol.json').read_text())
schedule=[]
for scene in protocol['scenes']:
 for seed in protocol['p2_seeds']:
  if scene!='empty':schedule.append(f'{scene}:CAL:{seed}')
  groups=protocol['groups'] if seed==13 else list(reversed(protocol['groups']))
  for group in groups:schedule.append(f'{scene}:{group}:{seed}')
(cache/'schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
for spec in schedule:
 scene,mode,seed=spec.split(':');directory=cache/'runs'/f'{scene}_{mode}_{int(seed):02d}'
 if directory.exists():
  if not (directory/'events.json').exists():raise RuntimeError('incomplete preserved run '+spec)
  event=json.loads((directory/'events.json').read_text())
  if event['goal_ns'] is None:raise RuntimeError('unclassified startup '+spec)
  print('preserved',spec,event['finished'],flush=True)
 else:
  with (cache/f'{scene}_{mode}_{seed}_runner.log').open('w') as log:
   result=subprocess.run(['bash',str(root/'experiments/stvl_temporal/run.sh'),spec],cwd=root,stdout=log,stderr=subprocess.STDOUT)
  if not (directory/'events.json').exists():raise RuntimeError('no events '+spec)
  event=json.loads((directory/'events.json').read_text())
  print(spec,event['finished'],'action',event['result_status'],'contact',len(event['contacts']),'native_errors',len(event['native_mppi_errors']),flush=True)
  if event['goal_ns'] is None:raise RuntimeError('startup stopped; preserve/classify before resuming '+spec)
 if mode=='CAL':
  # Separate actual actor run: no goal/robot movement and no runtime truth input
  # to any reactive group. Oracle is explicitly replay, checked after runs.
  truth=[json.loads(l) for l in (directory/'truth.jsonl').read_text().splitlines()]
  actor=[v for v in truth if v['model']=='moving_obstacle']
  origin=event['goal_ns'];data={'t':[(v['source_ns']-origin)/1e9 for v in actor],'xy':[v['position'][:2] for v in actor]}
  assert len(data['t'])>1000 and data['t'][-1]>39
  (cache/'assets'/f'oracle_{scene}_{seed}.json').write_text(json.dumps(data,separators=(',',':'))+'\n')
print('fixed finite schedule complete',flush=True)
