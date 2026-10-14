"""SOURCE-DERIVED catch semantics only: not a live Nav2 server or execution run.
Link: navigation2/1.1.20/nav2_controller/src/controller_server.cpp:442-467.
No output publisher, no dynamics and no conversion of arbitrary runtime errors.
"""
import json,pathlib,gzip
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_failure_contract/evidence';OUT.mkdir(exist_ok=True)
def run(case,typed):
 last_valid_ms=0;events=[]
 for time_ms in (0,100,200,300,400,500):
  # Controlled fixed 0.3s patience; choose errors from the registered schedule.
  fail=(case=='persistent_failure' and time_ms>=100) or (case=='transient_failure' and time_ms==100)
  if fail:
   if not typed:
    events.append(dict(time_ms=time_ms,output='zero',state='abort_outer_runtime'));break
   if (time_ms-last_valid_ms)/1000>.3:
    events.append(dict(time_ms=time_ms,output='zero',state='abort_patience'));break
   events.append(dict(time_ms=time_ms,output='zero',state='continue_patience'))
  else:
   last_valid_ms=time_ms;events.append(dict(time_ms=time_ms,output='fresh_control',state='continue'))
 return dict(case=case,known_optimizer_exhaustion_as_typed=typed,events=events,scope='semantic replay, no native server or motion')
result=[run(c,t) for c in ('normal','transient_failure','persistent_failure') for t in (False,True)]
assert result[0]['events']==result[1]['events']
assert result[2]['events'][-1]['state']=='abort_outer_runtime'
assert result[3]['events'][1]['output']=='zero' and result[3]['events'][2]['output']=='fresh_control'
assert result[4]['events'][-1]['time_ms']==100 and result[5]['events'][-1]['time_ms']==400
(OUT/'server_semantics.json').write_text(json.dumps(result,indent=2)+'\n')
history=[]
for label,fn in (
 ('original','/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned/control.jsonl'),
 ('durable162','/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014/build/c83_cache/shadow_durable/runs/narrow_C_83_aligned/control.jsonl')):
 p=pathlib.Path(fn)
 with p.open() if p.exists() else gzip.open(str(p)+'.gz','rt') as f:
  for i,line in enumerate(f):
   x=json.loads(line)
   if not x.get('error'):continue
   reason=x.get('prediction',{}).get('reason')
   history.append(dict(source=label,cycle_id=x.get('cycle_id',i),error=x['error'],prediction_failure_reason=reason,native_scoring_attempts=x.get('native_scoring_attempts'),scope='input-validity rejection independent of resampling in same frozen snapshot' if reason in ('observation_anchor_epoch','stale_observation') else 'failure source not identified by retained fields; do not assign to prediction'))
(OUT/'historical_failures.json').write_text(json.dumps(history,indent=2)+'\n')
lines=(ROOT/'build/lifecycle_cache/failure_contract_probe.log').read_text().splitlines();probe=json.loads(next(x for x in lines if x.startswith('{"native_version"')));assert probe['predeclared_acceptance_pass'];(OUT/'native_pairs.json').write_text(json.dumps(probe,indent=2)+'\n')
print(json.dumps(dict(native_pairs=len(probe['pairs']),semantic_cases=len(result),historical_failures=len(history),frozen_prediction_rejections=sum(x['prediction_failure_reason'] is not None for x in history))))
