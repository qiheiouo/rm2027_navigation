"""Copy four selected immutable input snapshots, no ground-truth input."""
import pathlib,json,gzip,copy
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_prediction_contract/evidence';OUT.mkdir(exist_ok=True)
p=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014/build/c83_cache/shadow_durable/runs/narrow_C_83_aligned/control.jsonl')
with p.open() if p.exists() else gzip.open(str(p)+'.gz','rt') as f:a=[json.loads(l) for l in f]
selected=[]
for cycle in (24,98,99,129):
 x=next(x for x in a if x['cycle_id']==cycle);prediction=x['prediction'];selected.append(dict(case='durable_'+str(cycle),now_ns=prediction['consume_ns'],prediction=prediction['source'],historical_reason=prediction.get('reason','')))
bad=copy.deepcopy(selected[0]);bad['case']='synthetic_bad_active_epoch';active=next(x for x in bad['prediction']['tracks'] if x['state']==2);active['observed_ns']-=1;bad['historical_reason']='synthetic'
empty=copy.deepcopy(selected[0]);empty['case']='synthetic_no_active_tracks';empty['prediction']['tracks']=[];empty['historical_reason']='synthetic'
selected+=[bad,empty]
origin=next(x for x in a if x['cycle_id']==0)['prediction'];selected.append(dict(case='durable_0_original_empty',now_ns=origin['consume_ns'],prediction=origin['source'],historical_reason=origin.get('reason','')))
(OUT/'inputs.json').write_text(json.dumps(selected,indent=2)+'\n');print('five historical and two synthetic snapshots, all labelled')
