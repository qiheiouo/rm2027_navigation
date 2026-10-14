"""Preserve real counterexample, synth controls clearly labelled, one-token diff."""
import pathlib,json,copy,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_coasting_epoch/evidence';OUT.mkdir(exist_ok=True);CACHE=ROOT/'build/coasting_epoch_cache';CACHE.mkdir(exist_ok=True)
negative=json.loads((ROOT/'experiments/stvl_active_scope/evidence/active_anchor_negative.json').read_text());c=negative['controller_record'];fresh=dict(case='real_fresh_coasting',prediction=c['prediction']['source'],now_ns=c['prediction']['consume_ns'],expected_before='observation_anchor_epoch',expected_after='')
previous=json.loads((ROOT/'experiments/stvl_prediction_contract/evidence/inputs.json').read_text());lookup={x['case']:x for x in previous}
def real(name,original,before,after):
 x=copy.deepcopy(lookup[original]);x['case']=name;x['prediction']['tracks']=[t for t in x['prediction']['tracks'] if t['state']!=1];x['expected_before']=before;x['expected_after']=after;return x
old=real('real_old_coasting','durable_98','','');confirmed=real('real_confirmed','durable_24','','');stale=real('real_stale','durable_129','stale_observation','stale_observation')
future=copy.deepcopy(fresh);future.update(case='synthetic_future_coasting',expected_before='stale_observation',expected_after='stale_observation');future['prediction']['tracks'][0]['observed_ns']=future['prediction']['source_ns']+1
bad=copy.deepcopy(confirmed);bad.update(case='synthetic_bad_confirmed',expected_before='observation_anchor_epoch',expected_after='observation_anchor_epoch');bad['prediction']['tracks'][0]['observed_ns']-=1
future_source=copy.deepcopy(fresh);future_source.update(case='synthetic_future_source',expected_before='stale_or_future_source',expected_after='stale_or_future_source');future_source['prediction']['source_ns']=future_source['now_ns']+1
incomplete=copy.deepcopy(fresh);incomplete.update(case='synthetic_incomplete',expected_before='incomplete_or_overflow',expected_after='incomplete_or_overflow');incomplete['prediction']['complete']=False
cases=[fresh,old,confirmed,stale,future,bad,future_source,incomplete];(OUT/'inputs.json').write_text(json.dumps(cases,indent=2)+'\n')
original=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014/experiments/stvl_temporal/vendor/model.hpp').read_text();before='observed_ns >= source_ns';after='observed_ns > source_ns';assert original.count(before)==1;modified=original.replace(before,after)
(CACHE/'model_coasting.hpp').write_text(modified)
(OUT/'change.json').write_text(json.dumps(dict(frozen_header_sha256=hashlib.sha256(original.encode()).hexdigest(),generated_header_sha256=hashlib.sha256(modified.encode()).hexdigest(),before=before,after=after,only_occurrence=1,original_header_untouched=True),indent=2)+'\n')
print('8 cases fixed; one generated relational change, no original header edit')
