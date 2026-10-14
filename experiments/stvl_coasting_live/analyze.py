"""Reuse frozen physical gates; add actual coasting reception evidence."""
import importlib.util,json,pathlib,collections,math
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/coasting_live_cache';OUT=ROOT/'experiments/stvl_coasting_live/evidence';OUT.mkdir(exist_ok=True)
spec=importlib.util.spec_from_file_location('active_stage_metrics',ROOT/'experiments/stvl_active_scope/analyze.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.OUT=CACHE/'derived_metrics';m.OUT.mkdir(exist_ok=True)
def latency_witness(p,cs):
 timings=m.jsonlines(p/'paired_timing.jsonl');receipts=[x for x in m.rows(p/'command_receipts.csv') if x['topic']=='/cmd_vel_nav'];byid={x['cycle_id']:x for x in cs};used=set();matches=[]
 for t in timings:
  c=byid.get(t['cycle_id']);cmd=c.get('command') if c else None
  if cmd is None:continue
  hit=next(((i,z) for i,z in enumerate(receipts) if i not in used and t['entry_steady_ns']<=int(z['steady_ns'])<=t['entry_steady_ns']+500000000 and max(abs(float(z[k])-v) for k,v in zip(('vx','vy','wz'),cmd))<1e-7),None)
  if hit:
   i,z=hit;used.add(i);pub=int(z['steady_ns'])+int(z['DDS_source_ns'])-int(z['system_ns']);matches.append((pub-t['entry_steady_ns'],t,c,i,z,pub))
 if not matches:return None
 delta,t,c,index,z,pub=max(matches,key=lambda x:x[0]);start=t['entry_steady_ns'];rows=[x for x in m.rows(p/'native_boundary.csv') if start<=int(x['steady_ns'])<=pub]
 travel=sum(math.hypot(float(b['x'])-float(a['x']),float(b['y'])-float(a['y'])) for a,b in zip(rows,rows[1:]));tail=[x for x in rows if int(x['steady_ns'])>=start+100000000]
 tail_travel=sum(math.hypot(float(b['x'])-float(a['x']),float(b['y'])-float(a['y'])) for a,b in zip(tail,tail[1:]));previous=receipts[index-1] if index>0 else None
 previous_pub=int(previous['steady_ns'])+int(previous['DDS_source_ns'])-int(previous['system_ns']) if previous else None
 return {'cycle_id':c['cycle_id'],'controller_record':c,'paired_timing_record':t,'matched_DDS_receipt':z,'previous_NAV_receipt':previous,'entry_to_DDS_publication_ms':delta/1e6,'over100_ms':max(0,delta/1e6-100),'previous_to_new_NAV_publication_ms':(pub-previous_pub)/1e6 if previous_pub else None,'previous_NAV_publication_to_entry_ms':(start-previous_pub)/1e6 if previous_pub else None,'native_samples_in_window':len(rows),'sampled_window_travel_m':travel,'native_samples_after_100ms_before_new_publication':len(tail),'sampled_tail_travel_m':tail_travel,'first_native':rows[0] if rows else None,'last_native':rows[-1] if rows else None,'scope':'actual sampled execution during wall-time computation window; not constant old Twist reconstruction or collision causality; omit endpoint interpolation; numeric samples at 1ms physics','tail_resolution_note':'only 1 sampled native pose beyond 100ms in allow_equal; tail_travel=0 is an empty interval sum, not proof of no sub-2ms motion'}
def main():
 rows=[];counterexamples=[];inputs=[];failures=[]
 for p in sorted(CACHE.glob('*/runs/*')):
  kind=p.parent.parent.name
  if not (p/'events.json').exists():failures.append({'variant':kind,'reason':'no events, engineering start incomplete'});continue
  r=m.run(p);r['variant']=kind;r.pop('input_scope',None);r.pop('projection',None)
  e=json.loads((p/'events.json').read_text());cs=[x for x in m.jsonlines(p/'control.jsonl') if e['goal_ns'] is not None and e['goal_ns']<=x['epoch_ns']<=e['end_ns']]
  accept=[];reject=[];ages=[];observation_ages=[]
  for c in cs:
   pr=c.get('prediction',{});source=pr.get('source',{});tracks=source.get('tracks',[]);fresh=[t['id'] for t in tracks if t['state']==3 and t['observed_ns']==source['source_ns']]
   if source and pr.get('consume_ns'):
    ages.append((pr['consume_ns']-source['source_ns'])/1e9)
    observation_ages.extend((pr['consume_ns']-t['observed_ns'])/1e9 for t in tracks if t['state'] in (2,3))
   if fresh:
    (reject if pr.get('reason') else accept).append(c['cycle_id'])
  for cycles,tag in ((accept,'accepted_fresh'),(reject,'rejected_fresh')):
   if cycles:
    c=next(x for x in cs if x['cycle_id']==cycles[0]);counterexamples.append({'variant':kind,'kind':tag,'controller_record':c})
    # Freeze an actual accepted input for the existing before/after validator.
    # This proves the narrow time-relation effect on identical source data.
    if tag=='accepted_fresh' and kind=='allow_equal':
     inputs.append({'case':'live_allow_equal_fresh_c'+str(c['cycle_id']),'prediction':c['prediction']['source'],'now_ns':c['prediction']['consume_ns'],'expected_before':'observation_anchor_epoch','expected_after':''})
  for reason in ('observation_anchor_epoch','stale_observation'):
   c=next((x for x in cs if x.get('prediction',{}).get('reason')==reason),None)
   if c:counterexamples.append({'variant':kind,'kind':reason,'controller_record':c})
  r['fresh_coasting']={'accepted_cycles':accept,'rejected_cycles':reject};r['prediction_source_age_s']=m.stats(ages);r['active_observation_age_s']=m.stats(observation_ages)
  r['diagnostic_export_disabled_all']=bool(cs) and all(x.get('candidate_export_enabled') is False and x.get('raw_candidates_retained')==0 for x in cs)
  r['raw_candidate_file_absent']=not (p/'paired_timing.jsonl.raw').exists() and not (p/'paired_timing.jsonl.raw.gz').exists()
  r['native_scoring_attempt_counts']=dict(collections.Counter(str(x['native_scoring_attempts']) for x in cs));r['raw_read_ms']=m.stats([x['timing']['raw_read_ms'] for x in cs]);r['effective_controller_records']=len(cs)
  w=latency_witness(p,cs);(OUT/(kind+'_latency_witness.json')).write_text(json.dumps(w,indent=2)+'\n')
  (OUT/(kind+'.json')).write_text(json.dumps(r,indent=2)+'\n');rows.append(r)
 (OUT/'paired_summary.json').write_text(json.dumps(rows,indent=2)+'\n');(OUT/'actual_consumption_examples.json').write_text(json.dumps(counterexamples,indent=2)+'\n');(OUT/'validator_inputs.json').write_text(json.dumps(inputs,indent=2)+'\n');(OUT/'incomplete_starts.json').write_text(json.dumps(failures,indent=2)+'\n')
 for r in rows:print(json.dumps({k:r.get(k) for k in ('variant','hard_gate_pass','arrival_s','path_length_m','body_clearance_m','base_collision_box_gap_m','native_errors','prediction_reason_counts','zero_safe_candidates','fresh_coasting','diagnostic_export_disabled_all','entry_to_DDS_publication_ms','entry_to_DDS_over100')}))
if __name__=='__main__':main()
