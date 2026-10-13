import json,pathlib,sys,math,csv
sys.path.insert(0,'experiments/stvl_temporal');from analyze import stats,jsonlines,CACHE
root=pathlib.Path.cwd();s=json.loads((root/'experiments/stvl_temporal/evidence/summary.json').read_text());out=[]
for variant in ['consume','input_pose']:
 for group in 'ABCD':
  rows=[r for r in s['trials'] if r['phase']=='finite' and r['cohort']=='final_fixture' and r['variant']==variant and r['group']==group];cycles=[];ages=[];gaps=[];zeros=[];future={str(t):[] for t in (0,1,3)};epocherrors=[];refdif=[];poseages=[]
  for r in rows:
   p=CACHE/'runs'/r['run'];e=json.loads((p/'events.json').read_text());controls=[x for x in jsonlines(p/'control.jsonl') if e['goal_ns']<=x['epoch_ns']<=e['end_ns']];cycles.extend(x['compute_ms'] for x in controls)
   for x in controls:
    pr=x.get('prediction',{});src=pr.get('source',{});
    if 'age_s' in pr:ages.append(pr['age_s']*1000)
    if 'candidate_epoch_ns' in pr and x.get('pose_epoch_ns',0)>0:epocherrors.append((pr['candidate_epoch_ns']-x['pose_epoch_ns'])/1e6)
    if 'consume_ns' in pr and x.get('pose_epoch_ns',0)>0:refdif.append((pr['consume_ns']-x['pose_epoch_ns'])/1e6)
    if x.get('pose_epoch_ns',0)>0:poseages.append((x['epoch_ns']-x['pose_epoch_ns'])/1e6)
   pk=[x for x in csv.DictReader((p/'commands.csv').open()) if x['topic']=='/simulation/chassis/cmd_vel' and e['goal_ns']<=int(x['receipt_ns'])<=e['end_ns']]
   for a,b in zip(pk,pk[1:]):
    gap=(int(b['receipt_ns'])-int(a['receipt_ns']))/1e6
    (zeros if all(abs(float(a[k]))<1e-5 for k in ('vx','vy','wz')) else gaps).append(gap)
  out.append(dict(variant=variant,group=group,n=len(rows),goal_success=sum(r['success'] for r in rows),safe_success=sum(r['safe_success'] for r in rows),contact_trials=sum(r['contact_messages']>0 for r in rows),compute_ms=stats(cycles),over100=sum(v>100 for v in cycles),prediction_age_ms=stats(ages),query_to_input_pose_ms=stats(epocherrors),consume_to_input_pose_ms=stats(refdif),input_pose_age_ms=stats(poseages),active_output_gap_ms=stats(gaps),zero_hold_output_gap_ms=stats(zeros),active_output_gaps_over100=sum(v>100 for v in gaps),zero_hold_output_gaps_over100=sum(v>100 for v in zeros),native_errors=sum(r['native_errors'] for r in rows),command_bounds_violations=sum(r['output_bounds_violations'] for r in rows)))
(root/'experiments/stvl_temporal/evidence/aggregate_metrics.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
