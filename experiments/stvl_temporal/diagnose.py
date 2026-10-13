"""Offline attribution only: fixed raw candidates; never feeds control."""
import json,pathlib,sys,math
sys.path.insert(0,str(pathlib.Path(__file__).parent));from analyze import jsonlines,prediction_at,circle_gap,stats,CACHE,EVIDENCE
examples=['crossing_B_13','crossing_C_13','sudden_stop_D_13','narrow_D_29_gap','narrow_D_29_aligned','narrow_D_47_aligned','wait_release_C_29_aligned']
result=[]
for name in examples:
 p=CACHE/'runs'/name;e=json.loads((p/'events.json').read_text());rows=jsonlines(p/'control.jsonl');snapshots=[]
 for r in rows:
  if 'raw' not in r or r.get('prediction',{}).get('reason'):continue
  prediction=r.get('prediction',{});payload=prediction.get('source',{});tracks=[t for t in payload.get('tracks',[]) if t['state'] in (2,3)]
  if not tracks:continue
  epoch=prediction.get('candidate_epoch_ns',prediction['consume_ns']);scored=[];nearcost=[];farcost=[];safe=[];progress=[];replay=[]
  for raw in r['raw']:
   stages=[];minimum=math.inf
   for k,pose in enumerate(raw):
    ds=[circle_gap(pose,prediction_at(payload,t,epoch,k),max(.36,math.hypot(*t['size'])/2)) for t in tracks]
    d=min(ds);minimum=min(minimum,d);near=max(0,(.4-d)/.4);danger=max(0,(.02-d)/.02);stages.append((near*near+10000*danger*danger)*.1)
   scored.append(sum(stages));nearcost.append(sum(stages[:10]));farcost.append(sum(stages[10:]));safe.append(minimum>.02);progress.append(minimum>.02 and raw[-1][0]>=r['pose'][0]+.5)
  native=r['critic_deltas'].get('TemporalResearchCritic',[])
  dif=[abs(a-b) for a,b in zip(scored,native)]
  order=sorted(range(len(scored)),key=scored.__getitem__);best=order[0]
  snapshots.append(dict(time_s=(r['epoch_ns']-e['goal_ns'])/1e9,pose=r['pose'],safe_candidates=sum(safe),safe_progress_05m_candidates=sum(progress),max_lateral_displacement_m=max(abs(point[1]-r['pose'][1]) for raw in r['raw'] for point in raw),score_replay_absolute_error=stats(dif),near_first1s_cost=stats(nearcost),far_1to3s_cost=stats(farcost),best_temporal_candidate_near_cost=nearcost[best],best_temporal_candidate_far_cost=farcost[best],soft_all_unsafe=not any(safe),command=r.get('command'),native_other_critic_ranges={k:stats(v) for k,v in r['critic_deltas'].items() if k!='TemporalResearchCritic'}))
 result.append(dict(run=name,snapshots=snapshots))
(EVIDENCE/'diagnostic_replay.json').write_text(json.dumps({'method':'same .34/.29 padded rectangle, R2 signed circle hinge, map->odom identity; subtracting float cost deltas can lose precision at large costs; only attribution, no control','progress_threshold':'.5m relative longitudinal progress, diagnostic only, not a performance success criterion','examples':result},indent=2)+'\n')
for r in result:
 print(r['run'])
 for x in r['snapshots']:print(round(x['time_s'],2),'safe',x['safe_candidates'],'progress',x['safe_progress_05m_candidates'],'replay-max',round(x['score_replay_absolute_error']['max'],3),'best-near/far',round(x['best_temporal_candidate_near_cost'],3),round(x['best_temporal_candidate_far_cost'],3))
