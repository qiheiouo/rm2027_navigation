"""Original C83 only, reconstruct display inputs and independently evaluate prefix.
No original raw writes; future truth is retrospective, no inevitability proof.
"""
import pathlib,json,sys,gzip,math,bisect,subprocess,os
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/c83_cache';OUT=ROOT/'experiments/stvl_c83_endtoend/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_c83_endtoend'));from causal_replay import lines,actor_at
sys.path.insert(0,str(ROOT/'experiments/stvl_execution_realtime'));from mapping import previous_yaw_view,rows
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned')

def inverse_sdk(c):
 last=c['pose'];vs=[]
 for k,p in enumerate(c['aggregate']):
  yaw=last[2] if k==0 else p[2];dx=(p[0]-last[0])/.1;dy=(p[1]-last[1])/.1
  vs.append([math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,(p[2]-last[2])/.1]);last=p
 return vs

def run():
 event=json.loads((OLD/'events.json').read_text());goal=event['goal_ns'];truth=lines(OLD/'truth.jsonl');actor=[x for x in truth if x['model']=='moving_obstacle'];ts=[x['source_ns'] for x in actor];bytime={}
 for x in truth:bytime.setdefault(x['source_ns'],{})[x['model']]=x
 gaps=[dict(relative_s=(t-goal)/1e9,source_ns=t,gap_m=analyze.polygon_distance(x['rm_sentry_2027']['polygon'],x['moving_obstacle']['polygon'])) for t,x in sorted(bytime.items()) if goal<=t<=event['end_ns'] and len(x)==2]
 rowsout=[];static=json.loads((CACHE/'no_shadow_retry/assets/narrow_static.json').read_text());fp=[[-.34,-.29],[.34,-.29],[.34,.29],[-.34,.29]]
 temp=CACHE/'original_input.json'
 for i,c in enumerate(lines(OLD/'control.jsonl')):
  rel=(c['epoch_ns']-goal)/1e9;pr=c.get('prediction',{});source=pr.get('source',{});t=c['pose_epoch_ns'];active=any(x['state'] in (2,3) for x in source.get('tracks',[]));selected=None;prefix=[];firstv_error=None
  if 'aggregate' in c:
   vs=inverse_sdk(c);firstv_error=max(abs(a-b) for a,b in zip(vs[0],c['command']));poses=previous_yaw_view(c['pose'],vs)
   ctx=dict(case=f'original__{i}',pose=c['pose'],speed=c['speed'],pose_epoch_ns=t,dt=.1,identity_frame_fixture=True,execution_mapping_known=False,feedback_consistent=True,footprint=fp,prediction=pr,static_map=static,trajectories=[dict(id='selected_inferred_sdk_controls',poses=poses,velocities=vs)])
   temp.write_text(json.dumps(ctx,separators=(',',':'))+'\n');selected=json.loads(subprocess.check_output([str(CACHE/'plugin_build/replay_cycle'),str(temp)],text=True))[0]
   for k,p in enumerate(poses):
    a=actor_at(actor,ts,t+(k+1)*100000000)
    if a is not None:prefix.append(dict(step=k+1,gap_actual_actor_circle_m=analyze.circle_gap(p,a,math.hypot(.225,.275))))
  rowsout.append(dict(cycle_id=i,relative_s=rel,pose=c['pose'],speed=c['speed'],command=c.get('command'),error=c.get('error'),active=active,native_safe_count=pr.get('safe_candidates'),consume_ns=pr.get('consume_ns'),source_ns=source.get('source_ns'),candidate_epoch_ns=pr.get('candidate_epoch_ns'),prediction_age_s=pr.get('age_s'),selected=selected,actual_actor_prefix=prefix,inferred_first_control_max_error=firstv_error))
 # Validate inverse display algebra against this round's genuinely recorded U.
 errors=[]
 for c in lines(CACHE/'no_shadow_retry/runs/narrow_C_83_aligned/control.jsonl'):
  if 'aggregate' not in c:continue
  inferred=inverse_sdk(c);actual=c['selected_post_sg_shift_controls'];errors.append(max(abs(a-b) for u,v in zip(inferred,actual) for a,b in zip(u,v)))
 commands=rows(OLD/'command_receipts.csv');gid_names={}
 for x in lines(OLD/'execution_endpoints.jsonl'):
  for topic,eps in x['topics'].items():
   for ep in eps:gid_names[ep['gid']]=ep['name']
 cmd=[{**x,'publisher_name':gid_names.get(x['publisher_gid'],x['publisher_gid'])} for x in commands if x['topic'] in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel') and goal<=int(x['ROS_receipt_ns'])<=event['end_ns']]
 after=next((x for x in rowsout if x['native_safe_count']==0),None)
 result=dict(source=str(OLD),events=event,cycles=rowsout,actual_gaps=gaps,first_actual_gate_violation=next((x for x in gaps if x['gap_m']<.047),None),first_actual_zero_gap=next((x for x in gaps if x['gap_m']<=1e-10),None),first_native_zero_safe=after,inverse_sdk_validation=dict(cycles=len(errors),max_control_error=max(errors),tolerance=1e-5,valid=max(errors)<1e-5),commands=cmd,inevitability='not proven; missing executable braking/retreat reachable-set contract')
 (OUT/'original_forensics.json').write_text(json.dumps(result,separators=(',',':'))+'\n');temp.unlink(missing_ok=True);print('original',len(rowsout),'first actual gap',result['first_actual_gate_violation'],'inverse error',max(errors))
if __name__=='__main__':run()
