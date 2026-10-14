"""Offline same-input causal checking; never outputs commands or edits old raw."""
import pathlib,json,gzip,struct,sys,subprocess,copy,bisect,math,os
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/c83_cache';OUT=ROOT/'experiments/stvl_c83_endtoend/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_execution_realtime'));from mapping import previous_yaw_view

def lines(p):
 q=p if p.exists() else pathlib.Path(str(p)+'.gz');op=gzip.open if q.suffix=='.gz' else open
 with op(q,'rt') as f:return [json.loads(x) for x in f]
BINARY_EXCLUSIONS={}
def binary(p):
 q=p if p.exists() else pathlib.Path(str(p)+'.gz');op=gzip.open if q.suffix=='.gz' else open
 with op(q,'rb') as f:
  while True:
   h=f.read(16)
   if not h:break
   if len(h)!=16:BINARY_EXCLUSIONS[str(p)]=dict(reason='shutdown_partial_header',bytes=len(h));break
   cycle,count=struct.unpack('<QQ',h);assert count<=600;buf=f.read(count*30*48)
   if len(buf)!=count*30*48:BINARY_EXCLUSIONS[str(p)]=dict(reason='shutdown_partial_final_candidate_record',cycle_id=cycle,expected_bytes=count*30*48,preserved_bytes=len(buf));break
   yield cycle,np.frombuffer(buf,dtype='<f8').reshape(count,30,2,3)
def actor_at(actor,ts,t):
 i=bisect.bisect_left(ts,t)
 if i<1 or i>=len(ts):return None
 a,b=actor[i-1],actor[i];alpha=(t-ts[i-1])/(ts[i]-ts[i-1]);return [a['position'][q]*(1-alpha)+b['position'][q]*alpha for q in (0,1)]
def run(p):
 records=lines(p/'control.jsonl');lookup={x['cycle_id']:x for x in records};event=json.loads((p/'events.json').read_text());goal=event['goal_ns'];scene=event['scene'];static=json.loads((p.parent.parent/'assets'/f'{scene}_static.json').read_text())
 truth=lines(p/'truth.jsonl');actor=[x for x in truth if x['model']=='moving_obstacle'];ts=[x['source_ns'] for x in actor]
 fp=[[-.34,-.29],[.34,-.29],[.34,.29],[-.34,.29]] # native configured padding; validated below
 import yaml
 cfg=yaml.safe_load((p/'nav2.yaml').read_text());cm=cfg['local_costmap']['local_costmap']['ros__parameters'];foot=__import__('ast').literal_eval(cm['footprint']);pad=cm.get('footprint_padding',.01)
 fp=[[x+math.copysign(pad,x),y+math.copysign(pad,y)] for x,y in foot]
 temp=CACHE/'offline_input.json';answer=CACHE/'offline_result.json';result=[];excluded=[]
 for cycle,raw in binary(p/'paired_timing.jsonl.raw'):
  if cycle not in lookup:excluded.append(dict(cycle_id=cycle,reason='no_complete_control_json'));continue
  c=lookup[cycle]
  if not goal<=c['epoch_ns']<=event['end_ns']:excluded.append(dict(cycle_id=cycle,reason='outside_navigation_epoch'));continue
  pr=c.get('prediction',{});source=pr.get('source',{});active=any(t['state'] in (2,3) for t in source.get('tracks',[]));rel=(c['epoch_ns']-goal)/1e9
  # Early empty/tentative cycles still diagnose selected motion. Active and native
  # failure cycles diagnose every raw; no post-outcome cherry-picking within A6.
  raw_needed=active or 'error' in c
  ids=range(len(raw)) if raw_needed else []
  trajectories=[dict(id=f'raw_{i}',poses=raw[i,:,0,:].tolist(),velocities=raw[i,:,1,:].tolist()) for i in ids]
  if 'command' in c:trajectories.append(dict(id='selected_previous_yaw',poses=previous_yaw_view(c['pose'],c['selected_post_sg_shift_controls']),velocities=c['selected_post_sg_shift_controls']))
  if not trajectories:continue
  ctx=dict(case=f'{p.parent.parent.name}__{cycle}',pose=c['pose'],speed=c['speed'],pose_epoch_ns=c['pose_epoch_ns'],dt=.1,identity_frame_fixture=True,execution_mapping_known=False,feedback_consistent=True,footprint=fp,prediction=pr,static_map=static,trajectories=trajectories)
  def execute(x):
   temp.write_text(json.dumps(x,separators=(',',':'))+'\n')
   if os.environ.get('C83_REPLAY_DIRECT')=='1':
    args=[str(CACHE/'plugin_build/replay_cycle'),str(temp)]
   else:raise RuntimeError('run inside supplied isolated replay container; no per-cycle docker startup')
   with answer.open('w') as f:subprocess.run(args,check=True,stdout=f)
   return json.loads(answer.read_text())
  cv=execute(ctx);native_costs=c.get('native_final_costs_after_gamma',[]);safe=[int(x['id'][4:]) for x in cv if x['id'].startswith('raw_') and x['joint_geometry']=='Safe'];selected=next((x for x in cv if x['id']=='selected_previous_yaw'),None)
  # Complete ground truth from THIS execution, sampled at unchanged source clock.
  # Preserve identity/state/size/consume/pose/U; only source points change, as in D.
  oracle_ctx=copy.deepcopy(ctx);covered=active
  if covered:
   for t in oracle_ctx['prediction']['source']['tracks']:
    pts=[actor_at(actor,ts,source['source_ns']+k*100000000) for k in range(36)]
    if any(x is None for x in pts):covered=False;break
    t['points']=pts
  oracle=execute(oracle_ctx) if covered else None
  if native_costs:
   best=int(np.argmin(native_costs));best_safe=min(safe,key=lambda i:native_costs[i]) if safe else None
  else:best=best_safe=None
  chosen_ids=([best] if best is not None else [])+([best_safe] if best_safe is not None else [])
  item=dict(cycle_id=cycle,relative_s=rel,active=active,failed='error' in c,raw_diagnosed=len(list(ids)),native_safe_count=pr.get('safe_candidates'),joint_counts={x:sum(q['joint_geometry']==x for q in cv if q['id'].startswith('raw_')) for x in ('Safe','Unknown','Unsafe')},selected=selected,best_native_id=best,best_safe_id=best_safe,best_native_check=next((x for x in cv if x['id']==f'raw_{best}'),None),native_best_cost=None if best is None else native_costs[best],native_best_safe_cost=None if best_safe is None else native_costs[best_safe],same_input_oracle_covered=covered,oracle_counts=None if oracle is None else {x:sum(q['joint_geometry']==x for q in oracle if q['id'].startswith('raw_')) for x in ('Safe','Unknown','Unsafe')},oracle_selected=None if oracle is None else next((x for x in oracle if x['id']=='selected_previous_yaw'),None),example_raw={str(i):raw[i].tolist() for i in chosen_ids})
  result.append(item)
  print(p.parent.parent.name,cycle,len(result),flush=True)
 (OUT/f'{p.parent.parent.name}_stream_exclusions.json').write_text(json.dumps(dict(binary=BINARY_EXCLUSIONS,excluded=excluded),indent=2)+'\n')
 (OUT/f'{p.parent.parent.name}_causal.json').write_text(json.dumps(result,separators=(',',':'))+'\n');temp.unlink(missing_ok=True);answer.unlink(missing_ok=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
