"""Read-only original and 162-cycle command lifecycle reconstruction."""
import pathlib,json,csv,gzip,math,bisect,sys,collections
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence';CACHE=ROOT/'build/lifecycle_cache'
BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014');PREV=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014')
sys.path.insert(0,str(BASE/'experiments/stvl_c83_endtoend'));from causal_replay import lines
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows,previous_yaw_view

def json_native(x):
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
def stats(v):
 if not v:return None
 a=np.array(v);return dict(n=len(a),p50=float(np.quantile(a,.5)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),max=float(a.max()))
def vec(r,names=('vx','vy','wz')):return np.array([float(r[k]) for k in names])
def at(data,ts,t,fields):
 i=bisect.bisect_left(ts,t)
 if i==0 or i>=len(ts):return None
 z=(t-ts[i-1])/(ts[i]-ts[i-1]);return vec(data[i-1],fields)*(1-z)+vec(data[i],fields)*z
def run(p,label):
 e=json.loads((p/'events.json').read_text());goal=e['goal_ns'];end=e['end_ns'];controls=[x for x in lines(p/'control.jsonl') if goal<=x['epoch_ns']<=end];receipts=rows(p/'command_receipts.csv');native=rows(p/'native_boundary.csv');od=rows(p/'odometry.csv');ots=[int(x['source_ns']) for x in od];nts=[int(x['steady_ns']) for x in native];sim=[int(x['sim_ns']) for x in native]
 gid={}
 for x in lines(p/'execution_endpoints.jsonl'):
  for eps in x['topics'].values():
   for a in eps:gid[a['gid']]=a['name']
 for x in receipts:
  x['pub_steady_ns']=int(x['steady_ns'])+int(x['DDS_source_ns'])-int(x['system_ns']);x['pub_sim_ns']=float(np.interp(x['pub_steady_ns'],nts,sim));x['publisher_name']=gid.get(x['publisher_gid'],x['publisher_gid'])
 nav=[x for x in receipts if x['topic']=='/cmd_vel_nav' and goal<=int(x['ROS_receipt_ns'])<=end];cmd=[x for x in receipts if x['topic']=='/cmd_vel'];stub=[x for x in receipts if x['topic']=='/simulation/chassis/cmd_vel'];nvs=[x['pub_steady_ns'] for x in nav];cvts=[x['pub_steady_ns'] for x in cmd];sts=[x['pub_steady_ns'] for x in stub];used=set();cycle_rows=[]
 timed=lines(p/'paired_timing.jsonl') if (p/'paired_timing.jsonl.gz').exists() else [];timing={x['cycle_id']:x for x in timed}
 for i,c in enumerate(controls):
  cycle=c.get('cycle_id',i);match=None
  if 'command' in c:
   eligible=[(j,x) for j,x in enumerate(nav) if j not in used and c['epoch_ns']-5000000<=int(x['ROS_receipt_ns'])<=c['epoch_ns']+300000000 and max(abs(a-b) for a,b in zip(vec(x),c['command']))<1e-7]
   if eligible:j,match=eligible[0];used.add(j)
  t=c['pose_epoch_ns'];cur=at(od,ots,t,('x','y','yaw','vx','vy','wz'));pr=c.get('prediction',{});ref_steady=timing.get(cycle,{}).get('entry_steady_ns',float(np.interp(c['epoch_ns'],sim,nts)))
  oldidx=bisect.bisect_right(nvs,ref_steady)-1;old=nav[oldidx] if oldidx>=0 else None
  row=dict(cycle_id=cycle,relative_s=(c['epoch_ns']-goal)/1e9,input_pose=c['pose'],speed=c['speed'],feedback_pose_epoch_ns=t,prediction_age_s=pr.get('age_s'),dynamic_safe=pr.get('safe_candidates'),prediction_reason=pr.get('reason'),command=c.get('command'),error=c.get('error'),native_temporal_cost_min=pr.get('cost_min'),native_temporal_cost_max=pr.get('cost_max'),publication=None,held_previous_nav=None if old is None else dict(age_ms=(ref_steady-old['pub_steady_ns'])/1e6,command=vec(old).tolist(),publisher=old['publisher_name']),timing=timing.get(cycle),next_state=at(od,ots,t+100000000,('x','y','yaw','vx','vy','wz')).tolist() if at(od,ots,t+100000000,('x','y','yaw','vx','vy','wz')) is not None else None)
  if match:
   pub=match['pub_steady_ns'];next_pub=nvs[nav.index(match)+1] if nav.index(match)+1<len(nav) else None
   a=at(od,ots,match['pub_sim_ns'],('x','y','yaw','vx','vy','wz'));b=at(od,ots,float(np.interp(next_pub,nts,sim)),('x','y','yaw','vx','vy','wz')) if next_pub is not None else None
   row['publication']=dict(publisher=match['publisher_name'],pub_steady_ns=pub,pub_sim_ns=match['pub_sim_ns'],input_pose_to_publication_ms=(match['pub_sim_ns']-t)/1e6,DDS_to_cpp_receipt_ms=(int(match['system_ns'])-int(match['DDS_source_ns']))/1e6,until_next_nav_publication_ms=None if next_pub is None else (next_pub-pub)/1e6,measured_displacement_until_next_nav_m=None if a is None or b is None else float(np.linalg.norm(b[:2]-a[:2])))
   idx=bisect.bisect_left(cvts,pub);sm=cmd[idx] if idx<len(cmd) else None
   row['first_downstream_cmd']=None if sm is None else dict(command=vec(sm).tolist(),publisher=sm['publisher_name'],pub_steady_ns=sm['pub_steady_ns'],raw_to_first_cmd_ms=(sm['pub_steady_ns']-pub)/1e6)
  cycle_rows.append(row)
 # Decision provenance and transport renewal are deliberately different ages.
 hold=[]
 for j,x in enumerate(stub):
  if not goal<=x['pub_sim_ns']<=end:continue
  k=bisect.bisect_right(nvs,x['pub_steady_ns'])-1;up=nav[k] if k>=0 else None
  if up is None:continue
  nj=bisect.bisect_left(nts,x['pub_steady_ns']);nr=native[min(nj,len(native)-1)]
  hold.append(dict(sim_ns=x['pub_sim_ns'],decision_age_ms=(x['pub_steady_ns']-up['pub_steady_ns'])/1e6,transport_age_at_native_sample_ms=(int(nr['steady_ns'])-x['pub_steady_ns'])/1e6,nav_command=vec(up).tolist(),stub_command=vec(x).tolist(),native_limited_target=vec(nr,('applied_vx','applied_vy','applied_wz')).tolist(),publisher=x['publisher_name']))
 first_zero=next((x for x in cycle_rows if x['dynamic_safe']==0),None);anchors={}
 if first_zero:
  t0=goal+round(first_zero['relative_s']*1e9);v0=np.linalg.norm(first_zero['speed'][:2]);eligible=[x for x in od if int(x['source_ns'])>=t0];
  def first_sustained(pred,count):
   for i in range(len(eligible)-count+1):
    if all(pred(x) for x in eligible[i:i+count]):return eligible[i]
   return None
  slow=first_sustained(lambda x:np.linalg.norm(vec(x)[:2])<=v0-.05,3);stop=first_sustained(lambda x:np.linalg.norm(vec(x)[:2])<.02 and abs(float(x['wz']))<.02,6)
  for name,entry in [('meaningful_deceleration',slow),('body_stop',stop)]:
   if entry:
    state=vec(entry,('x','y','yaw','vx','vy','wz'));origin=at(od,ots,t0,('x','y','yaw','vx','vy','wz'));segment=[z for z in od if t0<=int(z['source_ns'])<=int(entry['source_ns'])];length=sum(float(np.linalg.norm(vec(b,('x','y'))-vec(a,('x','y')))) for a,b in zip(segment,segment[1:]));anchors[name]=dict(relative_s=(int(entry['source_ns'])-goal)/1e9,state=state.tolist(),since_zero_s=(int(entry['source_ns'])-t0)/1e9,displacement_m=float(np.linalg.norm(state[:2]-origin[:2])),path_length_m=length)
 result=dict(label=label,source=str(p),events=e,cycles=cycle_rows,decision_vs_transport=hold,anchors=anchors,matched_new_nonzero_at_zero_safe=sum(bool(x['dynamic_safe']==0 and x['publication'] and x['command'] and np.linalg.norm(x['command'])>.02) for x in cycle_rows),zero_safe_cycles=sum(x['dynamic_safe']==0 for x in cycle_rows),decision_age_ms=stats([x['decision_age_ms'] for x in hold]),limitations='native target is limited wheel command, not physical velocity; matching changed Twist and clocks, no generation timestamp in historical original')
 (OUT/f'{label}_lifecycle.json').write_text(json.dumps(result,separators=(',',':'),default=json_native)+'\n');return result
if __name__=='__main__':
 orig=run(PREV/'build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned','original_c83');dur=run(BASE/'build/c83_cache/shadow_durable/runs/narrow_C_83_aligned','durable162');over=run(BASE/'build/c83_cache/shadow/runs/narrow_C_83_aligned','overrun_history')
 print([(r['label'],len(r['cycles']),r['zero_safe_cycles'],r['matched_new_nonzero_at_zero_safe'],r['anchors']) for r in [orig,dur,over]])
