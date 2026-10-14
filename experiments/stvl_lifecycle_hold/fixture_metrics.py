"""Physical actuator-chain metrics, not a controller or hardware safety claim."""
import pathlib,json,csv,gzip,sys,math,bisect,xml.etree.ElementTree as ET
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/lifecycle_cache';OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence';BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows
sys.path.insert(0,str(BASE/'experiments/stvl_temporal'));from analyze import polygon_distance

def jl(p):
 q=p if p.exists() else pathlib.Path(str(p)+'.gz');op=gzip.open if q.suffix=='.gz' else open
 return [json.loads(x) for x in op(q,'rt')]
def interp(a,t,key='source_ns',fields=('x','y','yaw','vx','vy','wz')):
 ts=[int(x[key]) for x in a];j=bisect.bisect_left(ts,t)
 if j<1 or j>=len(a):return None
 z=(t-ts[j-1])/(ts[j]-ts[j-1]);return np.array([float(a[j-1][k])*(1-z)+float(a[j][k])*z for k in fields])
def run(p):
 e=json.loads((p/'events.json').read_text());origin=e['origin_ns'];data=rows(p/'observations.csv');od=[x for x in data if x['topic']=='/physical_odom'];inputs=jl(p/'inputs.jsonl');native=rows(p/'native.csv') if (p/'native.csv').exists() or (p/'native.csv.gz').exists() else [];results=[]
 if e['kind']=='timing':cases=sorted({x['case'] for x in inputs if x.get('ready_sim_ns',0)>=origin})
 else:cases=[e['kind']]
 for name in cases:
  events=[x for x in inputs if x['case']==name];new=[x for x in events if x['event']=='new_output'];first=next((x for x in new if x['logical_s']>=0),None)
  if first is None:continue
  segment=math.floor(first['logical_s']/6)*6 if e['kind']=='timing' else 0;stop_s=segment+2 if e['kind']=='timing' else e.get('brake_trigger_s') or 2.522493
  start=origin+round(stop_s*1e9);a=interp(od,start);post=[x for x in od if start<=int(x['source_ns'])<origin+round((segment+3.7 if e['kind']=='timing' else 8.9)*1e9)]
  slow=None;stop=None
  for i,x in enumerate(post):
   vel=np.hypot(float(x['vx']),float(x['vy']))
   if slow is None and a is not None and vel<=np.linalg.norm(a[3:5])-.05:slow=x
   if stop is None and i+5<len(post) and all(math.hypot(float(z['vx']),float(z['vy']))<.02 and abs(float(z['wz']))<.02 for z in post[i:i+6]):stop=x
  zero=[x for x in new if x['publish_sim_ns']>=start and max(abs(v) for v in x['velocity'])<1e-10];z=zero[0] if zero else None
  sm=[x for x in data if x['topic']=='/cmd_vel' and int(x['ROS_ns'])>=start];drop=next((x for x in sm if float(x['vx'])<float(a[3])-.001),None) if a is not None else None
  point=None if stop is None else interp(od,int(stop['source_ns']));r=dict(case=name,stop_logical_s=stop_s,state_at_stop_request=None if a is None else a.tolist(),first_zero_publication_s=None if z is None else (z['publish_sim_ns']-origin)/1e9,injected_output_delays_ms=[x['actual_release_to_publish_ms'] for x in new if x['delay_s']>0],missing_output_cycles=[x['cycle_id'] for x in events if x['event']=='no_new_output'],first_smoother_deceleration_s=None if drop is None else (int(drop['ROS_ns'])-origin)/1e9,first_body_deceleration_s=None if slow is None else (int(slow['source_ns'])-origin)/1e9,body_stop_s=None if stop is None else (int(stop['source_ns'])-origin)/1e9,displacement_request_to_stop_m=None if a is None or point is None else float(np.linalg.norm(point[:2]-a[:2])),stopped_state=None if point is None else point.tolist(),feedback_observer_age_ms=max((x.get('feedback_age_ms') or 0 for x in new),default=None),outcomes_scope='no online MPPI; missing/late frozen upstream signals')
  results.append(r)
 bytime={}
 for x in jl(p/'truth.jsonl'):bytime.setdefault(x['source_ns'],{})[x['model']]=x
 gaps=[dict(source_ns=t,relative_s=(t-origin)/1e9,gap_m=polygon_distance(x['rm_sentry_2027']['polygon'],x['moving_obstacle']['polygon'])) for t,x in sorted(bytime.items()) if t>=origin and len(x)==2]
 prefix=None
 if e['kind']!='timing':
  old=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned');oe=json.loads((old/'events.json').read_text());oldod=rows(old/'odometry.csv');oldstate=interp(oldod,oe['goal_ns']+round(e['brake_trigger_s']*1e9),fields=('x','y','yaw','vx','vy','wz'));current=interp(od,origin+round(e['brake_trigger_s']*1e9));
  if oldstate is not None and current is not None:
   dx=float(np.linalg.norm(current[:2]-oldstate[:2]));dy=abs(math.atan2(math.sin(current[2]-oldstate[2]),math.cos(current[2]-oldstate[2])));dv=float(max(abs(current[3:]-oldstate[3:])));prefix=dict(old=oldstate.tolist(),fixture=current.tolist(),position_error_m=dx,yaw_error_rad=dy,velocity_component_error=dv,within_frozen_tolerance=dx<=.05 and dy<=.06 and dv<=.12)
   ot=jl(old/'truth.jsonl');actold=[x for x in ot if x['model']=='moving_obstacle'];actnew=[x for x in jl(p/'truth.jsonl') if x['model']=='moving_obstacle'];pe=[];ye=[];ve=[];ae=[]
   def actor_at(a,t):
    ts=[x['source_ns'] for x in a];i=bisect.bisect_left(ts,t)
    if i<1 or i>=len(a):return None
    z=(t-ts[i-1])/(ts[i]-ts[i-1]);return np.array(a[i-1]['position'][:2])*(1-z)+np.array(a[i]['position'][:2])*z
   for relative in np.arange(.1,e['brake_trigger_s'],.1):
    oldt=oe['goal_ns']+round(relative*1e9);newt=origin+round(relative*1e9);a0=interp(oldod,oldt);b0=interp(od,newt)
    if a0 is not None and b0 is not None:pe.append(float(np.linalg.norm(a0[:2]-b0[:2])));ye.append(abs(math.atan2(math.sin(a0[2]-b0[2]),math.cos(a0[2]-b0[2]))));ve.append(float(max(abs(a0[3:]-b0[3:]))))
    ac=actor_at(actold,oldt);bc=actor_at(actnew,newt)
    if ac is not None and bc is not None:ae.append(float(np.linalg.norm(ac-bc)))
   prefix.update(prefix_samples=len(pe),max_prefix_position_error_m=max(pe),max_prefix_yaw_error_rad=max(ye),max_prefix_velocity_error=max(ve),max_actor_prefix_error_m=max(ae),prefix_all_within_frozen_tolerance=max(pe)<=.05 and max(ye)<=.06 and max(ve)<=.12 and max(ae)<=.03)
 # Wheel target changes retain original sim-step limiter; actual body acceleration
 # remains physical and should not be confused with the smoother increment.
 nt=[int(x['sim_ns']) for x in native];max_target_acc=[0.,0.,0.];valid=0;parse_bad=0
 for a,b in zip(native,native[1:]):
  try:
   dt=(int(b['sim_ns'])-int(a['sim_ns']))/1e9
   if dt<=0:continue
   for j,k in enumerate(('applied_vx','applied_vy','applied_wz')):max_target_acc[j]=max(max_target_acc[j],abs(float(b[k])-float(a[k]))/dt)
   valid+=1
  except (ValueError,TypeError):parse_bad+=1
 walls=[]
 for model in ET.parse(p/'scene.sdf').getroot().find('world').findall('model'):
  if model.attrib['name'].startswith('corridor_'):
   x,y,*_=map(float,model.findtext('pose').split());sx,sy,*_=map(float,model.findtext('link/collision/geometry/box/size').split());walls.append([[x-sx/2,y-sy/2],[x+sx/2,y-sy/2],[x+sx/2,y+sy/2],[x-sx/2,y+sy/2]])
 wallrecords=[dict(relative_s=(t-origin)/1e9,gap_m=min(polygon_distance(x['rm_sentry_2027']['polygon'],wall) for wall in walls)) for t,x in sorted(bytime.items()) if t>=origin and walls and 'rm_sentry_2027' in x]
 wallgaps=[min(polygon_distance(x['rm_sentry_2027']['polygon'],wall) for wall in walls) for t,x in bytime.items() if t>=origin and walls and 'rm_sentry_2027' in x]
 event_summary=dict(e);event_summary['contacts']=e['contacts'][:1];event_summary['contact_messages_total']=len(e['contacts'])
 result=dict(first_wall_overlap=next((x for x in wallrecords if x['gap_m']<=1e-10),None),wall_min_gap_m=min(wallgaps) if wallgaps else None,wall_overlap_samples=sum(x<=1e-10 for x in wallgaps),label=p.name,events=event_summary,profiles=results,prefix_comparison=prefix,dynamic_min_gap_m=min((x['gap_m'] for x in gaps),default=None),bbox_overlap_samples=sum(x['gap_m']<=1e-10 for x in gaps),contact_count=len(e['contacts']),gaps=gaps,native_max_target_acceleration=max_target_acc if native else None,native_valid_intervals=valid if native else None,native_parse_bad=parse_bad if native else None,native_scope='full' if native else 'not included in selected archive')
 (OUT/f'{p.name}_metrics.json').write_text(json.dumps(result,separators=(',',':'))+'\n');return result
if __name__=='__main__':
 summaries=[]
 for p in sorted(CACHE.iterdir()):
  if p.is_dir() and (p/'events.json').exists():
   x=run(p);summaries.append(x);print(p.name,x['contact_count'],x['dynamic_min_gap_m'],x['prefix_comparison'],x['profiles'])
 (OUT/'fixture_summary.json').write_text(json.dumps([{k:v for k,v in x.items() if k!='gaps'} for x in summaries],indent=2)+'\n')
