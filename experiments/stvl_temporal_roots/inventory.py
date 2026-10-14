"""Read-only first-violation/cycle evidence from the frozen complete raw runs."""
import sys,pathlib,json,math,csv,bisect,xml.etree.ElementTree as E
ROOT=pathlib.Path(__file__).resolve().parents[2];OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_consumption_20261013');RUNS=OLD/'build/stvl_cache/runs';OUT=ROOT/'experiments/stvl_temporal_roots/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));from analyze import polygon_distance,jsonlines,csvrows,stats,circle_gap
from importlib.util import spec_from_file_location,module_from_spec

def rectangle(x,y,yaw,hx,hy):
 c,s=math.cos(yaw),math.sin(yaw);return [[x+c*a-s*b,y+s*a+c*b] for a,b in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
def signed_gap(a,b):
 d=polygon_distance(a,b)
 if d>0:return d
 overlap=math.inf
 for p in (a,b):
  for u,v in zip(p,p[1:]+p[:1]):
   nx,ny=u[1]-v[1],v[0]-u[0];n=math.hypot(nx,ny)
   pa=[(x*nx+y*ny)/n for x,y in a];pb=[(x*nx+y*ny)/n for x,y in b];overlap=min(overlap,min(max(pa),max(pb))-max(min(pa),min(pb)))
 return -overlap

def walls(p):
 result=[]
 for m in E.parse(p/'scene.sdf').getroot().find('world').findall('model'):
  if not m.attrib['name'].startswith('corridor_'):continue
  x,y,*_=map(float,m.findtext('pose').split());sx,sy,*_=map(float,m.findtext('link/collision/geometry/box/size').split());result.append(rectangle(x,y,0,sx/2,sy/2))
 return result

def last(rows,t,key):
 eligible=[r for r in rows if int(r[key])<=t];return eligible[-1] if eligible else None

def inspect(name):
 p=RUNS/name;e=json.loads((p/'events.json').read_text());m=json.loads((p/'metrics.json').read_text());start=e['goal_ns'];end=e['end_ns'];ws=walls(p);truth=jsonlines(p/'truth.jsonl');robot=[r for r in truth if r['model']=='rm_sentry_2027' and start<=r['source_ns']<=end];actors={r['source_ns']:r for r in truth if r['model']=='moving_obstacle'};control=[r for r in jsonlines(p/'control.jsonl') if start<=r['epoch_ns']<=end];od=csvrows(p/'odometry.csv');cmd=csvrows(p/'commands.csv');wall_bad=[]
 for r in robot:
  gap=min(signed_gap(r['polygon'],w) for w in ws);rotation=r['rotation'];yaw=math.atan2(rotation[1][0],rotation[0][0]);nominal=rectangle(*r['position'][:2],yaw,.34,.29);ng=min(signed_gap(nominal,w) for w in ws)
  if gap<0:wall_bad.append((r,gap,ng,yaw))
 first_wall=wall_bad[0] if wall_bad else None;contact=e['contacts'][0] if e['contacts'] else None;moment=first_wall[0]['source_ns'] if first_wall else contact['source_ns'] if contact else None
 result={k:m[k] for k in ['run','scene','group','seed','variant','success','safe_success','contact_messages','wall_overlap_samples','body_clearance_m','actual_max_acceleration','actual_accel_exceed_intervals_raw']};result['original_path']=str(p);result['frozen_gate_failure']='wall_envelope' if m['success'] and m['body_clearance_m']>=.047 and m['wall_overlap_samples'] else 'actor_contact' if contact else 'other_or_success';result['acceleration_is_frozen_primary_gate']=False;result['first_contact']=contact;result['max_wall_penetration_m']=-min((v[1] for v in wall_bad),default=0)
 if first_wall:
  r,gap,ng,yaw=first_wall;result['first_wall']={'source_ns':r['source_ns'],'relative_s':(r['source_ns']-start)/1e9,'position':r['position'],'yaw':yaw,'mechanical_gap':gap,'planning_polygon_gap_at_actual_pose':ng,'polygon':r['polygon']}
 if moment:
  decision=last(control,moment,'epoch_ns');result['decision_before_first_violation']=decision
  result['feedback_before_first_violation']=last(od,moment,'source_ns');result['held_commands']={topic:last([r for r in cmd if r['topic']==topic],moment,'receipt_ns') for topic in ['/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel']}
  if decision:
   near=[r for r in cmd if r['topic']=='/cmd_vel_nav' and decision['epoch_ns']<=int(r['receipt_ns'])<=decision['epoch_ns']+100000000 and 'command' in decision and max(abs(float(r[k])-v) for k,v in zip(('vx','vy','wz'),decision['command']))<1e-5];result['decision_output_receipt_ns']=int(near[0]['receipt_ns']) if near else None;result['decision_was_returned_before_violation']=bool(near and int(near[0]['receipt_ns'])<=moment)
  window=[]
  for c in control:
   if moment-1400000000<=c['epoch_ns']<=moment+200000000:
    ag=c.get('aggregate',[]);ng=[min(signed_gap(rectangle(*pose,.34,.29),w) for w in ws) for pose in ag];phys=[min(signed_gap(rectangle(*pose,.325,.300),w) for w in ws) for pose in ag];pr=c.get('prediction',{});epoch=c.get('pose_epoch_ns',c['epoch_ns']);future=epoch+100000000;following=min(robot,key=lambda r:abs(r['source_ns']-future)) if robot else None
    row={'relative_s':(c['epoch_ns']-start)/1e9,'epoch_ns':c['epoch_ns'],'pose_epoch_ns':c.get('pose_epoch_ns'),'pose':c['pose'],'speed':c['speed'],'command':c.get('command'),'error':c.get('error'),'compute_ms':c['compute_ms'],'planning_aggregate_wall_min':min(ng) if ng else None,'mechanical_aggregate_wall_min':min(phys) if phys else None,'prediction':pr,'first_aggregate_pose':ag[0] if ag else None,'actual_at_pose_plus_01':{'source_ns':following['source_ns'],'position':following['position'],'yaw':math.atan2(following['rotation'][1][0],following['rotation'][0][0])} if following else None}
    if following and ag:row['next_position_error_m']=math.dist(ag[0][:2],following['position'][:2]);row['next_yaw_error_rad']=math.atan2(math.sin(ag[0][2]-row['actual_at_pose_plus_01']['yaw']),math.cos(ag[0][2]-row['actual_at_pose_plus_01']['yaw']))
    if 'critic_deltas' in c:row['critic_ranges']={k:stats(v) for k,v in c['critic_deltas'].items()};row['raw_planning_wall_hit_candidates']=sum(any(min(signed_gap(rectangle(*pose,.34,.29),w) for w in ws)<0 for pose in traj) for traj in c.get('raw',[]))
    window.append(row)
  result['window']=window
 # Initial/delayed availability is part of the oracle input, not future accuracy.
 result['active_track_history']=[{'relative_s':(c['epoch_ns']-start)/1e9,'active':sum(t['state'] in (2,3) for t in c.get('prediction',{}).get('source',{}).get('tracks',[])),'reason':c.get('prediction',{}).get('reason','not_scored'),'safe_candidates':c.get('prediction',{}).get('safe_candidates'),'cost_min':c.get('prediction',{}).get('cost_min'),'command':c.get('command')} for c in control]
 # Avoid duplicating the 300x30 raw in this concise first-cycle packet.
 if result.get('decision_before_first_violation'):
  result['decision_before_first_violation']={k:v for k,v in result['decision_before_first_violation'].items() if k not in ('raw','critic_deltas')}
 return result

def main():
 names=['wait_release_C_29_aligned','stale_C_29_aligned','narrow_D_29_aligned','narrow_D_47_aligned','narrow_D_29_gap','narrow_D_13_aligned','wait_release_B_29_aligned','stale_B_29_aligned','narrow_C_29_aligned','wait_release_C_13_aligned']
 rows=[inspect(n) for n in names];OUT.mkdir(exist_ok=True);(OUT/'original_inventory.json').write_text(json.dumps({'base':'d2b5f0714585e1e71abaf20def932937057f644e','primary_gate':'unchanged contact/gap>=.047/no wall-envelope overlap/goal; acceleration is a separate observed model contract issue','cases':rows},indent=2)+'\n')
 for r in rows:print(r['run'],r['frozen_gate_failure'],'first-wall',r.get('first_wall',{}).get('relative_s'),'depth',r['max_wall_penetration_m'],'nominalgap',r.get('first_wall',{}).get('planning_polygon_gap_at_actual_pose'),'contact',r.get('first_contact',{}).get('source_ns') if r.get('first_contact') else None)
if __name__=='__main__':main()
