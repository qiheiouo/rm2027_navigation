"""All started runs, sampled mechanical oracle, paired data and score attribution."""
import csv,json,math,pathlib,statistics,sys,bisect,gzip,shutil,re,os,xml.etree.ElementTree as ET
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=pathlib.Path(os.environ.get('STVL_EVIDENCE_CACHE',str(ROOT/'build/stvl_cache')));EVIDENCE=ROOT/'experiments/stvl_temporal/evidence'
FROZEN=pathlib.Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/experiments/temporal_mpc')
sys.path.insert(0,str(FROZEN))
from temporal_mpc.oracle import polygon_distance

def quantile(v,q):
 if not v:return None
 v=sorted(v);x=(len(v)-1)*q;i=int(x);return v[i]+(x-i)*(v[min(i+1,len(v)-1)]-v[i])
def stats(v):return {'n':len(v),'min':min(v),'p50':quantile(v,.5),'p95':quantile(v,.95),'p99':quantile(v,.99),'max':max(v)} if v else None
def valid_json(line):
 try:json.loads(line);return True
 except json.JSONDecodeError:return False
def jsonlines(p):
 result=[]
 if p.exists():
  for line in p.open():
   try:result.append(json.loads(line))
   except json.JSONDecodeError:pass # incomplete final bytes are explicitly counted below
 return result
def csvrows(p):return list(csv.DictReader(p.open())) if p.exists() else []
def timed_interpolation(t,xy,query):
 if not t or query<t[0] or query>t[-1]:return None
 i=max(1,min(len(t)-1,bisect.bisect_left(t,query)));a=i-1;alpha=(query-t[a])/(t[i]-t[a]);return [xy[a][k]+alpha*(xy[i][k]-xy[a][k]) for k in (0,1)]
def circle_gap(pose,point,radius):
 dx,dy=point[0]-pose[0],point[1]-pose[1];c,s=math.cos(pose[2]),math.sin(pose[2]);qx=abs(c*dx+s*dy)-.34;qy=abs(-s*dx+c*dy)-.29
 return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-radius
def prediction_at(payload,track,consume_ns,k):
 age=(consume_ns-payload['source_ns'])/1e9;index=(age+(k+1)*.1)/.1
 if not 0<=index<=35:return None
 a=min(35,int(index));b=min(35,a+1);alpha=index-a;return [track['points'][a][j]+alpha*(track['points'][b][j]-track['points'][a][j]) for j in (0,1)]

def trial(path):
 event=json.loads((path/'events.json').read_text());manifest=json.loads((path/'manifest.json').read_text());start=event['goal_ns'];end=event['end_ns'];mode=manifest['mode'];seed=manifest['repeat']
 row={'run':path.name,'scene':manifest['scene'],'group':mode,'seed':seed,'phase':manifest['phase'],'startup':start is None,'outcome':event['finished'],'result_status':event['result_status'],'contact_messages':len(event['contacts']),'native_events':len(event['native_mppi_errors']),'fatal_reason':event['controller_failure']}
 row['variant']=manifest.get('variant','consume');row['validation_split']='holdout' if seed==47 else 'repeat'
 scenario=json.loads((path/'scenario.json').read_text());corridor=manifest['scene'] in ('narrow','wait_release','stale')
 row['fixture_revision']=scenario.get('corridor_revision',1) if corridor else 0
 row['cohort']='fixture_v1_blocked' if corridor and row['fixture_revision']==1 else 'final_fixture'
 row['incomplete_jsonl_lines']={p.name:sum(1 for line in p.open() if not valid_json(line)) for p in path.glob('*.jsonl')}
 if start is None:return row
 row['elapsed_s']=(end-start)/1e9;row['success']=event['result_status']==4 and not event['contacts'] and not event['controller_failure'];row['arrival_s']=row['elapsed_s'] if row['success'] else None
 truth=jsonlines(path/'truth.jsonl');bytime={}
 for r in truth:
  if start<=r['source_ns']<=end:bytime.setdefault(r['source_ns'],{})[r['model']]=r
 robot=[(ns,m['rm_sentry_2027']) for ns,m in sorted(bytime.items()) if 'rm_sentry_2027' in m]
 common=[(ns,m) for ns,m in sorted(bytime.items()) if 'rm_sentry_2027' in m and 'moving_obstacle' in m]
 distances=[polygon_distance(m['rm_sentry_2027']['polygon'],m['moving_obstacle']['polygon']) for ns,m in common]
 row['body_clearance_m']=min(distances) if distances else None;row['overlap_samples']=sum(d<=1e-10 for d in distances);row['truth_samples']=len(common);row['truth_gap_ms']=stats([(b[0]-a[0])/1e6 for a,b in zip(common,common[1:])])
 walls=[]
 for model in ET.parse(path/'scene.sdf').getroot().find('world').findall('model'):
  if not model.attrib['name'].startswith('corridor_'):continue
  x,y,*_=map(float,model.findtext('pose').split());sx,sy,*_=map(float,model.findtext('link/collision/geometry/box/size').split())
  walls.append([[x-sx/2,y-sy/2],[x+sx/2,y-sy/2],[x+sx/2,y+sy/2],[x-sx/2,y+sy/2]])
 wd=[min(polygon_distance(r['polygon'],w) for w in walls) for _,r in robot] if walls else []
 row['wall_clearance_m']=min(wd) if wd else None;row['wall_overlap_samples']=sum(d<=1e-10 for d in wd)
 row['safe_success']=row['success'] and (not distances or min(distances)>=.047) and not row['wall_overlap_samples']
 od=[r for r in csvrows(path/'odometry.csv') if start<=int(r['source_ns'])<=end];wait=stall=current_wait=0.;starts=0;previous_moving=False;sign=None;flips=0;peak=-math.inf;retreat=0;length=0.;maxspeed=[0.,0.,0.];maxacc=[0.,0.,0.];acc_exceed=[0,0,0];vel_exceed=[0,0,0]
 for i,r in enumerate(od):
  velocity=[float(r[k]) for k in ('vx','vy','wz')];moving=math.hypot(*velocity[:2])>=.02
  if moving and not previous_moving:starts+=1
  previous_moving=moving
  if i+1<len(od):
   dt=(int(od[i+1]['source_ns'])-int(r['source_ns']))/1e9
   if 0<dt<=.1:
    if not moving:wait+=dt;current_wait+=dt;stall=max(stall,current_wait)
    else:current_wait=0
    for j,k in enumerate(('vx','vy','wz')):
     acceleration=abs(float(od[i+1][k])-velocity[j])/dt;maxacc[j]=max(maxacc[j],acceleration);acc_exceed[j]+=acceleration>(.8,.8,2.)[j]
    length+=math.hypot(float(od[i+1]['x'])-float(r['x']),float(od[i+1]['y'])-float(r['y']))
  for j,v in enumerate(velocity):maxspeed[j]=max(maxspeed[j],abs(v));vel_exceed[j]+=abs(v)>(.5,.5,1.2)[j] or (j==0 and v<-.3)
  forward=math.cos(float(r['yaw']))*velocity[0]-math.sin(float(r['yaw']))*velocity[1];newsign=1 if forward>.02 else -1 if forward<-.02 else None
  if newsign is not None:
   if sign is not None and sign!=newsign:flips+=1
   sign=newsign
  peak=max(peak,float(r['x']));retreat=max(retreat,peak-float(r['x']))
 row.update(wait_s=wait,longest_stall_s=stall,start_count=starts,forward_flips=flips,max_retreat_m=retreat,path_length_m=length,actual_max_velocity=maxspeed,actual_max_acceleration=maxacc)
 row['actual_accel_exceed_intervals_raw']=acc_exceed;row['actual_velocity_exceed_samples_raw']=vel_exceed;row['physical_exceedance_basis']='raw finite differences vs smoother settings, no calibration/tolerance; not new post-hoc primary safety gate'
 controls=[r for r in jsonlines(path/'control.jsonl') if start<=r['epoch_ns']<=end]
 row['cycles']=len(controls);row['compute_ms']=stats([r['compute_ms'] for r in controls]);row['compute_over100']=sum(r['compute_ms']>100 for r in controls);row['native_errors']=sum('error' in r for r in controls)
 row['candidate_epoch_to_consume_ms']=stats([(r['prediction']['consume_ns']-r['epoch_ns'])/1e6 for r in controls if 'consume_ns' in r.get('prediction',{})])
 row['input_pose_to_entry_ms']=stats([(r['epoch_ns']-r['pose_epoch_ns'])/1e6 for r in controls if r.get('pose_epoch_ns',0)>0])
 row['epoch_basis_counts']={basis:sum(r.get('prediction',{}).get('epoch_basis')==basis for r in controls) for basis in ('consume','input_pose')}
 row['prediction_receipt_to_consume_ms']=stats([(r['prediction']['consume_ns']-r['prediction']['receipt_ns'])/1e6 for r in controls if r.get('prediction',{}).get('receipt_ns',0)>0])
 row['prediction_age_ms']=stats([r['prediction']['age_s']*1000 for r in controls if 'age_s' in r.get('prediction',{})]);row['prediction_reason_counts']={}
 for r in controls:
  reason=r.get('prediction',{}).get('reason','not_scored' if mode in ('B','C','D') else 'not_enabled');row['prediction_reason_counts'][reason]=row['prediction_reason_counts'].get(reason,0)+1
 row['temporal_scored_cycles']=sum('consume_ns' in r.get('prediction',{}) for r in controls);row['temporal_valid_cycles']=sum('age_s' in r.get('prediction',{}) for r in controls)
 row['consumed_nonzero']=sum(r.get('prediction',{}).get('cost_max',0)>0 for r in controls);row['zero_safe_candidates']=sum(r.get('prediction',{}).get('safe_candidates')==0 for r in controls)
 packets=[r for r in csvrows(path/'commands.csv') if r['topic']=='/simulation/chassis/cmd_vel' and start<=int(r['receipt_ns'])<=end];stamps=[int(r['receipt_ns']) for r in packets]
 row['output_gap_ms']=stats([(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]);row['output_gap_over100']=sum(b-a>100_000_000 for a,b in zip(stamps,stamps[1:]))
 zero_hold=[];active_gaps=[]
 for a,b in zip(packets,packets[1:]):
  gap=(int(b['receipt_ns'])-int(a['receipt_ns']))/1e6
  (zero_hold if all(abs(float(a[k]))<1e-5 for k in ('vx','vy','wz')) else active_gaps).append(gap)
 row['zero_hold_output_gap_ms']=stats(zero_hold);row['active_output_gap_ms']=stats(active_gaps)
 row['output_max_command']=[max((abs(float(r[k])) for r in packets),default=0) for k in ('vx','vy','wz')]
 row['output_bounds_violations']=sum(float(r['vx'])<-.30001 or float(r['vx'])>.50001 or abs(float(r['vy']))>.50001 or abs(float(r['wz']))>1.20001 for r in packets)
 cm=[r for r in csvrows(path/'costmap.csv') if start<=int(r['source_ns'])<=end];row['observed_current_occupied_samples']=sum(int(r['actor_window_lethal'])>0 for r in cm if r['actor_window_lethal'])
 endpoint=(path/'output_endpoints.txt').read_text() if (path/'output_endpoints.txt').exists() else '';count=re.search(r'Publisher count: (\d+)',endpoint);row['output_publisher_count']=int(count[1]) if count else None;row['output_single_owner']=row['output_publisher_count']==1;row['output_owner_named']='chassis_interface_stub' in endpoint
 row['shutdown_minus11']='exit code -11' in (path/'launch.log').read_text()
 snapshots=[];optimistic=0;aggregate_unsafe=0;forecast_errors={str(t):[] for t in (0,1,3)};track_counts=[]
 actor=[((ns-start)/1e9,m['moving_obstacle']['position'][:2]) for ns,m in sorted(bytime.items()) if 'moving_obstacle' in m];actor_t=[r[0] for r in actor];actor_xy=[r[1] for r in actor]
 oracle_file=CACHE/'assets'/f'oracle_{manifest["scene"]}_{seed}.json';oracle=json.loads(oracle_file.read_text()) if oracle_file.exists() else None
 oracle_errors=[]
 if oracle:
  for t,xy in actor:
   p=timed_interpolation(oracle['t'],oracle['xy'],t)
   if p:oracle_errors.append(math.dist(p,xy))
 row['oracle_replay_alignment_m']=stats(oracle_errors)
 for r in controls:
  payload=r.get('prediction',{}).get('source',{});active=[t for t in payload.get('tracks',[]) if t['state'] in (2,3)];aggregate=r.get('aggregate',[]);predicted=math.inf;actual=math.inf;covered=0
  track_counts.append(len(active));trajectory_epoch=r.get('pose_epoch_ns',r['epoch_ns']) or r['epoch_ns'];query_epoch=r.get('prediction',{}).get('candidate_epoch_ns',r.get('prediction',{}).get('consume_ns',r['epoch_ns']))
  for future in (0,1,3):
   center=timed_interpolation(actor_t,actor_xy,(payload.get('source_ns',0)-start)/1e9+future)
   if center:
    for track in active:forecast_errors[str(future)].append(math.dist(track['points'][future*10],center))
  for k,pose in enumerate(aggregate):
   for track in active:
    p=prediction_at(payload,track,query_epoch,k)
    if p:predicted=min(predicted,circle_gap(pose,p,max(.36,math.hypot(*track['size'])/2)))
   actual_center=timed_interpolation(actor_t,actor_xy,(trajectory_epoch-start)/1e9+(k+1)*.1)
   if actual_center:actual=min(actual,circle_gap(pose,actual_center,math.hypot(.225,.275)));covered+=1
  if covered==30 and actual<=.02:aggregate_unsafe+=1;optimistic+=predicted>.02
  if 'critic_deltas' in r:
   parts=r['critic_deltas'];total=[sum(v[i] for v in parts.values()) for i in range(300)] if parts else []
   best=min(range(len(total)),key=total.__getitem__) if total else None
   snapshots.append({'epoch_ns':r['epoch_ns'],'t':(r['epoch_ns']-start)/1e9,'predicted_aggregate_circle_clearance':None if predicted==math.inf else predicted,'actual_aggregate_circle_proxy_clearance':None if actual==math.inf else actual,'GT_full30':covered==30,'best_raw_index_before_gamma':best,'best_raw_cost_parts':{k:v[best] for k,v in parts.items()} if best is not None else {},'cost_part_ranges':{k:stats(v) for k,v in parts.items()},'command':r.get('command'),'prediction_reason':r.get('prediction',{}).get('reason')})
 row['aggregate_unsafe_circle_proxy_cycles']=aggregate_unsafe;row['optimistic_circle_proxy_cycles']=optimistic;row['snapshots']=snapshots;row['active_track_counts']=stats(track_counts);row['forecast_center_error_m']={k:stats(v) for k,v in forecast_errors.items()}
 (path/'metrics.json').write_text(json.dumps(row,indent=2)+'\n');return row

def main():
 rows=[trial(p) for p in sorted((CACHE/'runs').iterdir()) if (p/'events.json').exists()];finite=[r for r in rows if r['phase']=='finite' and not r['startup'] and r['cohort']=='final_fixture']
 groups=[]
 for variant,split,scene in sorted({(r['variant'],r['validation_split'],r['scene']) for r in finite}):
  for mode in ('A','B','C','D'):
   r=[v for v in finite if v['scene']==scene and v['group']==mode and v['variant']==variant and v['validation_split']==split]
   if not r:continue
   groups.append({'variant':variant,'split':split,'scene':scene,'group':mode,'n':len(r),'success':sum(v['success'] for v in r),'safe_success':sum(v['safe_success'] for v in r),'contact_trials':sum(v['contact_messages']>0 for v in r),'arrival_s':stats([v['arrival_s'] for v in r if v['arrival_s'] is not None]),'clearance_m':stats([v['body_clearance_m'] for v in r if v['body_clearance_m'] is not None]),'wait_s':stats([v['wait_s'] for v in r]),'native_errors':sum(v['native_errors'] for v in r),'output_gap_over100':sum(v['output_gap_over100'] for v in r),'compute_over100':sum(v['compute_over100'] for v in r)})
 pairs=[]
 for variant,scene in sorted({(r['variant'],r['scene']) for r in finite}):
  for seed in (13,29,47):
   by={r['group']:r for r in finite if r['scene']==scene and r['seed']==seed and r['variant']==variant}
   for first,second in (('A','C'),('B','C'),('C','D')):
    if first not in by or second not in by:continue
    a,b=by[first],by[second];pairs.append({'variant':variant,'scene':scene,'seed':seed,'comparison':first+'->'+second,'first_outcome':a['outcome'],'second_outcome':b['outcome'],'safe_success_change':int(b['safe_success'])-int(a['safe_success']),'time_change_fraction':b['arrival_s']/a['arrival_s']-1 if a['success'] and b['success'] else None})
 EVIDENCE.mkdir(exist_ok=True);(EVIDENCE/'summary.json').write_text(json.dumps({'groups':groups,'trials':rows,'paired':pairs,'final_n':len(finite),'excluded_fixture_trials':[r['run'] for r in rows if r['cohort']!='final_fixture'],'incomplete_run_directories':[p.name for p in sorted((CACHE/'runs').iterdir()) if not (p/'events.json').exists()]},indent=2)+'\n')
 lines=['| 消融/划分 | 场景 | 组 | 到达/安全到达/contact | 到达中位s | 净空min / p50 m | WAIT中位s | native异常 |','| --- | --- | --- | --- | --- | --- | --- | --- |']
 for r in groups:
  f=lambda s,k:'—' if s is None else f'{s[k]:.3f}'
  lines.append(f"| {r['variant']}/{r['split']} | {r['scene']} | {r['group']} | {r['success']}/{r['n']} / {r['safe_success']}/{r['n']} / {r['contact_trials']}/{r['n']} | {f(r['arrival_s'],'p50')} | {f(r['clearance_m'],'min')} / {f(r['clearance_m'],'p50')} | {f(r['wait_s'],'p50')} | {r['native_errors']} |")
 (EVIDENCE/'table.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
if __name__=='__main__':main()
