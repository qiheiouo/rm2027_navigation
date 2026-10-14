"""Focused read-only causal measurements; never reclassifies the frozen gate."""
import pathlib,json,math,bisect,sys,csv
from inventory import OLD,OUT,rectangle,signed_gap,walls
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));from analyze import stats,circle_gap,timed_interpolation
from data import jsonlines

def interp(rows,t,key='source_ns',fields=('x','y','yaw','vx','vy','wz')):
 ts=[int(r[key]) for r in rows];i=bisect.bisect_left(ts,t)
 if not i or i>=len(rows):return None
 a,b=rows[i-1],rows[i];alpha=(t-int(a[key]))/(int(b[key])-int(a[key]));result={}
 for k in fields:
  delta=float(b[k])-float(a[k]);delta=math.atan2(math.sin(delta),math.cos(delta)) if k=='yaw' else delta
  result[k]=float(a[k])+alpha*delta
 result.update(query_ns=t,bracket_ns=[int(a[key]),int(b[key])]);return result

def held(rows,t,topic):
 selected=[r for r in rows if r['topic']==topic and int(r['receipt_ns'])<=t]
 return selected[-1] if selected else None

def point_wall(point,wall):
 xs=[p[0] for p in wall];ys=[p[1] for p in wall];x,y=point;dx=max(min(xs)-x,0,x-max(xs));dy=max(min(ys)-y,0,y-max(ys))
 return math.hypot(dx,dy) if dx or dy else -min(x-min(xs),max(xs)-x,y-min(ys),max(ys)-y)

def solid_wall(x,y,yaw,ws):
 body=rectangle(x,y,yaw,.30,.25);c,s=math.cos(yaw),math.sin(yaw)
 centers=[(x+c*a-s*b,y+s*a+c*b) for a in (-.25,.25) for b in (-.225,.225)]
 return min([signed_gap(body,w) for w in ws]+[point_wall(p,w)-.075 for p in centers for w in ws])

def dynamic_gap(c,poses):
 pr=c.get('prediction',{});source=pr.get('source',{});epoch=pr.get('candidate_epoch_ns',pr.get('consume_ns',c['epoch_ns']));mins=[]
 for k,pose in enumerate(poses):
  index=((epoch-source.get('source_ns',0))/1e9+(k+1)*.1)/.1
  for tr in source.get('tracks',[]):
   if tr['state'] not in (2,3) or not 0<=index<=35:continue
   a=min(35,int(index));b=min(35,a+1);q=[tr['points'][a][j]+(index-a)*(tr['points'][b][j]-tr['points'][a][j]) for j in (0,1)]
   mins.append(circle_gap(pose,q,max(.36,math.hypot(*tr['size'])/2)))
 return min(mins) if mins else None

def brake_witness(c,ws,feedback):
 pose=list(c['pose']);v=[feedback[k] for k in ('vx','vy','wz')];roll=[]
 for k in range(30):
  v=[math.copysign(max(0,abs(u)-bound*.1),u) for u,bound in zip(v,(.8,.8,2.))]
  yaw=pose[2];pose=[pose[0]+(math.cos(yaw)*v[0]-math.sin(yaw)*v[1])*.1,pose[1]+(math.sin(yaw)*v[0]+math.cos(yaw)*v[1])*.1,yaw+v[2]*.1];roll.append(pose)
 return {'basis':'offline bounded braking witness only; input pose plus existing recorded /odometry/lio body velocity interpolated at pose epoch (native speed input was erroneously zero), .8/.8/2 ramps, .1s, 30 steps; not Gazebo execution or deployed fallback','initial_measured_velocity':[feedback[k] for k in ('vx','vy','wz')],'dynamic_circle_min_m':dynamic_gap(c,roll),'wall_envelope_min_m':min(signed_gap(rectangle(*p,.325,.300),w) for p in roll for w in ws),'first_pose':roll[0],'last_pose':roll[-1]}

def run(case):
 p=OLD/'build/stvl_cache/runs'/case['run'];e=json.loads((p/'events.json').read_text());start=e['goal_ns'];end=e['end_ns'];ws=walls(p);truth=jsonlines(p/'truth.jsonl');robots=[r for r in truth if r['model']=='rm_sentry_2027'];actors=[r for r in truth if r['model']=='moving_obstacle'];at=[r['source_ns']/1e9 for r in actors];axy=[r['position'][:2] for r in actors];controls=[r for r in jsonlines(p/'control.jsonl') if start<=r['epoch_ns']<=end];od=list(csv.DictReader((p/'odometry.csv').open()));cmd=list(csv.DictReader((p/'commands.csv').open()));solid=[]
 for r in robots:
  if start<=r['source_ns']<=end:
   yaw=math.atan2(r['rotation'][1][0],r['rotation'][0][0]);solid.append(solid_wall(*r['position'][:2],yaw,ws))
 first=case.get('first_wall');contact=case.get('first_contact');moment=first['source_ns'] if first else contact['source_ns'] if contact else start+3000000000
 near=[r for r in controls if r['epoch_ns']<=moment];chosen=[near[-1]] if near else []
 active=[r for r in controls if any(t['state'] in (2,3) for t in r.get('prediction',{}).get('source',{}).get('tracks',[]))]
 if active:chosen.append(active[0])
 chosen.extend(r for r in controls if 'critic_deltas' in r and moment-500000000<=r['epoch_ns']<=moment+100000000)
 if contact:
  near_contact=[r for r in controls if r['epoch_ns']<=contact['source_ns']]
  if near_contact:chosen.append(near_contact[-1])
 chosen=list({r['epoch_ns']:r for r in chosen}.values());rows=[]
 for c in sorted(chosen,key=lambda r:r['epoch_ns']):
  epoch=c.get('pose_epoch_ns',c['epoch_ns']);future=epoch+100000000;actual=interp(od,future);ag=c.get('aggregate',[]);pr=c.get('prediction',{});parts=c.get('critic_deltas',{});cstamp=c['epoch_ns'];r={'relative_s':(cstamp-start)/1e9,'epoch_ns':cstamp,'input_pose_ns':epoch,'input_pose':c['pose'],'feedback_body_velocity':c['speed'],'compute_ms':c['compute_ms'],'raw_command':c.get('command'),'error':c.get('error'),'actual_next_feedback_interpolated':actual,'model_next_post_shift_pose':ag[0] if ag else None,'held_commands_at_entry':{t:held(cmd,cstamp,t) for t in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel')},'held_commands_at_next_feedback':{t:held(cmd,future,t) for t in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel')},'prediction':pr,'critic_ranges':{k:stats(v) for k,v in parts.items()}}
  if actual and ag:r.update(next_position_error_m=math.dist(ag[0][:2],[actual['x'],actual['y']]),next_yaw_error_rad=math.atan2(math.sin(ag[0][2]-actual['yaw']),math.cos(ag[0][2]-actual['yaw'])))
  if ag:
   r['post_shift_aggregate_wall_min_m']=min(signed_gap(rectangle(*p,.34,.29),w) for p in ag for w in ws);r['aggregate_dynamic_predicted_circle_min_m']=dynamic_gap(c,ag)
   dense=[];previous=c['pose']
   for pose in ag:
    dyaw=math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2]))
    dense.extend([[previous[0]+q*(pose[0]-previous[0]),previous[1]+q*(pose[1]-previous[1]),previous[2]+q*dyaw] for q in (.1,.2,.3,.4,.5,.6,.7,.8,.9,1.)]);previous=pose
   r['linear_10ms_sweep_wall_min_m']=min(signed_gap(rectangle(*p,.34,.29),w) for p in dense for w in ws)
  errs=[]
  for tr in pr.get('source',{}).get('tracks',[]):
   if tr['state'] not in (2,3):continue
   for k,point in enumerate(tr['points']):
    query=pr['source']['source_ns']/1e9+k*.1;actual_center=timed_interpolation(at,axy,query)
    if actual_center:errs.append(math.dist(point,actual_center))
  r['consumed_grid_vs_same_run_actor_truth_error_m']=stats(errs)
  current_feedback=interp(od,epoch)
  r['measured_feedback_at_input_epoch']=current_feedback
  if active and c is active[0] and current_feedback:r['bounded_brake_reference']=brake_witness(c,ws,current_feedback)
  rows.append(r)
 result={'run':p.name,'frozen_gate_unchanged':True,'plan_geometry':[.34,.29],'frozen_bbox':[.325,.300],'proxy_solid_union':'body .30/.25 + wheel circles at +/-.25,+/-.225 radius .075; planar audit only, link articulations/roll/pitch not reconstructed','minimum_planar_solid_union_wall_gap_m':min(solid),'solid_union_overlaps':sum(g<0 for g in solid),'cycles':rows,'first_wall':first,'first_contact':contact}
 if first:result['solid_union_gap_at_first_frozen_bbox_failure_m']=solid_wall(*first['position'][:2],first['yaw'],ws)
 if contact:
  rr=min(robots,key=lambda r:abs(r['source_ns']-contact['source_ns']));aa=min(actors,key=lambda r:abs(r['source_ns']-contact['source_ns']));yaw=math.atan2(rr['rotation'][1][0],rr['rotation'][0][0]);result['contact_geometry_proxy']={'sample_ns':rr['source_ns'],'offset_to_sensor_contact_ms':(rr['source_ns']-contact['source_ns'])/1e6,'robot_center':rr['position'],'actor_center':aa['position'],'front_right_wheel_center_planar':[rr['position'][0]+math.cos(yaw)*.25+math.sin(yaw)*.225,rr['position'][1]+math.sin(yaw)*.25-math.cos(yaw)*.225],'exact_contact_point_3d':'Evidence Missing: original recorder retained sensor pair/time only'}
 return result

def probes():
 rows=[]
 for cohort in ('diag','repair','repair_abi','heldout_abi','heldout_abi_retry','repair_odom','heldout_odom'):
  for p in sorted((ROOT/'build/roots_cache'/cohort/'runs').glob('*')):
   if not (p/'events.json').exists():continue
   start=json.loads((p/'events.json').read_text())['goal_ns']
   for c in jsonlines(p/'control.jsonl'):
    probe=c.get('root_diagnostics',{}).get('cost_probe');cost=c.get('native_final_costs_after_gamma')
    if not probe or not cost:continue
    weights=[math.exp(-(x-min(cost))/c['native_temperature']) for x in cost];z=sum(weights);bad=probe['full_native_footprint_hits'];native=probe['native_shortcut_hits'];dyn=c.get('prediction',{}).get('candidate_min_clearance',[]);joint=[not bad[i] and (not dyn or dyn[i]>.02) for i in range(len(bad))]
    noised=c['root_diagnostics'].get('candidate_noised_controls',[]);weighted=[[sum(w*noised[i][k][j] for i,w in enumerate(weights))/z for j in range(3)] for k in range(3)] if noised else None
    ag=c.get('aggregate',[]);ws=walls(p)
    rows.append({'cohort':cohort,'run':p.name,'relative_s':(c['epoch_ns']-start)/1e9,'epoch_ns':c['epoch_ns'],'full_footprint_hit_candidates':sum(bad),'shortcut_hit_candidates':sum(native),'missed_pose_checks':probe['missed_pose_checks'],'softmax_mass_on_full_footprint_hits':sum(w for w,b in zip(weights,bad) if b)/z,'joint_static_dynamic_safe_candidates':sum(joint),'dynamic_safe_candidates':c.get('prediction',{}).get('safe_candidates'),'command':c.get('command'),'weighted_noised_controls_first3_before_constraints_sg_shift':weighted,'first_selected_post_shift_control':c.get('selected_post_sg_shift_controls',[None])[0],'selected_equals_returned':math.dist(c['command'],c['selected_post_sg_shift_controls'][0])<1e-6,'post_shift_aggregate_wall_min_m':min(signed_gap(rectangle(*a,.34,.29),w) for a in ag for w in ws) if ws and ag else None,'post_shift_aggregate_dynamic_circle_min_m':dynamic_gap(c,ag),'native_feedback_speed':c['speed'],'raw_candidate_first_velocity':c['root_diagnostics'].get('candidate_velocity',[[None]])[0][0],'cost_storage':c['root_diagnostics'].get('cost_storage'),'compute_ms':c['compute_ms'],'examples':probe['examples'][:2]})
 return rows
if __name__=='__main__':
 inventory=json.loads((OUT/'original_inventory.json').read_text());result={'base':inventory['base'],'aggregate_caution':'SDK trajectory is reconstructed AFTER native SG and control-sequence shift; not the lowest raw sample. First control matches returned Twist in new observer. Truth interpolated at input pose epoch+.1, bracket explicitly retained. 10ms sweep is an offline linear interpolation audit, not a new runtime checker.','original_cases':[run(c) for c in inventory['cases'][:6]],'native_probe_cycles':probes()};(OUT/'causal_measurements.json').write_text(json.dumps(result,indent=2)+'\n');print('wrote original focused cases and',len(result['native_probe_cycles']),'instrumented cycles')
