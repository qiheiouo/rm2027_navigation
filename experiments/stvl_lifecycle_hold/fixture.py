"""Existing smoother/stub/Gazebo actuator chain, one frozen upstream producer.
No MPPI or perception runtime; no second final output owner. All fault input is
software-isolated and no runtime safety policy is implemented.
"""
import pathlib,json,os,sys,subprocess,time,signal,math,csv,collections
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014');PREV=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014');OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013')
sys.path.insert(0,str(PREV/'experiments/stvl_execution_realtime'));from scene_adapter import insert
sys.path.insert(0,str(BASE/'experiments/stvl_c83_endtoend'));from causal_replay import lines
sys.path.insert(0,str(OLD/'experiments/stvl_temporal'));from motion import target
FROZEN=pathlib.Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/experiments/temporal_mpc');sys.path[:0]=[str(FROZEN),str(FROZEN/'gazebo')];from audit_run import physical_projection,hull
import numpy as np
kind=sys.argv[1];assert kind in ('timing','reference','brake','brake_delayed')
label=os.environ.get('HOLD_LABEL',kind);assert label.replace('_','').isalnum();out=pathlib.Path('/check')/label;out.mkdir(exist_ok=False);scene='empty' if kind=='timing' else 'narrow'
source=OLD/'build/roots_cache/assets';(out/'scene.sdf').write_bytes((source/f'{scene}.sdf').read_bytes());identity=insert(out/'scene.sdf',PREV/'build/realtime_cache/small_build/libboundary_observer.so',out/'native.csv');(out/'scene_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
bridge=yaml.safe_load((source/'bridge.yaml').read_text());keep=['/clock','/simulation/chassis/cmd_vel','/simulation/ground_truth/odom','/simulation/oracle/rm_sentry_2027','/simulation/oracle/moving_obstacle','/simulation/oracle/contacts','/simulation/moving_obstacle/target'];(out/'bridge.yaml').write_text(yaml.safe_dump([x for x in bridge if x['ros_topic_name'] in keep]))
config=yaml.safe_load((BASE/'build/c83_cache/shadow_durable/runs/narrow_C_83_aligned/nav2.yaml').read_text());(out/'smoother.yaml').write_text(yaml.safe_dump({'velocity_smoother':config['velocity_smoother']}))
processes=[];streams=[]
def spawn(label,args):
 f=(out/(label+'.log')).open('w');streams.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
spawn('gazebo',['ign','gazebo','-r','-s','--headless-rendering',str(out/'scene.sdf')])
spawn('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p',f'config_file:={out}/bridge.yaml'])
spawn('smoother',['ros2','run','nav2_velocity_smoother','velocity_smoother','--ros-args','--params-file',str(out/'smoother.yaml'),'-r','cmd_vel:=/cmd_vel_nav','-r','cmd_vel_smoothed:=/cmd_vel'])
spawn('receipts',[str(PREV/'build/realtime_cache/small_build/receipt_observer'),str(out/'command_receipts.csv')])
spawn('stub',['ros2','run','rm_chassis_interface','chassis_interface_stub','--ros-args','-p','use_sim_time:=true','-p','cmd_vel_topic:=/cmd_vel','-p','mock_output_cmd_vel_topic:=/simulation/chassis/cmd_vel'])
rclpy.init();node=Node('frozen_lifecycle_fixture',parameter_overrides=[Parameter('use_sim_time',value=True)]);publisher=node.create_publisher(Twist,'/cmd_vel_nav',10);actor=node.create_publisher(Float64,'/simulation/moving_obstacle/target',10);service=node.create_client(ChangeState,'/velocity_smoother/change_state')
f=(out/'observations.csv').open('w');w=csv.writer(f);w.writerow(['topic','ROS_ns','steady_ns','source_ns','x','y','yaw','vx','vy','wz']);inputs=(out/'inputs.jsonl').open('w');truthfile=(out/'truth.jsonl').open('w');odom_latest=None;contacts=[];odom_queue=collections.deque();source_epoch=None;events=[]
def on_cmd(topic,m):w.writerow([topic,node.get_clock().now().nanoseconds,time.monotonic_ns(),0,'','','',m.linear.x,m.linear.y,m.angular.z])
def on_odom(m):
 global odom_latest
 t=m.header.stamp.sec*10**9+m.header.stamp.nanosec;p=m.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));v=m.twist.twist
 row=[t,p.position.x,p.position.y,yaw,v.linear.x,v.linear.y,v.angular.z];odom_latest=row;odom_queue.append(row);w.writerow(['/physical_odom',node.get_clock().now().nanoseconds,time.monotonic_ns(),*row])
def on_truth(name,m):
 try:
  from rosidl_runtime_py.convert import message_to_ordereddict
  t,poly,p,r=physical_projection(message_to_ordereddict(m),name,'base_link' if name=='rm_sentry_2027' else 'obstacle_link')
  if name=='moving_obstacle':poly=hull([(p+r@np.array([x,y,z]))[:2] for x in (-.225,.225) for y in (-.275,.275) for z in (-.4,.4)])
  truthfile.write(json.dumps(dict(model=name,source_ns=t,polygon=poly,position=p.tolist()),separators=(',',':'))+'\n')
 except Exception as exc:events.append(dict(kind='truth_error',error=str(exc)))
def on_contact(m):
 pairs=[(c.collision1.name,c.collision2.name) for c in m.contacts]
 if source_epoch and any('rm_sentry_2027' in str(p) for p in pairs):contacts.append(dict(source_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,receipt_ns=node.get_clock().now().nanoseconds,pairs=pairs))
for t in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel'):node.create_subscription(Twist,t,lambda m,topic=t:on_cmd(topic,m),100)
node.create_subscription(Odometry,'/simulation/ground_truth/odom',on_odom,100)
for name in ('rm_sentry_2027','moving_obstacle'):node.create_subscription(TFMessage,'/simulation/oracle/'+name,lambda m,n=name:on_truth(n,m),100)
node.create_subscription(Contacts,'/simulation/oracle/contacts',on_contact,100)
profiles=['normal','single_100ms','three_period_300ms','missing_cycle','silence_1500ms','stop_ramp','feedback_observer_delay','resume']
commands=[];brake_trigger_s=None
if kind=='timing':
 for i,name in enumerate(profiles):
  for k in range(60):
   t=k*.1;v=.3 if t<2 else .2 if 3.8<=t<4.8 else 0.;delay=0;drop=False
   if name=='single_100ms' and k==20:delay=.1
   if name=='three_period_300ms' and 20<=k<=22:delay=.3
   if name=='missing_cycle' and k==20:drop=True
   if name=='silence_1500ms' and 20<=k<35:drop=True
   commands.append(dict(cycle_id=i*60+k,case=name,logical_s=i*6+t,velocity=[v,0,0],delay_s=delay,drop=drop))
else:
 original=json.loads((BASE/'experiments/stvl_c83_endtoend/evidence/original_forensics.json').read_text());goal=original['events']['goal_ns'];rc=[x for x in original['commands'] if x['topic']=='/cmd_vel_nav' and x['publisher_name']=='controller_server'];
 # Publication sim clock was reconstructed using the original native1ms clock.
 sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows
 p=PREV/'build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned';native=rows(p/'native_boundary.csv');nt=[int(x['steady_ns']) for x in native];sim=[int(x['sim_ns']) for x in native]
 probe=next(x for x in rc if int(x['ROS_receipt_ns'])>=goal+2514000000 and max(abs(float(x[k])-v) for k,v in zip(('vx','vy','wz'),[.4957907199859619,.25456488132476807,.3363713026046753]))<1e-7);probe_pub=int(probe['steady_ns'])+int(probe['DDS_source_ns'])-int(probe['system_ns']);brake_trigger_s=(float(np.interp(probe_pub,nt,sim))-goal)/1e9
 for i,x in enumerate(rc):
  pub=int(x['steady_ns'])+int(x['DDS_source_ns'])-int(x['system_ns']);t=(float(np.interp(pub,nt,sim))-goal)/1e9
  if kind!='reference' and t>=brake_trigger_s:continue
  commands.append(dict(cycle_id=i,case=kind,logical_s=t,velocity=[float(x[k]) for k in ('vx','vy','wz')],delay_s=0,drop=False))
 if kind!='reference':
  for k in range(65):commands.append(dict(cycle_id=1000+k,case=kind,logical_s=brake_trigger_s+k*.1,velocity=[0.,0.,0.],delay_s=.1 if kind=='brake_delayed' and k==0 else 0,drop=False))
 commands.sort(key=lambda x:(x['logical_s'],x['cycle_id']))
active=False;pending=[];idx=0;activate_wall=time.monotonic();last_boot=0;last_actor=0;last_publish=None;end_reason='wall_cap'
try:
 for transition in (1,3):
  assert service.wait_for_service(timeout_sec=30),'smoother lifecycle service absent';q=ChangeState.Request();q.transition.id=transition;future=service.call_async(q);rclpy.spin_until_future_complete(node,future,timeout_sec=10);assert future.done() and future.result().success,'lifecycle transition failed'
 while time.monotonic()-activate_wall<100:
  rclpy.spin_once(node,timeout_sec=.002);now=node.get_clock().now().nanoseconds;wall=time.monotonic_ns()
  if not source_epoch:
   if kind!='timing' and now-last_actor>=10000000:actor.publish(Float64(data=target('narrow',0.,83)));last_actor=now
   if now-last_boot>=50000000:publisher.publish(Twist());last_boot=now
   if now>=4002000000 and odom_latest is not None:source_epoch=now;events.append(dict(kind='fixture_start',sim_ns=now,steady_ns=wall));continue
   continue
  t=(now-source_epoch)/1e9
  if kind!='timing' and now-last_actor>=10000000:actor.publish(Float64(data=target('narrow',t,83)));last_actor=now
  while idx<len(commands) and t>=commands[idx]['logical_s']:
   x=dict(commands[idx]);idx+=1;x['ready_wall_ns']=wall;x['ready_sim_ns']=now
   if x['drop']:x['event']='no_new_output';inputs.write(json.dumps(x)+'\n');continue
   x['due_wall_ns']=wall+round(x['delay_s']*1e9);pending.append(x)
  # Serial output: delayed old cycles cannot overwrite a newer completed command.
  pending.sort(key=lambda x:x['cycle_id'])
  while pending and wall>=pending[0]['due_wall_ns']:
   x=pending.pop(0);m=Twist();m.linear.x,m.linear.y,m.angular.z=[float(v) for v in x['velocity']];publisher.publish(m)
   sample=odom_latest
   if x['case']=='feedback_observer_delay':
    eligible=[z for z in odom_queue if z[0]<=now-200000000];sample=eligible[-1] if eligible else None
   x.update(event='new_output',publish_wall_ns=wall,publish_sim_ns=now,actual_release_to_publish_ms=(wall-x['ready_wall_ns'])/1e6,previous_decision_age_ms=None if last_publish is None else (wall-last_publish)/1e6,feedback_source_ns=None if sample is None else sample[0],feedback_age_ms=None if sample is None else (now-sample[0])/1e6);inputs.write(json.dumps(x)+'\n');inputs.flush();last_publish=wall
  while odom_queue and odom_queue[0][0]<now-1000000000:odom_queue.popleft()
  if t>= (48 if kind=='timing' else 9):end_reason='complete';break
except Exception as exc:
 end_reason='exception';events.append(dict(kind='fixture_exception',type=type(exc).__name__,message=str(exc)));raise
finally:
 end=node.get_clock().now().nanoseconds;(out/'events.json').write_text(json.dumps(dict(kind=kind,brake_trigger_s=brake_trigger_s,origin_ns=source_epoch,end_ns=end,reason=end_reason,events=events,contacts=contacts,producer_owners=node.count_publishers('/cmd_vel_nav'),smoother_owners=node.count_publishers('/cmd_vel'),final_owners=node.count_publishers('/simulation/chassis/cmd_vel'),state_feedback='physical odom, no canonical localization or MPPI in fixture'),indent=2)+'\n');publisher.publish(Twist());f.flush();inputs.flush();truthfile.flush()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
  for p in processes:
   if p.poll() is None:
    try:os.killpg(p.pid,sig)
    except ProcessLookupError:pass
  for p in processes:
   try:p.wait(timeout=2)
   except subprocess.TimeoutExpired:pass
 for a in streams:a.close()
 f.close();inputs.close();truthfile.close();node.destroy_node();rclpy.shutdown()
print(kind,end_reason,flush=True)
