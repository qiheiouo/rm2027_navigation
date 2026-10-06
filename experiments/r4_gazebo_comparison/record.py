"""Inputs/commands and independent oracle recording; never control with oracle."""
import collections,csv,json,math,pathlib,sys,time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64,String
from ros_gz_interfaces.msg import Contacts
from rm_r4_interfaces.msg import ObservedPredictionEnvelope
# Read-only reuse of independent physical observer; no R3 controller or launch.
frozen=pathlib.Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/experiments/temporal_mpc')
sys.path[:0]=[str(frozen),str(frozen/'gazebo')]
from audit_run import physical_projection,polygon_distance

def ns(stamp): return stamp.sec*10**9+stamp.nanosec
scene,mode=sys.argv[1:3];out=pathlib.Path(sys.argv[3]);rclpy.init()
scenario_file=out.parent.parent/'assets/scenario.json'
scenario=json.loads(scenario_file.read_text()) if scenario_file.exists() else {}
motion=scenario.get('motion',{}).get(scene)
node=Node('r4_comparison_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
latest={};ring=collections.deque(maxlen=150);events=[];truth={};positions={};contacts=[];contact_seen=0;failure=None;goal_ns=None;result_future=None;handle=None;finished=None
streams={name:(out/name).open('w') for name in ('truth.jsonl','odometry.csv','commands.csv','prediction.jsonl','costmap.csv')}
if scenario: streams['plans.jsonl']=(out/'plans.jsonl').open('w')
odom=csv.writer(streams['odometry.csv']);odom.writerow(['source_ns','receipt_ns','x','y','yaw','vx','vy','wz'])
cmd=csv.writer(streams['commands.csv']);cmd.writerow(['topic','receipt_ns','vx','vy','wz'])
costmap=csv.writer(streams['costmap.csv']);costmap.writerow(['source_ns','receipt_ns','frame','lethal','positive','actor_window_lethal'])
actor=node.create_publisher(Float64,'/simulation/moving_obstacle/target',10)
bootstrap=node.create_publisher(Twist,'/cmd_vel_nav',10)
client=ActionClient(node,NavigateToPose,'/navigate_to_pose');state=node.create_client(GetState,'/bt_navigator/get_state');state_future=None;active=False
wall_begin=time.monotonic()
def now(): return node.get_clock().now().nanoseconds
def event(kind,**data): events.append(dict(kind=kind,ROS_ns=now(),wall_ns=time.monotonic_ns(),**data))
def plain(message): return message_to_ordereddict(message)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def on_odom(m):
 latest['odom']=ns(m.header.stamp);p=m.pose.pose;t=m.twist.twist
 odom.writerow([ns(m.header.stamp),now(),p.position.x,p.position.y,yaw(p.orientation),t.linear.x,t.linear.y,t.angular.z]);ring.append(('odom',plain(m)))
def on_command(topic,m):
 cmd.writerow([topic,now(),m.linear.x,m.linear.y,m.angular.z]);latest['command']=now()
def on_scan(m): latest['scan']=ns(m.header.stamp);ring.append(('scan',plain(m)))
def on_prediction(m):
 latest['prediction']=ns(m.prediction.header.stamp)
 if goal_ns is not None: streams['prediction.jsonl'].write(json.dumps({'receipt_ns':now(),'message':plain(m)},separators=(',',':'))+'\n')
 ring.append(('prediction',plain(m)))
def on_plan(m):
 ring.append(('plan',plain(m)))
 if scenario and goal_ns is not None:
  streams['plans.jsonl'].write(json.dumps(dict(receipt_ns=now(),source_ns=ns(m.header.stamp),xy=[[p.pose.position.x,p.pose.position.y] for p in m.poses]),separators=(',',':'))+'\n')
def on_truth(model,m):
 try:
  source,polygon,p,r=physical_projection(plain(m),model,'base_link' if model=='rm_sentry_2027' else 'obstacle_link')
  latest[model]=source;truth.setdefault(source,{})[model]=polygon;positions[model]=p.tolist()
  streams['truth.jsonl'].write(json.dumps(dict(model=model,source_ns=source,polygon=polygon,position=p.tolist(),rotation=r.tolist()),separators=(',',':'))+'\n')
  if len(truth)>400: truth.pop(next(iter(truth)))
 except Exception as exc: latest['oracle_error']=str(exc)
def on_costmap(m):
 # Observation only. This scene's map->odom is the original identity stub.
 lethal=sum(v>=99 for v in m.data);positive=sum(v>0 for v in m.data);near=None
 if 'moving_obstacle' in positions and m.header.frame_id=='odom':
  x,y,z=positions['moving_obstacle'];info=m.info;res=info.resolution;origin=info.origin.position
  near=0
  for row in range(max(0,int((y-.4-origin.y)/res)),min(info.height,int((y+.4-origin.y)/res)+1)):
   for col in range(max(0,int((x-.4-origin.x)/res)),min(info.width,int((x+.4-origin.x)/res)+1)):
    near+=m.data[row*info.width+col]>=99
 costmap.writerow([ns(m.header.stamp),now(),m.header.frame_id,lethal,positive,near])
def on_contact(m):
 global contact_seen
 contact_seen+=1
 if goal_ns is None: return
 pairs=[(c.collision1.name,c.collision2.name) for c in m.contacts]
 robot=[p for p in pairs if 'rm_sentry_2027' in str(p)]
 if robot: contacts.append(dict(source_ns=ns(m.header.stamp),receipt_ns=now(),pairs=robot));event('actor_contact',pairs=robot)
def on_failure(m):
 global failure
 if goal_ns is not None and failure is None: failure=m.data;event('controller_failure',reason=m.data)
node.create_subscription(Odometry,'/odometry/lio',on_odom,qos_profile_sensor_data)
node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
node.create_subscription(ObservedPredictionEnvelope,'/perception/dynamic_obstacles_shadow/observed_predictions',on_prediction,10)
node.create_subscription(OccupancyGrid,'/map',lambda m:latest.update(map=max(1,ns(m.header.stamp))),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(OccupancyGrid,'/local_costmap/costmap',on_costmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(Path,'/plan',on_plan,10)
for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel'): node.create_subscription(Twist,topic,lambda m,t=topic:on_command(t,m),10)
for model in ('rm_sentry_2027','moving_obstacle'): node.create_subscription(TFMessage,f'/simulation/oracle/{model}',lambda m,name=model:on_truth(name,m),100)
node.create_subscription(Contacts,'/simulation/oracle/contacts',on_contact,100)
node.create_subscription(String,'/research/failure',on_failure,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 while time.monotonic()-wall_begin<130:
  rclpy.spin_once(node,timeout_sec=.01);current=now();t=0 if goal_ns is None else (current-goal_ns)/1e9
  if scene!='S0':
   target=min(motion['end_y'],motion['start_y']+motion['speed']*max(0.,t-motion['begin_s'])) if motion else ((-.9 if t<1 else min(.9,-.9+.9*(t-1))) if scene=='S1' else (-.9 if t<1 else -.9*(2-t) if t<2 else 0. if t<7 else min(.9,.45*(t-7))))
   actor.publish(Float64(data=target))
  if goal_ns is None:
   if state_future is not None and state_future.done(): active=state_future.result().current_state.id==3;state_future=None
   if not active and state_future is None and state.service_is_ready(): state_future=state.call_async(GetState.Request())
   # Common pre-goal zero through the original smoother/stub. Receipt supplies
   # genuine output history; stop this test input before any controller runs.
   if active: bootstrap.publish(Twist())
   required=['odom','scan','prediction','map','command','rm_sentry_2027']+([] if scene=='S0' else ['moving_obstacle'])
   if active and current>4_000_000_000 and all(latest.get(k,0)>0 for k in required) and client.server_is_ready():
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg();goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
    request=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,request,timeout_sec=5.)
    if not request.done() or not request.result().accepted: finished='goal_rejected';event(finished);break
    handle=request.result();goal_ns=now();result_future=handle.get_result_async();event('goal_accepted',latest=latest.copy())
   if time.monotonic()-wall_begin>60: finished='startup_not_ready';event(finished,latest=latest);break
  else:
   boundaries=[(motion['begin_s'],'obstacle_motion_start'),(motion['begin_s']-motion['start_y']/motion['speed'],'crossed_or_hold'),(motion['begin_s']+(motion['end_y']-motion['start_y'])/motion['speed'],'clear_target')] if motion else [(1,'obstacle_motion_start'),(3 if scene=='S1' else 2,'crossed_or_hold'),(3 if scene=='S1' else 9,'clear_target')]
   for boundary,kind in boundaries:
    if scene!='S0' and t>=boundary and not any(e['kind']==kind for e in events): event(kind)
   if failure or contacts: finished='controller_failure' if failure else 'actor_contact';break
   if result_future.done(): finished='goal_result';event(finished,status=result_future.result().status);break
   if t>=30: finished='trial_timeout';event(finished);break
 else: finished='wall_cap';event(finished)
finally:
 result=result_future.result().status if result_future is not None and result_future.done() else None
 if finished!='goal_result' or result!=4: (out/'first_failure_inputs.json').write_text(json.dumps(list(ring),separators=(',',':'))+'\n')
 (out/'events.json').write_text(json.dumps(dict(scene=scene,mode=mode,events=events,goal_ns=goal_ns,end_ns=now(),result_status=result,finished=finished,controller_failure=failure,contacts=contacts,contact_seen=contact_seen,contacts_publishers=node.count_publishers('/simulation/oracle/contacts'),latest=latest),indent=2)+'\n')
 if handle is not None and result is None: handle.cancel_goal_async()
 for _ in range(20): rclpy.spin_once(node,timeout_sec=.01)
 for stream in streams.values(): stream.close()
 node.destroy_node();rclpy.shutdown()
