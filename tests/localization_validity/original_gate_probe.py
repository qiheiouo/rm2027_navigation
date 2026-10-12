import json,subprocess,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from std_msgs.msg import Bool
OUT=Path(os.environ['PROBE_OUTPUT']);OUT.mkdir(parents=True,exist_ok=True);Q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);logs=[];procs=[];result={}
rclpy.init();node=Node('audit_gate_pending_observer');vals=[];tfs=[]
node.create_subscription(Bool,'/localization/global_localization_valid',lambda m:vals.append((time.monotonic(),m.data)),Q)
node.create_subscription(TransformStamped,'/localization/map_to_odom',lambda m:tfs.append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec/1e9)),10)
rawpub=node.create_publisher(PoseWithCovarianceStamped,'/localization/amcl_pose_raw',10);opub=node.create_publisher(Odometry,'/odometry/lio',100)
def pose(stamp):
 m=PoseWithCovarianceStamped();m.header.stamp=stamp;m.header.frame_id='map';m.pose.pose.orientation.w=1.;return m
def odom(stamp):
 m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;return m
def launch(exe,args):
 l=(OUT/(exe+'_gate_pending.log')).open('a');logs.append(l);p=subprocess.Popen([os.environ['PROBE_INSTALL']+'/rm_relocalization_bridge/lib/rm_relocalization_bridge/'+exe,'--ros-args',*args],stdout=l,stderr=subprocess.STDOUT);procs.append(p);return p
def spin(sec,raw=False,od=False):
 end=time.monotonic()+sec;last=0
 while time.monotonic()<end:
  if time.monotonic()-last>.04:
   stamp=node.get_clock().now().to_msg()
   if od:opub.publish(odom(stamp))
   if raw:rawpub.publish(pose(stamp))
   last=time.monotonic()
  rclpy.spin_once(node,timeout_sec=.01)
def state():return {'latest_valid':vals[-1][1] if vals else None,'transforms_last_half_second':sum(t>time.monotonic()-.5 for t,_ in tfs)}
try:
 bridge=launch('map_odom_from_global_pose',['-p','upstream_valid_topic:=/localization/amcl_backend_valid'])
 gate=launch('global_pose_gate_node',[]);spin(1.5,True,True);result['healthy_actual_gate']=state();assert result['healthy_actual_gate']['latest_valid']
 spin(1.5,False,True);result['raw_pose_stops_gate_alive']=state();assert result['raw_pose_stops_gate_alive']['latest_valid'] is False
 spin(1,True,True);result['raw_pose_restored']=state();assert result['raw_pose_restored']['latest_valid']
 gate.terminate();gate.wait(timeout=2);spin(1.5,False,True);result['actual_gate_process_exits_odom_continues']=state();assert result['actual_gate_process_exits_odom_continues']['latest_valid'] is (not bool(os.environ.get('PROBE_EXPECT_FIXED')))
 bridge.terminate();bridge.wait(timeout=2);spin(.2);vals.clear();tfs.clear()
 bridge=launch('map_odom_from_global_pose',['-p','global_pose_topic:=/audit/pending_pose','-p','odom_topic:=/audit/pending_odom','-p','pending_pose_max_wait_sec:=0.2','-p','max_global_pose_age_sec:=0.5'])
 gp=node.create_publisher(PoseWithCovarianceStamped,'/audit/pending_pose',10);od=node.create_publisher(Odometry,'/audit/pending_odom',100);spin(.6)
 stamp=node.get_clock().now().to_msg();sent=time.monotonic();gp.publish(pose(stamp));spin(.8);od.publish(odom(stamp));spin(.3)
 result['pending_matching_odom_after_deadline']={**state(),'configured_max_wait_sec':.2,'configured_max_pose_age_sec':.5,'matching_odom_delay_sec':.8,'global_pose_age_at_result_sec':time.monotonic()-sent};assert result['pending_matching_odom_after_deadline']['latest_valid'] is (not bool(os.environ.get('PROBE_EXPECT_FIXED')))
finally:
 for p in procs:
  if p.poll() is None:
   p.terminate()
   try:p.wait(timeout=2)
   except subprocess.TimeoutExpired:p.kill();p.wait()
 node.destroy_node();rclpy.shutdown()
 for l in logs:l.close()
 (OUT/'probe_gate_pending.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
