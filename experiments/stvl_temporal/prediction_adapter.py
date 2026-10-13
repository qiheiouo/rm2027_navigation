"""Existing canonical state -> research points. No tracking, no velocity output."""
import json,pathlib,sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String,Int64
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
scene,out=sys.argv[1],pathlib.Path(sys.argv[2]);manifest=json.loads((out/'manifest.json').read_text());mode=manifest['mode']
rclpy.init();node=Node('research_prediction_adapter',parameter_overrides=[Parameter('use_sim_time',value=True)])
publisher=node.create_publisher(String,'/research/time_predictions',1);epoch=[None]
node.create_subscription(Int64,'/research/trial_epoch',lambda m:epoch.__setitem__(0,m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
oracle_file=pathlib.Path('/check/assets')/f'oracle_{scene}_{manifest["repeat"]}.json'
oracle=json.loads(oracle_file.read_text()) if mode=='D' and oracle_file.exists() else None
def ns(t):return t.sec*10**9+t.nanosec
def callback(message):
 source=ns(message.header.stamp);relative=(node.get_clock().now().nanoseconds-epoch[0])/1e9 if epoch[0] else 0.
 if scene=='stale' and 4<=relative<5.2:return
 tracks=[];complete=message.complete
 for t in message.tracks:
  vx,vy=(0.,0.) if mode=='B' else (t.velocity.x,t.velocity.y)
  points=[[t.position.x+vx*k*.1,t.position.y+vy*k*.1] for k in range(36)]
  if mode=='D':
   if oracle is None or epoch[0] is None:complete=False
   else:
    import bisect
    points=[]
    for k in range(36):
     time=(source-epoch[0])/1e9+k*.1;index=bisect.bisect_left(oracle['t'],time);index=max(1,min(index,len(oracle['t'])-1));a,b=index-1,index
     alpha=max(0.,min(1.,(time-oracle['t'][a])/(oracle['t'][b]-oracle['t'][a])));points.append([oracle['xy'][a][i]+alpha*(oracle['xy'][b][i]-oracle['xy'][a][i]) for i in (0,1)])
  tracks.append(dict(id=t.track_id,state=t.state,position=[t.position.x,t.position.y],velocity=[vx,vy],size=[t.size.x,t.size.y],observed_ns=ns(t.last_observation_stamp),points=points))
 payload=dict(schema='rm_time_prediction/research_v1',frame=message.header.frame_id,mode=mode,source_ns=source,processing_ns=ns(message.processing_stamp),publish_ns=node.get_clock().now().nanoseconds,dt=.1,complete=complete,tracks=tracks)
 publisher.publish(String(data=json.dumps(payload,separators=(',',':'))))
node.create_subscription(DynamicObstaclePredictionArray,'/perception/dynamic_obstacles_shadow/predictions',callback,1)
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.shutdown()
