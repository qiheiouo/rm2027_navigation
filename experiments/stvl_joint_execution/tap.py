"""Read-only receipt times and endpoint inventory; no command/TF publishers.

Fixed Humble rclpy invokes callbacks with the message only. Per-message GID
and DDS timestamps are unavailable here; receipt is not application time.
"""
import json,pathlib,sys,time
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
ROOT=pathlib.Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('joint_execution_receipt_tap')
node.set_parameters([Parameter('use_sim_time',value=True)])
stream=(ROOT/'execution_receipts.jsonl').open('w');graph=(ROOT/'execution_endpoints.jsonl').open('w')
def receive(message,topic):
    v=message.twist.twist if topic=='/odometry/lio' else message
    row={'topic':topic,'ROS_receipt_ns':node.get_clock().now().nanoseconds,'monotonic_receipt_ns':time.monotonic_ns(),
         'DDS_source_ns':None,'DDS_received_ns':None,'publisher_gid':None,
         'velocity':[v.linear.x,v.linear.y,v.angular.z]}
    if topic=='/odometry/lio':row['state_source_ns']=message.header.stamp.sec*10**9+message.header.stamp.nanosec
    stream.write(json.dumps(row,separators=(',',':'))+'\n')
def callback(topic):
    def call(message):receive(message,topic)
    return call
for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel'):
    node.create_subscription(Twist,topic,callback(topic),100)
node.create_subscription(Odometry,'/odometry/lio',lambda m:receive(m,'/odometry/lio'),qos_profile_sensor_data)
def inventory():
    rows={t:[{'name':e.node_name,'namespace':e.node_namespace,'gid':bytes(e.endpoint_gid).hex()} for e in node.get_publishers_info_by_topic(t)] for t in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel')}
    graph.write(json.dumps({'ROS_receipt_ns':node.get_clock().now().nanoseconds,'topics':rows})+'\n');graph.flush();stream.flush()
node.create_timer(1.,inventory)
try:rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:stream.close();graph.close();node.destroy_node();rclpy.try_shutdown()
