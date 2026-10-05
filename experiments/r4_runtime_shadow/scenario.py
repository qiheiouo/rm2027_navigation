#!/usr/bin/env python3
"""Only native navigation goal and existing Gazebo obstacle target; no cmd_vel."""
import argparse, json, pathlib, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState

p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('output',type=pathlib.Path);a=p.parse_args()
rclpy.init();node=Node('r4_shadow_scenario',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
client=ActionClient(node,NavigateToPose,'/navigate_to_pose');pub=node.create_publisher(Float64,'/simulation/moving_obstacle/target',10)
state_client=node.create_client(GetState,'/bt_navigator/get_state');state_future=None;active=False
events=[];latest={'scan':0,'odom':0};begin=time.monotonic();goal_start=None;result=None
node.create_subscription(LaserScan,'/scan',lambda m:latest.update(scan=m.header.stamp.sec*10**9+m.header.stamp.nanosec),rclpy.qos.qos_profile_sensor_data)
node.create_subscription(Odometry,'/odometry/lio',lambda m:latest.update(odom=m.header.stamp.sec*10**9+m.header.stamp.nanosec),rclpy.qos.qos_profile_sensor_data)
def event(kind,**data):events.append(dict(kind=kind,ROS_ns=node.get_clock().now().nanoseconds,steady_ns=time.monotonic_ns(),**data))
try:
    while time.monotonic()-begin<90:
        rclpy.spin_once(node,timeout_sec=.02);now=node.get_clock().now().nanoseconds
        if goal_start is None and not active:
            if state_future is not None and state_future.done():
                active=state_future.result().current_state.id==3;state_future=None
            if not active and state_future is None and state_client.service_is_ready():
                state_future=state_client.call_async(GetState.Request())
        if a.scene!='S0':
            t=0 if goal_start is None else (now-goal_start)/1e9
            target=(-.9 if t<1 else min(.9,-.9+.9*(t-1))) if a.scene=='S1' else (-.9 if t<1 else -.9*(2-t) if t<2 else 0. if t<7 else min(.9,.45*(t-7)))
            pub.publish(Float64(data=target))
        if goal_start is None and active and latest['scan']>0 and latest['odom']>0 and client.server_is_ready():
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg();goal.pose.pose.position.x=4.;goal.pose.pose.orientation.w=1.
            fut=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,fut,timeout_sec=5)
            if not fut.done() or not fut.result().accepted:event('goal_rejected');break
            handle=fut.result();result=handle.get_result_async();goal_start=node.get_clock().now().nanoseconds;event('goal_accepted')
        if goal_start is None and time.monotonic()-begin>30:
            event('startup_not_ready');break
        if goal_start is not None:
            t=(now-goal_start)/1e9
            for boundary,kind in [(1,'obstacle_motion_start'),(3 if a.scene=='S1' else 2,'crossed_or_hold'),(3 if a.scene=='S1' else 9,'clear_target')]:
                if a.scene!='S0' and t>=boundary and not any(e['kind']==kind for e in events):event(kind)
            if result.done() and not any(e['kind']=='native_goal_result' for e in events):event('native_goal_result',status=result.result().status)
            if t>=20:event('observation_end');break
    else:event('wall_cap')
finally:
    (a.output/'events.json').write_text(json.dumps(dict(scene=a.scene,events=events,last_input_stamps=latest),indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
