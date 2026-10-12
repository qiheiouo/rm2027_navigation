#!/usr/bin/env python3
"""One bounded geometry/pointcloud/Nav2 smoke; no controller optimization."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.time import Time


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--no-nav2',action='store_true')
    p.add_argument('--no-sensors',action='store_true')
    p.add_argument('--software-entry',action='store_true')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    log=(args.output/'launch.log').open('w')
    env=dict(os.environ)
    command=['ros2','launch','rm_new_car_simulation','baseline.launch.py',
      'sensors:='+str(not args.no_sensors).lower(),'use_nav2:='+str(not args.no_nav2).lower(),'spawn:=-3,-3,.002,0,0,0',
      'output:='+str(args.output/'scene')]
    if args.software_entry:
        if not args.no_sensors or args.no_nav2:raise ValueError('software entry is Nav2 with no synthetic lidar')
        command=['ros2','launch','rm_navigation_bringup','new_car_software.launch.py','mode:=s2','enable_test:=true','output:='+str(args.output/'scene')]
    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    rclpy.init();node=Node('new_car_baseline_smoke');clouds=[];poses=[];maps=[];transform_errors=[]
    buffer=Buffer();listener=TransformListener(buffer,node)
    sub=node.create_subscription(PointCloud2,'/points/simulation_raw',clouds.append,10)
    odom=node.create_subscription(Odometry,'/simulation/ground_truth/odom',poses.append,20)
    map_sub=node.create_subscription(OccupancyGrid,'/map',maps.append,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    audit=subprocess.Popen(['ros2','run','rm_new_car_simulation','s2_tf_owner_probe','--ros-args','-p','output:='+str(args.output/'tf_owners.json')],stdout=log,stderr=subprocess.STDOUT,env=env)
    final_commands=[]
    node.create_subscription(Twist,'/cmd_vel',lambda m:final_commands.append(m.angular.z),50)
    pub=node.create_publisher(Float64,'/simulation/gimbal_velocity',10)
    action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    def spin(seconds):
        start=time.monotonic()
        while time.monotonic()-start<seconds and child.poll() is None:
            pub.publish(Float64(data=.6));rclpy.spin_once(node,timeout_sec=.03)
    result={'hardware_validated':False,'simulation_oracle':True}
    try:
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and child.poll() is None:
            spin(.2)
            if (args.no_sensors or len(clouds)>10) and (args.no_nav2 or action.server_is_ready() and maps): break
        result['pointcloud_messages']=len(clouds)
        result['pointcloud_frames']=sorted({m.header.frame_id for m in clouds})
        result['pointcloud_sizes']=sorted({m.width*m.height for m in clouds})
        result['pointcloud_fields']=sorted({f.name for m in clouds for f in m.fields})
        result['pointcloud_sim_stamp_nonzero']=bool(clouds and clouds[-1].header.stamp.sec>0)
        if clouds:
            for m in clouds[-10:]:
                try:
                    transform=buffer.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp))
                    transform_errors.append(False)
                except Exception: transform_errors.append(True)
        result['pointcloud_tf_lookup_successes']=len(transform_errors)-sum(transform_errors)
        result['pointcloud_gate']=bool(len(clouds)>10 and result['pointcloud_frames']==['sim_lidar_link'] and
                                      result['pointcloud_tf_lookup_successes']>=5)
        result['map_received']=bool(maps)
        if maps:
            m=maps[-1];result['map_metadata']={'frame':m.header.frame_id,'origin':[m.info.origin.position.x,m.info.origin.position.y],
              'resolution':m.info.resolution,'size':[m.info.width,m.info.height]}
        result['nav2_action_ready']=action.server_is_ready()
        if not args.no_nav2 and action.server_is_ready():
            state_client=node.create_client(GetState,'/bt_navigator/get_state')
            deadline=time.monotonic()+15;active=False
            while not active and time.monotonic()<deadline:
                if not state_client.service_is_ready():spin(.1);continue
                state=state_client.call_async(GetState.Request())
                while not state.done() and time.monotonic()<deadline:spin(.1)
                active=bool(state.done() and state.result().current_state.id==State.PRIMARY_STATE_ACTIVE)
                if not active:spin(.1)
            result['bt_navigator_active']=active
            parameters=AsyncParameterClient(node,'/controller_server')
            requested=parameters.get_parameters(['odom_topic']);deadline=time.monotonic()+5
            while not requested.done() and time.monotonic()<deadline:spin(.1)
            result['controller_odom_topic']=requested.result().values[0].string_value if requested.done() else None
            discovery_end=time.monotonic()+5
            while time.monotonic()<discovery_end and not any('/odometry/simulation'==name and 'nav_msgs/msg/Odometry' in types for name,types in node.get_subscriber_names_and_types_by_node('controller_server','/')):spin(.1)
            result['controller_odom_subscriptions']=[name for name,types in
                node.get_subscriber_names_and_types_by_node('controller_server','/') if 'nav_msgs/msg/Odometry' in types]
            result['simulation_odom_publishers']=[info.node_name for info in
                node.get_publishers_info_by_topic('/odometry/simulation')]
            result['controller_all_subscriptions']=node.get_subscriber_names_and_types_by_node('controller_server','/')
            result['simulation_odom_subscribers']=[(i.node_name,i.node_namespace) for i in node.get_subscriptions_info_by_topic('/odometry/simulation')]
            result['controller_odom_gate']=bool(result['controller_odom_topic']=='/odometry/simulation' and
                result['controller_odom_subscriptions']==['/odometry/simulation'] and
                result['simulation_odom_publishers']==['new_car_simulation_truth_tf'])
        if not args.no_nav2 and action.server_is_ready() and result.get('bt_navigator_active'):
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=-1.;goal.pose.pose.position.y=-3.
            goal.pose.pose.orientation.w=1.
            sent=action.send_goal_async(goal);deadline=time.monotonic()+10
            while not sent.done() and time.monotonic()<deadline: spin(.1)
            if sent.done():
                handle=sent.result();result['nav2_goal_accepted']=handle.accepted
                if handle.accepted:
                    future=handle.get_result_async();deadline=time.monotonic()+35
                    while not future.done() and time.monotonic()<deadline: spin(.1)
                    result['nav2_status']=future.result().status if future.done() else 'timeout'
                    if not future.done(): handle.cancel_goal_async();spin(.5)
        if result.get('nav2_status')==4:
            behavior=ActionClient(node,Spin,'/spin');deadline=time.monotonic()+5
            while not behavior.server_is_ready() and time.monotonic()<deadline:spin(.1)
            if behavior.server_is_ready():
                goal=Spin.Goal();goal.target_yaw=.3;goal.time_allowance=Duration(sec=8)
                before=len(final_commands);sent=behavior.send_goal_async(goal);deadline=time.monotonic()+5
                while not sent.done() and time.monotonic()<deadline:spin(.1)
                if sent.done() and sent.result().accepted:
                    done=sent.result().get_result_async();deadline=time.monotonic()+10
                    while not done.done() and time.monotonic()<deadline:spin(.1)
                    result['behavior_status']=done.result().status if done.done() else 'timeout'
                    if not done.done():sent.result().cancel_goal_async();spin(.5)
                result['behavior_smoother_max_wz']=max([abs(v) for v in final_commands[before:]],default=0.)
            result['behavior_gate']=result.get('behavior_status')==4 and result.get('behavior_smoother_max_wz',0)>.1
        if poses:
            pos=poses[-1].pose.pose.position;result['last_base_xyz']=[pos.x,pos.y,pos.z]
            result['goal_distance_m']=float(np.hypot(pos.x+1,pos.y+3))
        result['routing']={t:[i.node_name for i in node.get_publishers_info_by_topic(t)] for t in ('/cmd_vel_nav','/cmd_vel')}
        result['route_gate']=sorted(result['routing']['/cmd_vel_nav'])==['behavior_server']*4+['controller_server'] and result['routing']['/cmd_vel']==['velocity_smoother']
        owners=json.loads((args.output/'tf_owners.json').read_text()) if (args.output/'tf_owners.json').exists() else {}
        result['tf_owner_gids']=owners
        result['tf_owner_gate']=all(len(owners.get(e,[]))==1 for e in ('map>odom','odom>base_link'))
        result['node_graph']=[list(x) for x in node.get_node_names_and_namespaces()]
        result['no_serial_or_mission']=not any('serial' in name or 'mission' in name for name,ns in result['node_graph'])
        result['nav2_gate']=bool(result.get('nav2_status')==4 and result.get('goal_distance_m',100)<.2 and result.get('controller_odom_gate') and result['route_gate'] and result['tf_owner_gate'] and result['no_serial_or_mission'] and result.get('behavior_gate'))
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:
        node.destroy_node();rclpy.try_shutdown();child.send_signal(signal.SIGINT)
        try:child.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        audit.terminate()
        try:audit.wait(timeout=3)
        except subprocess.TimeoutExpired:audit.kill();audit.wait()
        log.close()
    if not args.no_nav2 and not result.get('nav2_gate'):raise SystemExit(2)


if __name__=='__main__': main()
