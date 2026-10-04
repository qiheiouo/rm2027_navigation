#!/usr/bin/env python3
"""Real ROS/Nav2 actions, deterministic ZOH kinematic test plant.

This fixture deliberately uses EMPTY synthetic prediction arrays; it verifies
transport/lifecycle/switch/failure engineering, not dynamic navigation success.
No Gazebo truth, tracker, serial hardware, or MPPI superiority evidence.
"""
import argparse
import json
import math
import os
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from lifecycle_msgs.srv import GetState
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray, DynamicObstaclePrediction
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

MPPI, MPC = 'FollowPathMPPI','FollowPathTemporalMPC'


class Harness(Node):
    def __init__(self, output):
        super().__init__('temporal_mpc_ros_harness')
        self.output=output
        output.mkdir(parents=True,exist_ok=True)
        self.trace=(output/'events.jsonl').open('w')
        self.x=self.y=self.yaw=0.
        self.command=np.zeros(3)
        self.last_step=time.monotonic()
        self.commands=[];self.health=[];self.selected=[];self.proposals=[];self.solver=[]
        self.stage='startup';self.prediction_mode='valid'
        self.tf=TransformBroadcaster(self)
        self.odom=self.create_publisher(Odometry,'odometry/lio',10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'map',qos)
        self.pred=self.create_publisher(DynamicObstaclePredictionArray,'dynamic_obstacle_predictions',1)
        self.request=self.create_publisher(String,'temporal_mpc/request_controller',10)
        self.fault=self.create_publisher(String,'temporal_mpc/test_fault',10)
        self.speed_limit=self.create_publisher(SpeedLimit,'speed_limit',10)
        self.create_subscription(Twist,'cmd_vel',self.on_command,10)
        self.create_subscription(String,'temporal_mpc/health',lambda p:self.on_json('health',p,self.health),10)
        self.create_subscription(String,'temporal_mpc/solver_diagnostic',lambda p:self.on_json('solver',p,self.solver),10)
        self.create_subscription(String,'controller_selector',self.on_selection,qos)
        self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.follow=ActionClient(self,FollowPath,'follow_path')
        self.create_timer(.01,self.step)
        self.create_timer(.05,self.publish_prediction)
        self.create_timer(.02,self.graph_watch)
        self.publish_map()
        self.graph_violation=False
        self.publisher_names=set()
        self.publisher_gids=set()
        self.latest_path=None
        self.create_subscription(RosPath,"plan",lambda p:setattr(self,"latest_path",p),1)

    def event(self,kind,data):
        record={'wall_s':time.monotonic(),'ros_ns':self.get_clock().now().nanoseconds,'stage':self.stage,'kind':kind,**data}
        self.trace.write(json.dumps(record,allow_nan=False)+'\n');self.trace.flush()
        return record

    def on_json(self,kind,p,storage):
        storage.append(self.event(kind,json.loads(p.data)))

    def on_selection(self,p):
        self.selected.append(self.event('selection',{'id':p.data}))

    def on_command(self,p):
        self.command=np.array([p.linear.x,p.linear.y,p.angular.z])
        self.commands.append(self.event('command',{'v':self.command.tolist(),'x':self.x,'y':self.y}))

    def graph_watch(self):
        info=self.get_publishers_info_by_topic('/cmd_vel')
        names={p.node_name for p in info}
        self.publisher_names.update(names-{'_NODE_NAME_UNKNOWN_'})
        self.publisher_gids.update(bytes(p.endpoint_gid).hex() for p in info)
        if len(info)>1 or len(self.publisher_gids)>1 or names-{'controller_server','_NODE_NAME_UNKNOWN_'}:
            self.graph_violation=True

    def step(self):
        now=time.monotonic();dt=now-self.last_step;self.last_step=now
        # Wall clock continues while worker/Nav2 are calculating. No paused sim.
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        self.x+=dt*(c*self.command[0]-s*self.command[1]);self.y+=dt*(s*self.command[0]+c*self.command[1]);self.yaw+=dt*self.command[2]
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='base_link'
        t.transform.translation.x=self.x;t.transform.translation.y=self.y
        t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(t)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='map';odom.child_frame_id='base_link'
        odom.pose.pose.position.x=self.x;odom.pose.pose.position.y=self.y;odom.pose.pose.orientation=t.transform.rotation
        odom.twist.twist.linear.x=float(self.command[0]);odom.twist.twist.linear.y=float(self.command[1]);odom.twist.twist.angular.z=float(self.command[2]);self.odom.publish(odom)

    def publish_map(self, blocked=False):
        p=OccupancyGrid();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
        p.info.resolution=.05;p.info.width=500;p.info.height=140;p.info.origin.position.x=-1.;p.info.origin.position.y=-3.5;p.info.origin.orientation.w=1.
        costs=np.zeros((140,500),np.int8);costs[[0,-1],:]=100;costs[:,[0,-1]]=100
        if blocked:
            ix=int((self.x+1.)/.05);iy=int((self.y+3.5)/.05);costs[iy,ix]=100
        p.data=costs.ravel().tolist();self.map_pub.publish(p)
        self.event('map',{'blocked':blocked,'revision':p.header.stamp.sec*10**9+p.header.stamp.nanosec})

    def publish_prediction(self):
        if self.prediction_mode=='silent':return
        p=DynamicObstaclePredictionArray();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.processing_stamp=p.header.stamp
        p.schema=p.SCHEMA_OBSERVATION_ANCHOR;p.authority=p.AUTHORITY_SHADOW_ONLY;p.prediction_dt=.1;p.prediction_steps=15;p.complete=self.prediction_mode!='incomplete';p.total_track_count=0
        if self.prediction_mode=='wrong_frame':p.header.frame_id='odom'
        if self.prediction_mode=='wrong_schema':p.schema=p.SCHEMA
        if self.prediction_mode=='authority':p.authority=''
        if self.prediction_mode=='future_stamp':p.header.stamp.sec+=1;p.processing_stamp=p.header.stamp
        if self.prediction_mode in ('new_obstacle','over_budget','duplicate_id'):
            count=65 if self.prediction_mode=='over_budget' else (2 if self.prediction_mode=='duplicate_id' else 1)
            tracks=[]
            for k in range(count):
                t=DynamicObstaclePrediction();t.track_id=1 if self.prediction_mode=='duplicate_id' else k+1
                t.state=2;t.last_observation_stamp=p.header.stamp;t.observation_count=1
                t.position.x=self.x if self.prediction_mode=='new_obstacle' else 40.
                t.position.y=self.y if self.prediction_mode=='new_obstacle' else 40.
                t.size.x=.45;t.size.y=.55;t.size.z=.5
                t.prediction=[Point(x=t.position.x,y=t.position.y) for _ in range(15)]
                tracks.append(t)
            p.tracks=tracks;p.total_track_count=count
        self.pred.publish(p)

    def pump(self,seconds,predicate=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.01)
            if predicate and predicate():return True
        return bool(predicate and predicate())

    def wait_ready(self,timeout=6.):
        return self.pump(timeout,lambda: bool(self.health and self.health[-1]['ready']))

    def select(self,controller):
        self.request.publish(String(data=controller))
        return self.pump(2.,lambda: bool(self.selected and self.selected[-1]['id']==controller))

    def arm_mpc(self):
        # Require an accepted compute callback while MPC is still selected.
        end=time.monotonic()+6.
        while time.monotonic()<end:
            if not self.wait_ready(1.):continue
            self.select(MPC)
            marker=time.monotonic()
            if self.pump(.3,lambda: self.selected[-1]['id']==MPC and any(
                    p['wall_s']>=marker and p['executed'] and p['ready'] for p in self.health)):
                self.pump(.2)
                if self.selected[-1]['id']==MPC and self.health[-1]['ready']:
                    return True
        return False

    def lifecycle_active(self):
        clients=[]
        for name in ('controller_server','planner_server','bt_navigator'):
            c=self.create_client(GetState,f'/{name}/get_state');clients.append(c)
            if not c.wait_for_service(timeout_sec=3.):return False
            future=c.call_async(GetState.Request())
            if not self.pump(2.,future.done) or future.result().current_state.id!=3:return False
        return True

    def run(self):
        checks={};cases=[]
        self.pump(3.)
        checks['lifecycle_active']=self.lifecycle_active()
        if not self.action.wait_for_server(timeout_sec=3.):raise RuntimeError('NavigateToPose unavailable')
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=20.;goal.pose.pose.orientation.w=1.
        future=self.action.send_goal_async(goal)
        if not self.pump(3.,future.done):raise RuntimeError('goal acceptance timeout')
        handle=future.result();checks['navigation_goal_accepted']=handle.accepted
        result_future=handle.get_result_async()
        self.stage='warm_start_MPPI';self.pump(1.)
        checks['ready_while_MPPI']=self.wait_ready()
        self.stage='explicit_MPC';checks['MPPI_to_MPC']=self.arm_mpc()
        self.pump(.6)
        checks['MPC_computes']=any(p['stage']=='explicit_MPC' and p['executed'] and p['ready'] for p in self.health)
        self.stage='explicit_MPPI';checks['MPC_to_MPPI']=self.select(MPPI);self.pump(.3)
        # Keep goal live in a corridor long enough for fault tests: reset plant
        # is forbidden; velocities remain continuous and all actual motion recorded.
        for kind in ('silent','timeout','infeasible','exception','nan','incomplete','wrong_frame','wrong_schema','prediction_silent','authority','future_stamp','duplicate_id','over_budget','new_obstacle'):
            self.stage='rearm_'+kind
            self.prediction_mode='valid';self.fault.publish(String(data=''))
            selected=self.arm_mpc()
            ready=selected and self.selected[-1]['id']==MPC
            self.stage='fault_'+kind;start=time.monotonic();before=len(self.commands)
            if kind in ('incomplete','wrong_frame','wrong_schema','authority','future_stamp','duplicate_id','over_budget','new_obstacle'):self.prediction_mode=kind
            elif kind=='prediction_silent':self.prediction_mode='silent'
            else:self.fault.publish(String(data=kind))
            switched=self.pump(1.5,lambda: bool(self.selected and self.selected[-1]['id']==MPPI))
            latency=time.monotonic()-start
            self.pump(.18)
            fault_health=[p for p in self.health if p['stage']==self.stage and p['executed']]
            case={'fault':kind,'rearmed':ready and selected,'fallback_to_MPPI':switched,
                  'fallback_latency_s':latency,'commands_during_fault':len(self.commands)-before,
                  'native_brake_seen':any(not p['ready'] for p in fault_health),
                  'degraded_health_seen':any(p['stage']==self.stage and not p['ready'] for p in self.health),
                  'selector_transition_after_fault':any(p['stage']==self.stage and p['id']==MPPI for p in self.selected)}
            cases.append(case);self.event('fault_result',case)
        self.prediction_mode='valid';self.fault.publish(String(data=''))
        self.stage='final_cancel';cancel=handle.cancel_goal_async();checks['cancel_reply']=self.pump(3.,cancel.done)
        checks['action_result_received']=self.pump(3.,result_future.done)
        statuses={'navigation_result':result_future.result().status if result_future.done() else None}
        self.pump(.2)
        # Direct FollowPath client holds MPC selected even after the supervisor
        # requests MPPI. This isolates the native brake before caller handoff.
        self.stage='direct_MPC_before_brake'
        self.wait_ready()
        direct=FollowPath.Goal();direct.path=self.latest_path;direct.controller_id=MPC;direct.goal_checker_id='general_goal_checker'
        sent=self.follow.send_goal_async(direct)
        checks['direct_goal_accepted']=self.pump(2.,sent.done) and sent.result().accepted
        direct_handle=sent.result() if sent.done() else None
        checks['direct_MPC_accepted_compute']=self.pump(.5,lambda:any(p['stage']==self.stage and p['executed'] and p['ready'] for p in self.health))
        self.stage='direct_speed_limit'
        self.speed_limit.publish(SpeedLimit(percentage=True,speed_limit=50.))
        self.pump(.3)
        limited=[p for p in self.commands if p['stage']==self.stage]
        checks['speed_limit_bounded_fallback']=bool(limited and max(abs(v) for v in limited[-1]['v'])<1e-6)
        self.speed_limit.publish(SpeedLimit(percentage=False,speed_limit=0.))
        self.stage='direct_MPC_recovered'
        checks['speed_limit_reset_recovery']=self.pump(1.,lambda:any(p['stage']==self.stage and p['executed'] and p['ready'] for p in self.health))
        self.stage='direct_worker_killed_brake'
        os.kill(int(os.environ['MPC_WORKER_PID']),signal.SIGKILL)
        self.event('worker_killed',{'signal':'SIGKILL'})
        self.pump(1.)
        h=[p for p in self.health if p['stage']==self.stage and p['executed'] and not p['ready']]
        commands=[p for p in self.commands if p['stage']==self.stage]
        checks['native_brake_after_worker_process_exit']=bool(h and len(commands)>=15 and max(abs(v) for v in commands[-1]['v'])<1e-6)
        if direct_handle:
            cancelled=direct_handle.cancel_goal_async();self.pump(2.,cancelled.done)
        self.stage='finished';self.fault.publish(String(data=''));self.pump(.1)
        checks['single_command_publisher']=not self.graph_violation and self.publisher_names=={'controller_server'} and len(self.publisher_gids)==1
        velocities=np.array([p['v'] for p in self.commands]);checks['finite_bounded_commands']=bool(np.isfinite(velocities).all() and np.max(np.abs(velocities[:,0]))<=.80001 and np.max(np.abs(velocities[:,1]))<=.50001 and np.max(np.abs(velocities[:,2]))<1e-6)
        times=np.array([p['wall_s'] for p in self.commands]);gaps=np.diff(times)
        compute=[p['elapsed_s'] for p in self.health if p['executed']]
        # Explicit action cancellation separates active control sessions. Keep
        # the overall gap visible, and gate all intervals within each live span.
        primary=[p['wall_s'] for p in self.commands if not p['stage'].startswith('direct_') and p['stage'] not in ('final_cancel','finished')]
        direct=[p['wall_s'] for p in self.commands if p['stage'].startswith('direct_')]
        active_gaps=np.r_[np.diff(primary),np.diff(direct)]
        checks['command_continuity_20Hz']=bool(len(active_gaps) and max(active_gaps)<.075)
        checks['native_compute_budget']=bool(compute and max(compute)<.01)
        summary={'scope':'Humble real Nav2/BT/DDS with synthetic empty predictions, static map, ZOH test plant; not Gazebo or tracker acceptance',
                 'checks':checks,'fault_cases':cases,'statuses':statuses,'command_count':len(self.commands),
                 'publisher_names':sorted(self.publisher_names),'publisher_gids':sorted(self.publisher_gids),'max_command_gap_s':float(max(gaps)) if len(gaps) else None,'max_active_command_gap_s':float(max(active_gaps)) if len(active_gaps) else None,
                 'native_compute_p95_s':float(np.percentile(compute,95)) if compute else None,
                 'native_compute_max_s':float(max(compute)) if compute else None,
                 'solver_p95_s':float(np.percentile([p['elapsed_s'] for p in self.solver],95)) if self.solver else None,
                 'all_engineering_gates_pass':all(checks.values()) and all(c['rearmed'] and c['fallback_to_MPPI'] and c['selector_transition_after_fault'] and c['degraded_health_seen'] and c['commands_during_fault']>0 for c in cases),
                 'dynamic_navigation_acceptance':False}
        (self.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');self.trace.close()
        print(json.dumps(summary,indent=2),flush=True)
        if not summary['all_engineering_gates_pass']:raise RuntimeError('integration engineering gate failed')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=Harness(args.output)
    try:node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
