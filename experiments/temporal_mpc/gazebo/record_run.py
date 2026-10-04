#!/usr/bin/env python3
"""Actual Gazebo observation recorder and unchanged NavigateToPose action client."""
import argparse
import base64
from collections import Counter
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMessage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from rm_temporal_mpc_msgs.msg import StateRequest, Proposal, Plan
from nav2_msgs.action import NavigateToPose
from ros_gz_interfaces.msg import Contacts


def clean(value):
    if isinstance(value,float) and not math.isfinite(value): return str(value)
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    return value


class Recorder(Node):
    def __init__(self,output,mode):
        super().__init__("temporal_gazebo_recorder",parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.output=Path(output); self.stream=(self.output/"events.jsonl").open("w")
        self.fixture=json.loads((self.output/'scene/scene.json').read_text())
        self.counts=Counter(); self.mode=mode; self.latest={}; self.goal=None; self.result=None
        self.recorded_static_edges=set()
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.request=self.create_publisher(String,"temporal_mpc/request_controller",10)
        self.client=ActionClient(self,NavigateToPose,"navigate_to_pose")
        transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        static_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        inputs=[("/scan",LaserScan,qos_profile_sensor_data),
                ("/dynamic_obstacle_predictions",DynamicObstaclePredictionArray,10),
                ("/odometry/lio",Odometry,50),("/simulation/ground_truth/odom",Odometry,50),
                ("/cmd_vel",Twist,50),("/nav2/cmd_vel",Twist,50),
                ("/temporal_mpc/smoothed_cmd_vel",Twist,50),
                ("/temporal_mpc/execution_health",String,100),
                ("/simulation/chassis/cmd_vel",Twist,50),
                ("/simulation/oracle/rm_sentry_2027",TFMessage,100),
                ("/simulation/oracle/world",TFMessage,100),
                ("/simulation/oracle/contacts",Contacts,100),
                ("/simulation/oracle/moving_obstacle",TFMessage,100),
                ("/tf",TFMessage,100),("/tf_static",TFMessage,static_qos),
                ("/map",OccupancyGrid,transient),("/plan",PathMessage,transient),
                ("/controller_selector",String,transient),
                ("/temporal_mpc/health",String,100),
                ("/temporal_mpc/solver_diagnostic",String,100),
                ("/temporal_mpc/state",StateRequest,100),("/temporal_mpc/proposal",Proposal,100),
                ("/temporal_mpc/plan",Plan,transient),
                ("/simulation/moving_obstacle/target",Float64,100)]
        self.subscriptions_kept=[]
        for topic,kind,qos in inputs:
            self.subscriptions_kept.append(self.create_subscription(kind,topic,lambda m,t=topic:self.record(t,m),qos))
        self.event("registration",dict(mode=mode,goal=self.fixture['goal'],phase=10.,timeout_sim_s=40.,
                   fixture_profile=self.fixture['fixture_profile'],strategy=os.environ.get('TEMPORAL_MPC_STRATEGY','single')))

    def event(self,topic,data,**extra):
        document=dict(topic=topic,receipt_monotonic_ns=time.monotonic_ns(),
                      receipt_sim_ns=self.get_clock().now().nanoseconds,data=clean(data),**extra)
        self.stream.write(json.dumps(document,separators=(",",":"),allow_nan=False)+"\n")
        self.counts[topic]+=1

    def record(self,topic,message):
        self.latest[topic]=message
        self.event(topic,message_to_ordereddict(message),cdr_b64=base64.b64encode(serialize_message(message)).decode())
        if topic=="/tf_static":
            self.recorded_static_edges.update((t.header.frame_id,t.child_frame_id) for t in message.transforms)
        if topic=="/dynamic_obstacle_predictions":
            try:
                tf=self.buffer.lookup_transform("map","sim_lidar_link",Time.from_msg(message.header.stamp))
                self.event("source_tf",dict(source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                    available=True,transform=message_to_ordereddict(tf)))
            except Exception as error:
                self.event("source_tf",dict(source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                    available=False,reason=str(error)))

    def graph(self):
        evidence={}
        for topic in ("/cmd_vel","/nav2/cmd_vel","/temporal_mpc/smoothed_cmd_vel","/simulation/chassis/cmd_vel","/dynamic_obstacle_predictions","/tf","/tf_static"):
            evidence[topic]=[dict(node_name=i.node_name,node_namespace=i.node_namespace,topic_type=i.topic_type,
                                 endpoint_gid=list(i.endpoint_gid)) for i in self.get_publishers_info_by_topic(topic)]
        self.event("graph_publishers",evidence)

    def ready(self):
        required=("/scan","/dynamic_obstacle_predictions","/odometry/lio",
                  "/simulation/oracle/rm_sentry_2027","/simulation/oracle/moving_obstacle","/map")
        return (self.static_chain_recorded() and
                all(self.counts[k]>2 if k!="/map" else self.counts[k]>0 for k in required)
                and self.client.server_is_ready())

    def static_chain_recorded(self):
        # Check the raw evidence subscription, not the separate TF listener.
        return {('map','odom'),('base_link','sim_lidar_link')}<=self.recorded_static_edges

    def start_goal(self):
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id="map"
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=self.fixture['goal']; goal.pose.pose.orientation.w=1.
        self.event("goal_sent",message_to_ordereddict(goal))
        self.goal_future=self.client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_accepted)

    def goal_accepted(self,future):
        self.goal=future.result()
        self.event("goal_accepted",dict(accepted=self.goal.accepted))
        if self.goal.accepted:
            self.result_future=self.goal.get_result_async(); self.result_future.add_done_callback(self.goal_result)
        else: self.result=dict(status="rejected")

    def goal_result(self,future):
        result=future.result(); self.result=dict(status=result.status,result=message_to_ordereddict(result.result))
        self.event("goal_result",self.result)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--output",required=True)
    parser.add_argument("--mode",choices=["shadow","mpc"],default="shadow")
    args=parser.parse_args(); rclpy.init(); node=Recorder(args.output,args.mode)
    wall_start=time.monotonic(); phase=None; goal_epoch=None; arming_attempted=False; stop_reason="wall timeout"
    try:
        while time.monotonic()-wall_start<130:
            rclpy.spin_once(node,timeout_sec=.01)
            now=node.get_clock().now().nanoseconds/1e9
            if phase is None and node.ready() and now<9.5:
                phase=10.
                node.event("ready",dict(now=now,start_phase=phase));node.graph()
            if phase is None and now>=9.5:
                stop_reason="input startup gate before fixed phase failed"; break
            if phase is not None and goal_epoch is None and now>=phase:
                goal_epoch=now; node.start_goal()
            if args.mode=="mpc" and goal_epoch is not None and not arming_attempted:
                health=node.latest.get("/temporal_mpc/health")
                if health is not None:
                    data=json.loads(health.data)
                    if data["ready"] and not data["fallback_requested"]:
                        node.request.publish(String(data="FollowPathTemporalMPC")); arming_attempted=True
                        node.event("mpc_requested",dict(health=data))
            if node.result is not None:
                stop_reason="action terminated"; break
            if goal_epoch is not None and now-goal_epoch>=40:
                stop_reason="sim timeout"; break
        if node.goal is not None and node.goal.accepted and node.result is None:
            cancel=node.goal.cancel_goal_async()
            limit=time.monotonic()+2
            while not cancel.done() and time.monotonic()<limit: rclpy.spin_once(node,timeout_sec=.02)
            node.event("cancel",dict(completed=cancel.done()))
        node.graph()
        final=node.get_clock().now().nanoseconds/1e9
        # Record the final cancellation stop command before teardown.
        limit=time.monotonic()+.4
        while time.monotonic()<limit: rclpy.spin_once(node,timeout_sec=.01)
        summary=dict(mode=args.mode,stop_reason=stop_reason,goal_epoch_s=goal_epoch,final_sim_s=final,
                     wall_s=time.monotonic()-wall_start,result=node.result,mpc_requested=arming_attempted,
                     counts=dict(node.counts),physical_acceptance=False,input_gate_pending=True)
        (Path(args.output)/"run_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
        print(json.dumps(summary,indent=2),flush=True)
    finally:
        node.stream.close(); node.destroy_node(); rclpy.try_shutdown()

if __name__=="__main__": main()
