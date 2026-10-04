#!/usr/bin/env python3
"""Synthetic ROS boundary faults only; not physical/detection acceptance."""
import argparse
import base64
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return str(value)
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


class Harness(Node):
    def __init__(self,root):
        super().__init__('execution_guard_fault_harness');self.root=root
        self.stream=(root/'events.jsonl').open('w');self.start=time.monotonic();self.last=self.start
        self.stage='startup';self.mode='';self.x=0.;self.velocity=0.;self.commands=[];self.health=[]
        self.clock=self.create_publisher(Clock,'clock',10)
        self.odom=self.create_publisher(Odometry,'/odometry/lio',10)
        self.pred=self.create_publisher(DynamicObstaclePredictionArray,'dynamic_obstacle_predictions',10)
        self.command=self.create_publisher(Twist,'temporal_mpc/smoothed_cmd_vel',10)
        self.fault=self.create_publisher(String,'temporal_mpc/test_guard_fault',10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map=self.create_publisher(OccupancyGrid,'map',qos)
        self.dynamic_tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
        self.static_tf.sendTransform(t)
        self.create_subscription(Twist,'cmd_vel',self.on_command,100)
        self.create_subscription(String,'temporal_mpc/execution_health',self.on_health,100)
        self.publish_map()
        self.timer=self.create_timer(.02,self.inputs)

    def record(self,topic,message):
        e=dict(topic=topic,wall_ns=time.monotonic_ns(),stage=self.stage,data=clean(message_to_ordereddict(message)),
               cdr_b64=base64.b64encode(serialize_message(message)).decode())
        self.stream.write(json.dumps(e,separators=(',',':'),allow_nan=False)+'\n')

    def on_command(self,m):
        self.record('/cmd_vel',m);self.commands.append((time.monotonic(),self.stage,m.linear.x,m.linear.y,m.angular.z))
        self.velocity=m.linear.x

    def on_health(self,m):
        self.record('/temporal_mpc/execution_health',m);self.health.append((time.monotonic(),self.stage,json.loads(m.data)))

    def stamp(self,seconds):
        from builtin_interfaces.msg import Time
        ns=round(seconds*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)

    def publish_map(self):
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.stamp(0.)
        m.info.width=200;m.info.height=100;m.info.resolution=.1;m.info.origin.position.x=-5.;m.info.origin.position.y=-5.
        m.info.origin.orientation.w=1.;m.data=[0]*20000
        self.map.publish(m);self.record('/map',m)

    def inputs(self):
        now=time.monotonic();self.x+=(now-self.last)*self.velocity;self.last=now;t=1.+now-self.start
        stamp=self.stamp(t);clock=Clock(clock=stamp);self.clock.publish(clock)
        if self.mode!='odom_silent':
            m=Odometry();m.header.frame_id='odom';m.header.stamp=self.stamp(t+.3 if self.mode=='future_odom' else t)
            m.child_frame_id='base_link';m.pose.pose.position.x=self.x;m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=self.velocity
            if self.mode=='nonplanar':m.pose.pose.orientation.x=.01
            self.odom.publish(m);self.record('/odometry/lio',m)
            tf=TransformStamped();tf.header=m.header;tf.child_frame_id='base_link';tf.transform.translation.x=self.x
            tf.transform.rotation=m.pose.pose.orientation;self.dynamic_tf.sendTransform(tf)
        if self.mode!='prediction_silent':
            p=DynamicObstaclePredictionArray();p.header.frame_id='map';p.header.stamp=stamp;p.processing_stamp=stamp
            p.schema='rm_dynamic_obstacle_predictions/v2_observation_anchor';p.authority='shadow_only'
            p.complete=self.mode!='incomplete';p.prediction_steps=15;p.prediction_dt=.1
            self.pred.publish(p);self.record('/dynamic_obstacle_predictions',p)
        if self.mode!='command_silent':
            cmd=Twist();cmd.linear.x=float('nan') if self.mode=='nan' else .4
            self.command.publish(cmd);self.record('/temporal_mpc/smoothed_cmd_vel',cmd)

    def pump(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)

    def run(self):
        cases=[];self.pump(2.)
        for kind in ('exception','nan','command_silent','odom_silent','prediction_silent','incomplete','future_odom','nonplanar'):
            self.mode='';self.fault.publish(String(data=''));self.stage='recover_'+kind;self.pump(1.)
            recovery=any(s==self.stage and d['model_certified'] and d['status']=='pass' for _,s,d in self.health)
            warm=[v for v in self.commands if v[1]==self.stage]
            moving=bool(warm and warm[-1][2]>.35)
            self.stage='fault_'+kind;self.mode='' if kind=='exception' else kind
            msg=String(data='exception' if kind=='exception' else '');self.fault.publish(msg);self.record('/temporal_mpc/test_guard_fault',msg)
            self.pump(1.)
            commands=[v for v in self.commands if v[1]==self.stage];health=[d for _,s,d in self.health if s==self.stage]
            case=dict(fault=kind,recovered_before_fault=recovery,moving_before_fault=moving,command_count=len(commands),
                      degraded_seen=any(not d['model_certified'] or d['status']!='pass' for d in health),
                      stopped_at_end=bool(commands and max(abs(v) for v in commands[-1][2:])<1e-6),
                      continuous_output=bool(len(commands)>=10 and max(b[0]-a[0] for a,b in zip(commands,commands[1:]))<=.075))
            cases.append(case)
        self.mode='';self.fault.publish(String(data=''));self.stage='final_recovery';self.pump(1.)
        gaps=[b[0]-a[0] for a,b in zip(self.commands,self.commands[1:])]
        slew=[abs(b[2]-a[2]) for a,b in zip(self.commands,self.commands[1:])]
        def maximum(key):
            values=[d[key] for _,_,d in self.health if d.get(key) is not None]
            return max(values) if values else None
        result=dict(cases=cases,command_count=len(self.commands),max_command_gap_s=max(gaps),max_observed_vx_change=max(slew),
                    max_producer_start_interval_s=maximum('producer_start_interval_s'),
                    max_producer_output_interval_s=maximum('producer_output_interval_s'),
                    max_tick_to_output_s=maximum('tick_to_output_s'),
                    max_tick_to_output_cpu_s=maximum('tick_to_output_cpu_s'),
                    max_nominal_lateness_s=maximum('nominal_lateness_s'),
                    nominal_missed_slots=sum(d.get('nominal_missed_slots',0) for _,_,d in self.health),
                    health_count=len(self.health),final_recovery=any(s==self.stage and d['model_certified'] and d['status']=='pass' for _,s,d in self.health),
                    all_pass=all(all(c[k] for k in ('recovered_before_fault','moving_before_fault','degraded_seen','stopped_at_end','continuous_output')) for c in cases)
                             and max(slew)<=.05000000001 and max(gaps)<=.075
                             and any(s==self.stage and d['model_certified'] for _,s,d in self.health),
                    scope='Synthetic empty public predictions and held-velocity plant; engineering faults only, not physical safety')
        (self.root/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
        return result['all_pass']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');args=p.parse_args();root=Path(args.output)
    if (root/'events.jsonl').exists():raise SystemExit('refuse overwrite')
    root.mkdir(parents=True,exist_ok=True);rclpy.init();node=Harness(root)
    try:passed=node.run()
    finally:node.stream.close();node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)
