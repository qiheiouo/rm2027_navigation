#!/usr/bin/env python3
"""Actual Humble DDS with controlled ROS clock and synthetic state inputs."""
import argparse
import base64
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rm_temporal_mpc_msgs.msg import StateRequest, Plan, Proposal
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from worker_node import Worker


def stamp(ns):return Time(sec=ns//10**9,nanosec=ns%10**9)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output');parser.add_argument('frontend');a=parser.parse_args();root=Path(a.output)
    if root.exists():raise SystemExit('refuse fixture overwrite')
    root.mkdir(parents=True);stream=(root/'events.jsonl').open('w')
    rclpy.init(args=['--ros-args','-p','frontend:='+a.frontend,'-p','strategy:=portfolio','-p','use_sim_time:=true'])
    worker=Worker();driver=Node('worker_clock_fixture');timings=[];proposals=[]
    def record(topic,m):
        data=message_to_ordereddict(m)
        if topic=='/temporal_mpc/worker_timing':timings.append(json.loads(m.data))
        if topic=='/temporal_mpc/proposal':proposals.append(m)
        stream.write(json.dumps(dict(topic=topic,data=data,receipt_monotonic_ns=time.monotonic_ns(),
                         receipt_sim_ns=driver.get_clock().now().nanoseconds,
                         cdr_b64=base64.b64encode(serialize_message(m)).decode()),allow_nan=False)+'\n')
    kept=[driver.create_subscription(String,'temporal_mpc/worker_timing',lambda m:record('/temporal_mpc/worker_timing',m),100),
          driver.create_subscription(Proposal,'temporal_mpc/proposal',lambda m:record('/temporal_mpc/proposal',m),100)]
    clock_pub=driver.create_publisher(Clock,'clock',10);request_pub=driver.create_publisher(StateRequest,'temporal_mpc/state',10)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    map_pub=driver.create_publisher(OccupancyGrid,'map',qos);plan_pub=driver.create_publisher(Plan,'temporal_mpc/plan',qos)
    prediction_pub=driver.create_publisher(DynamicObstaclePredictionArray,'dynamic_obstacle_predictions',10)
    def pump(predicate,timeout=3.):
        limit=time.monotonic()+timeout
        while time.monotonic()<limit:
            rclpy.spin_once(worker,timeout_sec=.002);rclpy.spin_once(driver,timeout_sec=.002)
            if predicate():return True
        return False
    def clock(ns):
        clock_pub.publish(Clock(clock=stamp(ns)))
        if not pump(lambda:worker.get_clock().now().nanoseconds==ns):raise RuntimeError('controlled clock delivery')
    def request(ns,frame='map'):
        m=StateRequest();m.header.stamp=stamp(ns);m.header.frame_id=frame;m.generation=7;m.pose.orientation.w=1.
        start=len(timings);request_pub.publish(m)
        if not pump(lambda:any(t['kind']=='request' for t in timings[start:])):raise RuntimeError('request callback absent')
        return next(t for t in timings[start:] if t['kind']=='request')
    checks={}
    if not pump(lambda:request_pub.get_subscription_count()>0 and clock_pub.get_subscription_count()>=2):raise RuntimeError('DDS discovery')
    for name,now,epoch,frame in [('frame',2_000_000_000,2_000_000_000,'odom'),
                               ('future',3_000_000_000,3_002_000_000,'map'),
                               ('stale',4_100_000_001,4_000_000_000,'map'),
                               ('current',5_100_000_000,5_000_000_000,'map')]:
        clock(now);t=request(epoch,frame);checks[name]=bool(t['request_clock_status']==name and t['request_signed_age_ns']==now-epoch
                                                    and 'solver_start_monotonic_ns' not in t and t['proposal_published'])
    t=request(5_000_000_000);checks['duplicate_recorded_without_proposal']=t['disposition']=='duplicate' and not t['proposal_published']
    worker.fault='silent';t=request(6_000_000_000);worker.fault=''
    checks['silent_recorded_without_proposal']=t['disposition']=='injected_silent' and not t['proposal_published']
    clock(7_010_000_000)
    m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=stamp(7_000_000_000);m.info.resolution=.05
    m.info.width=m.info.height=80;m.info.origin.position.x=m.info.origin.position.y=-2.;m.info.origin.orientation.w=1.;m.data=[0]*6400
    path=PathMessage();path.header.frame_id='map'
    for x in (0.,1.):
        pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.orientation.w=1.;path.poses.append(pose)
    plan=Plan(generation=7,path=path);map_pub.publish(m);plan_pub.publish(plan)
    if not pump(lambda:worker.route is not None):raise RuntimeError('real T-DT static route absent')
    p=DynamicObstaclePredictionArray();p.header.frame_id='map';p.header.stamp=stamp(7_001_000_000);p.processing_stamp=p.header.stamp
    p.schema='rm_dynamic_obstacle_predictions/v2_observation_anchor';p.authority='shadow_only';p.complete=True;p.prediction_dt=.1;p.prediction_steps=15
    prediction_pub.publish(p)
    if not pump(lambda:worker.snapshot is not None):raise RuntimeError('public empty prediction absent')
    t=request(7_000_000_000);checks['no_causal_snapshot_recorded_without_proposal']=t['disposition']=='no_causal_snapshot' and not t['proposal_published']
    clock(7_020_000_000);t=request(7_020_000_000)
    checks['current_qp_has_complete_publish_timing']=bool(t['proposal_feasible'] and
        t['callback_start_monotonic_ns']<=t['solver_start_monotonic_ns']<=t['solver_end_monotonic_ns']<=
        t['proposal_publish_monotonic_ns']<=t['proposal_publish_return_monotonic_ns']<=t['callback_body_end_monotonic_ns'])
    result=dict(checks=checks,pass_all=all(checks.values()),request_callbacks=sum(t['kind']=='request' for t in timings),
                scope='Actual Humble DDS/worker, controlled ROS clock, real static T-DT and empty public predictions. Synthetic contract fixture only; no physical/detection acceptance.')
    (root/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    stream.close();driver.destroy_node();worker.destroy_node();rclpy.try_shutdown()
    if not result['pass_all']:raise SystemExit(1)


if __name__=='__main__':main()
