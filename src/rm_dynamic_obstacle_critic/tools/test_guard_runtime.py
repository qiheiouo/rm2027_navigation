#!/usr/bin/env python3
"""Actual guard process/DDS tests. No simulator or hardware command topics used."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile, DurabilityPolicy
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array, DynamicObstaclePrediction as Track


def main():
    parser=argparse.ArgumentParser();parser.add_argument("output",type=Path);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    log=(args.output/"guard.log").open("w")
    proc=subprocess.Popen(["ros2","run","rm_dynamic_obstacle_critic","dynamic_safety_guard","--ros-args",
        "-p","world_frame:=map","-p","input_topic:=/guard_test/input","-p","output_topic:=/guard_test/output",
        "-p","obstacles_topic:=/guard_test/obstacles","-p","odom_topic:=/guard_test/odom","-p","costmap_topic:=/guard_test/map"],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node("guard_runtime_test");latest=[];diag=[]
    pubs={"cmd":node.create_publisher(Twist,"/guard_test/input",1),"odom":node.create_publisher(Odometry,"/guard_test/odom",1),
          "map":node.create_publisher(Costmap,"/guard_test/map",QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
          "obs":node.create_publisher(Array,"/guard_test/obstacles",1)}
    sub=node.create_subscription(Twist,"/guard_test/output",lambda m:latest.append([m.linear.x,m.linear.y,m.angular.z]),10)
    from diagnostic_msgs.msg import DiagnosticArray
    ds=node.create_subscription(DiagnosticArray,"/dynamic_guard/diagnostics",lambda m:diag.append({"reason":m.status[0].message,"values":{v.key:v.value for v in m.status[0].values}}),10)
    rows=[]
    def exercise(name,mode,expected):
        latest.clear();diag.clear();end=time.monotonic()+.65
        while time.monotonic()<end:
            now=node.get_clock().now().to_msg()
            cmd=Twist();cmd.linear.x=0. if mode=="measured" else .8 if mode=="collision" else .2
            odom=Odometry();odom.header.stamp=now;odom.header.frame_id="map";odom.child_frame_id="base_link";odom.pose.pose.orientation.w=1.0
            if mode=="measured":odom.twist.twist.linear.x=.8
            a=Array();a.header.stamp=now;a.header.frame_id="map";a.schema=Array.SCHEMA;a.authority=Array.AUTHORITY_SHADOW_ONLY;a.complete=True;a.prediction_dt=.1;a.prediction_steps=30
            if mode=="stale":a.header.stamp.sec-=2
            if mode=="collision":
                t=Track();t.track_id=1;t.state=Track.STATE_CONFIRMED;t.position.x=.9;t.position.y=-.9;t.velocity.y=1.0;t.size.x=.45;t.size.y=.55;t.last_observation_stamp=now;a.tracks=[t];a.total_track_count=1
            m=Costmap();m.header.stamp=now;m.header.frame_id="map";m.metadata.size_x=80;m.metadata.size_y=80;m.metadata.resolution=.1;m.metadata.origin.position.x=-4.;m.metadata.origin.position.y=-4.;m.metadata.origin.orientation.w=1.0;m.data=[0]*6400
            if mode=="unknown":m.data[40*80+40]=255
            if mode=="measured":m.data[40*80+46]=203
            if mode!="missing":
                pubs["cmd"].publish(cmd);pubs["obs"].publish(a);pubs["map"].publish(m)
                if mode!="no_odom":pubs["odom"].publish(odom)
            rclpy.spin_once(node,timeout_sec=.015);time.sleep(.005)
        tail=latest[-5:];passed=bool(tail) and all(abs(x[0]-expected)<1e-6 and x[1:]==[0.,0.] for x in tail)
        diagnostic_pass=True
        if mode in ("collision","unknown","measured"):
            expected_branch=1 if mode=="collision" else 0
            diagnostic_pass=bool(diag[-5:]) and all(int(float(d["values"].get("collision_branch",-1)))==expected_branch for d in diag[-5:])
            if mode in ("unknown","measured"):
                expected_cost=255 if mode=="unknown" else 203
                diagnostic_pass &= all(int(float(d["values"].get("static_cell_cost",-1)))==expected_cost for d in diag[-5:])
        rows.append({"case":name,"passed":passed and diagnostic_pass,"output_tail":tail,"reason_tail":[d["reason"] for d in diag[-5:]],"diagnostic_tail":diag[-5:]})
    try:
        time.sleep(.6)
        exercise("missing_input_brakes","missing",0.)
        exercise("fresh_clear_command_passes","clear",.2)
        exercise("future_collision_brakes","collision",0.)
        exercise("stale_tracker_brakes","stale",0.)
        exercise("odometry_watchdog_brakes","no_odom",0.)
        exercise("unknown_static_cell_brakes","unknown",0.)
        exercise("measured_motion_stopping_tail_brakes","measured",0.)
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        node.destroy_node();rclpy.shutdown();log.close()
    result={"verdict":"PASS" if all(r["passed"] for r in rows) else "FAILED","cases":rows}
    (args.output/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({"verdict":result["verdict"],"cases":[{"case":r["case"],"passed":r["passed"]} for r in rows]}))
    return 0 if result["verdict"]=="PASS" else 1
if __name__=="__main__":raise SystemExit(main())
