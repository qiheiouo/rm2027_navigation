#!/usr/bin/env python3
"""Bounded, isolated Gazebo trial. Source geometry and safety gates are fixed before execution."""
import argparse
import hashlib
import shutil
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ManageLifecycleNodes
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray


def main():
    parser=argparse.ArgumentParser();parser.add_argument("output",type=Path);parser.add_argument("--mode",choices=["baseline","critic","guard"],required=True);parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--guard-config",type=Path);parser.add_argument('--native-snapshots',action='store_true');args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    # Freeze full installed experiment inputs before any scene/goal is started.
    share=Path(get_package_share_directory("rm_dynamic_obstacle_critic"))
    guard_params=args.guard_config.resolve() if args.guard_config else share/"config"/"guard.yaml"
    frozen=args.output/"installed_inputs";frozen.mkdir()
    input_hashes={}
    input_names=["guard.yaml","tracker_cv.yaml","course_static.yaml","course_static.pgm"]
    if args.native_snapshots:input_names.append('native_command_observer.yaml')
    for name in input_names:
        src=guard_params if name=="guard.yaml" else share/"config"/name
        shutil.copyfile(src,frozen/name)
        input_hashes[name]=hashlib.sha256(src.read_bytes()).hexdigest()
    (args.output/"installed_input_identity.json").write_text(json.dumps(input_hashes,indent=2)+"\n")
    (args.output/"guard_input_selection.json").write_text(json.dumps({"selected_parameter_file":str(guard_params),"frozen_alias":"installed_inputs/guard.yaml","sha256":input_hashes["guard.yaml"]},indent=2)+"\n")
    scene=args.output/"scene_inputs";scene.mkdir()
    sim_share=Path(get_package_share_directory("rm_simulation"))
    scene_hashes={}
    for relative in ("worlds/phase1_omni.sdf","models/moving_obstacle.sdf"):
        src=sim_share/relative;shutil.copyfile(src,scene/src.name)
        scene_hashes[src.name]=hashlib.sha256(src.read_bytes()).hexdigest()
    (args.output/"scene_input_identity.json").write_text(json.dumps({"captured_before_launch":True,"sha256":scene_hashes},indent=2)+"\n")
    binary_paths=[Path(get_package_prefix("rm_dynamic_obstacle_critic"))/"lib"/"rm_dynamic_obstacle_critic"/"dynamic_safety_guard",
                  Path(get_package_prefix("rm_dynamic_obstacle_critic"))/"lib"/"librm_dynamic_obstacle_critic.so",
                  Path(get_package_prefix("nav2_mppi_controller"))/"lib"/"libmppi_controller.so",
                  Path(get_package_prefix("nav2_mppi_controller"))/"lib"/"libmppi_critics.so"]
    if args.native_snapshots:
        binary_paths.append(Path(get_package_prefix('rm_dynamic_obstacle_critic'))/'lib/rm_dynamic_obstacle_critic/native_command_observer')
    binary_identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in binary_paths}
    (args.output/"binary_identity.json").write_text(json.dumps(binary_identity,indent=2)+"\n")
    source_root=Path(__file__).resolve().parents[1]
    source_identity={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in sorted(source_root.rglob("*")) if p.is_file() and "__pycache__" not in p.parts}
    (args.output/"source_identity.json").write_text(json.dumps(source_identity,indent=2)+"\n")
    profile=yaml.safe_load(args.config.read_text());follow=profile["controller_server"]["ros__parameters"]["FollowPath"]
    if args.native_snapshots:
        if not follow['critics'] or follow['critics'][-1]!='NativeCycleSnapshotCritic':
            raise ValueError('native snapshot profile must append its read-only plugin last')
        follow['NativeCycleSnapshotCritic']['output_directory']=str((args.output/'native_cycles').resolve())
    if args.mode=="baseline":
        follow["critics"]=[c for c in follow["critics"] if c not in ("DynamicObstacleCritic","StaticStoppingCritic")]
    params=args.output/"profile.yaml";params.write_text(yaml.safe_dump(profile,sort_keys=False))
    policy={"mode":args.mode,"goal":[5.6,0.,0.],"start_phase":2.,"period":8.,"earliest_start":16.,"phase_tolerance":.06,
            "window":35.,"tail":3.5,"wall_timeout":150.,"body_clearance":.05,"padded_clearance_strict":0.,"raw_costmap_threshold":203,"bounds_tolerance":5e-5,
            "startup":"explicit lifecycle STARTUP after positive sim clock, scan, canonical odometry and tracker receipt",
            "full_mechanical_body_required":True,
            "scope":"one fixed Phase1.5 fixture; not independent field acceptance"}
    (args.output/"policy.json").write_text(json.dumps(policy,indent=2)+"\n")
    if args.native_snapshots:
        (args.output/'native_snapshot_policy.json').write_text(json.dumps({
            'parameters':follow['NativeCycleSnapshotCritic'],'scope':'read-only post-critic candidate data; before gamma/softmax/aggregation/SG',
            'unavailable':['control_sequence_mean','SG_history','exact_dynamic_consumer_input','returned_control'],
            'runtime_output_directory':str((args.output/'native_cycles').resolve())},indent=2)+'\n')
    env=os.environ.copy();env["ROS_LOG_DIR"]=str(args.output/"ros");env["XDG_RUNTIME_DIR"]=str(args.output/"xdg");Path(env["XDG_RUNTIME_DIR"]).mkdir(mode=0o700)
    log=(args.output/"launch.log").open("w");pose_log=(args.output/"gazebo_poses.jsonl").open("w");pose_err=(args.output/"pose_stderr.log").open("w")
    launch=subprocess.Popen(["ros2","launch","rm_dynamic_obstacle_critic","cv_course.launch.py","enabled:=true","nav2_autostart:=false","params_file:="+str(params),"guard_params_file:="+str(guard_params),"guard_enabled:="+("true" if args.mode=="guard" else "false")],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    transport=subprocess.Popen(["ign","topic","-e","--json-output","-t","/world/phase1_omni/pose/info"],stdout=pose_log,stderr=pose_err,env=env,start_new_session=True)
    command_observer=None;command_log=None
    if args.native_snapshots:
        command_log=(args.output/'native_command_observer.log').open('w')
        command_observer=subprocess.Popen(['ros2','run','rm_dynamic_obstacle_critic','native_command_observer','--ros-args',
            '--params-file',str(frozen/'native_command_observer.yaml'),'-p','use_sim_time:=true',
            '-p','output_file:='+str((args.output/'native_commands.jsonl').resolve())],stdout=command_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    rclpy.init();node=rclpy.create_node("cv_trial_observer",parameter_overrides=[rclpy.parameter.Parameter("use_sim_time",value=True)])
    stream=(args.output/"observations.jsonl").open("w",buffering=1);last={};counts={};subs=[]
    def write(kind,record):
        counts[kind]=counts.get(kind,0)+1
        stream.write(json.dumps({"kind":kind,"receive_sim":node.get_clock().now().nanoseconds*1e-9,"receive_wall":time.monotonic(),**record},allow_nan=False)+"\n")
    def odom(m):
        last["odom"]=m;write("odom",{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"xy":[m.pose.pose.position.x,m.pose.pose.position.y],"speed":[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]})
    def diagnostic(kind,m):
        last[kind]=m;write(kind,{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"statuses":[{"name":s.name,"reason":s.message,"level":(s.level[0] if isinstance(s.level,bytes) else s.level),"values":{v.key:v.value for v in s.values}} for s in m.status]})
    def command(kind,m):
        write(kind,{"velocity":[m.linear.x,m.linear.y,m.angular.z]})
    subs.append(node.create_subscription(Odometry,"/simulation/ground_truth/odom",odom,10))
    def canonical_odom(m):
        last["canonical_odom"]=m
        p=m.pose.pose.position;q=m.pose.pose.orientation
        write("canonical_odom",{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"frame":m.header.frame_id,"child_frame":m.child_frame_id,
            "pose":[p.x,p.y,q.x,q.y,q.z,q.w],"velocity":[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]})
    subs.append(node.create_subscription(Odometry,"/odometry/lio",canonical_odom,10))
    def costmap(m):
        p=m.metadata.origin.position;q=m.metadata.origin.orientation
        write("costmap",{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"frame":m.header.frame_id,
            "resolution":m.metadata.resolution,"size":[m.metadata.size_x,m.metadata.size_y],
            "origin":[p.x,p.y,q.x,q.y,q.z,q.w],"data":list(m.data)})
    subs.append(node.create_subscription(Costmap,"/local_costmap/costmap_raw",costmap,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def scan(m):
        last["scan"]=m
        # Keep source-time sensor data for offline audits; never publish TF or
        # transform measurements using latest robot/gimbal orientation.
        write("scan",{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"frame":m.header.frame_id,
            "angle_min":m.angle_min,"angle_max":m.angle_max,"angle_increment":m.angle_increment,
            "time_increment":m.time_increment,"scan_time":m.scan_time,
            "range_min":m.range_min,"range_max":m.range_max,
            "ranges":[r if math.isfinite(r) else "nan" if math.isnan(r) else "inf" if r>0 else "-inf" for r in m.ranges]})
    subs.append(node.create_subscription(LaserScan,"/scan",scan,qos_profile_sensor_data))
    for kind,topic in [("critic","/dynamic_critic/diagnostics"),("guard","/dynamic_guard/diagnostics"),("tracker","/perception/dynamic_obstacles_shadow/diagnostics"),("stopping","/static_stopping/diagnostics")]:
        subs.append(node.create_subscription(DiagnosticArray,topic,lambda m,k=kind:diagnostic(k,m),10))
    for kind,topic in [("final_cmd","/cmd_vel"),("raw_smoothed","/dynamic_test/cmd_vel_smoothed")]:
        subs.append(node.create_subscription(Twist,topic,lambda m,k=kind:command(k,m),10))
    def obstacle(m):
        last["obstacles"]=m
        write("obstacles",{"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"frame":m.header.frame_id,"complete":m.complete,
            "tracks":[{"id":t.track_id,"state":t.state,"xy":[t.position.x,t.position.y],"vxy":[t.velocity.x,t.velocity.y],"size":[t.size.x,t.size.y],"observed":t.last_observation_stamp.sec+t.last_observation_stamp.nanosec*1e-9} for t in m.tracks]})
    subs.append(node.create_subscription(DynamicObstaclePredictionArray,"/perception/dynamic_obstacles_shadow/predictions",obstacle,10))
    startup_client=node.create_client(ManageLifecycleNodes,"/lifecycle_manager_navigation/manage_nodes")
    startup_future=None;startup_ok=False
    action=ActionClient(node,NavigateToPose,"/navigate_to_pose");started=time.monotonic();goal_future=None;result_future=None;handle=None;start_sim=None;end_sim=None;status=None;graph=None;cancelled=False;error=None
    try:
        while time.monotonic()-started<policy["wall_timeout"]:
            if launch.poll() is not None:raise RuntimeError("launch exited before trial completion")
            if command_observer is not None and command_observer.poll() is not None:
                raise RuntimeError('native command evidence observer exited')
            rclpy.spin_once(node,timeout_sec=.02);now=node.get_clock().now().nanoseconds*1e-9
            if startup_future is None and now>2 and all(k in last for k in ("scan","canonical_odom","obstacles")) and startup_client.service_is_ready():
                request=ManageLifecycleNodes.Request();request.command=request.STARTUP
                startup_future=startup_client.call_async(request);write("event",{"event":"nav2_startup_requested"})
            if startup_future is not None and startup_future.done() and not startup_ok:
                if not startup_future.result().success:raise RuntimeError("Nav2 explicit startup failed")
                startup_ok=True;write("event",{"event":"nav2_startup_succeeded"})
            if start_sim is None and now>=policy["earliest_start"] and abs(now%8-policy["start_phase"])<=policy["phase_tolerance"] and "odom" in last and "scan" in last and "obstacles" in last and action.server_is_ready():
                graph={t:[i.node_name for i in node.get_publishers_info_by_topic(t)] for t in ["/cmd_vel","/cmd_vel_nav","/dynamic_test/cmd_vel_smoothed"]}
                expected=["dynamic_safety_guard"] if args.mode=="guard" else ["velocity_smoother"]
                if sorted(graph["/cmd_vel"])!=expected:raise RuntimeError("command publisher identity mismatch: "+str(graph))
                g=NavigateToPose.Goal();g.pose.header.frame_id="map";g.pose.header.stamp=node.get_clock().now().to_msg();g.pose.pose.position.x=5.6;g.pose.pose.orientation.w=1.
                start_sim=now;goal_future=action.send_goal_async(g);write("event",{"event":"goal_sent","graph":graph})
            if goal_future and handle is None and goal_future.done():
                handle=goal_future.result()
                if not handle.accepted:raise RuntimeError("goal rejected")
                result_future=handle.get_result_async()
            if result_future and result_future.done() and end_sim is None:
                status=result_future.result().status;end_sim=now;write("event",{"event":"goal_result","status":status})
            if start_sim is not None and end_sim is None and now-start_sim>policy["window"] and not cancelled:
                if handle:handle.cancel_goal_async()
                cancelled=True;end_sim=now;write("event",{"event":"fixed_timeout_cancel"})
            if end_sim is not None and now-end_sim>=policy["tail"]:break
        else:error="wall_timeout"
    except Exception as e:error=str(e)
    finally:
        final_sim=node.get_clock().now().nanoseconds*1e-9
        for proc in [p for p in [command_observer,transport,launch] if p is not None]:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        stream.close();pose_log.close();pose_err.close();log.close();node.destroy_node();rclpy.shutdown()
        if command_log is not None:command_log.close()
    summary={"mode":args.mode,"execution":"PASS" if error is None and start_sim is not None else "FAILED","error":error,"goal_status":status,"cancelled":cancelled,"start_sim":start_sim,"end_sim":end_sim,"last_sim":final_sim,"graph":graph,"message_counts":counts}
    (args.output/"execution.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary),flush=True)
    return 0 if summary["execution"]=="PASS" else 1
if __name__=="__main__":raise SystemExit(main())
