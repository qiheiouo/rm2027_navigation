"""Actual RSP + gimbal_state_adapter + lio_adapter, synthetic geometry/motion.
Demonstrates supported canonical-parent conversion AND sensor_initial counterexample.
No heading fusion, truth navigation, backend, map TF, chassis or serial.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import GimbalState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parents[2];out=Path(os.environ['DUAL_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
install=Path(os.environ['DUAL_INSTALL']);rclpy.init();n=Node('dynamic_rigid_probe');buf=Buffer();listener=TransformListener(buf,n)
gimbal=n.create_publisher(GimbalState,'/gimbal/state',10);raw=n.create_publisher(Odometry,'/odometry/fast_lio_raw',10);samples=[];owners={};processes=[];logs=[]
n.create_subscription(Odometry,'/odometry/lio',samples.append,100)
def on_tf(msg):
    for t in msg.transforms:owners.setdefault((t.header.frame_id,t.child_frame_id),set()).add("observed")
n.create_subscription(TFMessage,'/tf',on_tf,100)
def spin(sec):
    end=time.monotonic()+sec
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
def start(package,binary,args=()):
    f=(out/(binary+str(len(processes))+'.log')).open('w');logs.append(f)
    if package=='tf_probe':exe=os.environ['DUAL_TF_PROBE']
    elif package=='robot_state_publisher':exe='/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
    else:exe=str(install/package/'lib'/package/binary)
    p=subprocess.Popen([exe,'--ros-args',*args],stdout=f,stderr=subprocess.STDOUT);processes.append(p);return p
def stop(p):
    if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=5)
def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def matrix(t):
    m=np.eye(4);m[:3,:3]=rotation(t.rotation);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m
def pose_matrix(p):
    m=np.eye(4);m[:3,:3]=rotation(p.orientation);m[:3,3]=[p.position.x,p.position.y,p.position.z];return m
def sample_gimbal(yaw):
    m=GimbalState();m.header.stamp=n.get_clock().now().to_msg();m.valid=True;m.online=True;m.relative_yaw_rad=yaw;m.yaw_rate_rad_s=0.;gimbal.publish(m)
    end=time.monotonic()+1.
    while time.monotonic()<end:
        spin(.01)
        if buf.can_transform('base_link','lio_imu_link',Time.from_msg(m.header.stamp)):
            return m.header.stamp,matrix(buf.lookup_transform('base_link','lio_imu_link',Time.from_msg(m.header.stamp)).transform)
    raise AssertionError('real source-stamped RSP transform missing')
def base(yaw):
    m=np.eye(4);c,s=math.cos(yaw),math.sin(yaw);m[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];return m
result={'scope':__doc__,'cases':[]}
try:
    urdf=subprocess.check_output(['xacro',str(ROOT/'src/rm_description/urdf/rm_sentry_2027.urdf.xacro')],text=True)
    rsp_config=out/'rsp_software_fixture.yaml';rsp_config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':urdf,'publish_frequency':200.0}}}))
    start('robot_state_publisher','robot_state_publisher',['--params-file',str(rsp_config)])
    start('rm_localization_adapters','gimbal_state_adapter',['-p','use_input:=true'])
    spin(1.)
    for mode in ['canonical_odom','sensor_initial']:
        p=start('rm_localization_adapters','lio_adapter',['-p','raw_odom_parent_frame_mode:='+mode,'-p','twist_mode:=finite_difference','-p','twist_estimator.smoothing_alpha:=1.0','-p','twist_estimator.max_angular_speed:=100.0'])
        gid_path=out/(mode+'_tf_gids.json')
        tfp=start('tf_probe','s2_tf_owner_probe',['-p','output:='+str(gid_path),'-p','edges:=[map>odom,odom>base_link,base_link>gimbal_yaw_link]'])
        spin(.7);samples.clear();owners.clear();t0,b0=sample_gimbal(0.)
        # Three cases use a common known initial mount, same real components.
        for case,by,gy in [('gimbal_only',0.,.6),('base_only',.4,0.),('opposite',.4,-.4)]:
            # Initialization / finite difference history is not claimed as a navigation measurement.
            for repeat in range(4):
                ts,bt=sample_gimbal(gy);g=base(by)
                sensor=g@bt if mode=='canonical_odom' else np.linalg.inv(b0)@g@bt
                # Advance dynamic TF after the measurement: conversion must
                # select bt at ts, not the most recent mount rotation.
                sample_gimbal(gy+.2)
                m=Odometry();m.header.stamp=ts;m.header.frame_id='odom';m.child_frame_id='lio_imu_link'
                m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,sensor[:3,3])
                yaw=math.atan2(sensor[1,0],sensor[0,0]);m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
                m.pose.covariance[0]=.01;m.pose.covariance[35]=.02
                before=len(samples);raw.publish(m);spin(.09)
            assert samples and len(samples)>before,(mode,case,'no real adapter output')
            observed=samples[-1];mat=pose_matrix(observed.pose.pose)
            pe=float(np.linalg.norm(mat[:3,3]-g[:3,3]));ae=math.acos(max(-1.,min(1.,float((np.trace(g[:3,:3].T@mat[:3,:3])-1)/2))))
            stamp_ok=observed.header.stamp==ts
            assert stamp_ok and observed.header.frame_id=='odom' and observed.child_frame_id=='base_link'
            if mode=='canonical_odom':assert pe<1e-7 and ae<1e-7,(case,pe,ae)
            elif case=='base_only':assert pe<1e-7 and ae<1e-7
            else:assert ae>.3,'preserve demonstrated dynamic sensor_initial gap'
            result['cases'].append({'mode':mode,'case':case,'position_error_m':pe,'angle_error_rad':ae,'source_stamp_preserved':stamp_ok,'raw_pose_covariance_copied':bool(observed.pose.covariance[0]==.01),'twist':[observed.twist.twist.linear.x,observed.twist.twist.linear.y,observed.twist.twist.angular.z]})
        gids=json.loads(gid_path.read_text())
        assert len(gids.get('odom>base_link',[]))==1,gids
        assert len(gids.get('base_link>gimbal_yaw_link',[]))==1,gids
        assert not gids.get('map>odom'),gids
        result[mode+'_tf_owner_counts']={edge:len(ids) for edge,ids in gids.items()}
        stop(p);stop(tfp);spin(.3)
finally:
    for p in reversed(processes):stop(p)
    for f in logs:f.close()
    n.destroy_node();rclpy.shutdown();(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
