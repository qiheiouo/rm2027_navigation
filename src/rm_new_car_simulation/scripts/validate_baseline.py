#!/usr/bin/env python3
"""Run and preserve physical wheel/twist, sensor/joint TF and collision evidence."""
import argparse
from collections import defaultdict
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from scipy.spatial.transform import Rotation


class Recorder(Node):
    def __init__(self):
        super().__init__('new_car_baseline_validation');self.base={};self.sensor={};self.joints={};self.tf={};self.samples=[];self.static={}
        self.subs=[self.create_subscription(Odometry,'/simulation/ground_truth/odom',lambda m:self.record(self.base,m),100),
          self.create_subscription(Odometry,'/simulation/sensor_truth',lambda m:self.record(self.sensor,m),100),
          self.create_subscription(JointState,'/joint_states',self.joint,100),self.create_subscription(TFMessage,'/tf',self.transforms,100)]
        self.subs.append(self.create_subscription(TFMessage,'/tf_static',self.static_transforms,
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
        self.pub=self.create_publisher(Twist,'/cmd_vel',10);self.gimbal=self.create_publisher(Float64,'/simulation/gimbal_velocity',10)
        self.contacts=defaultdict(dict)
        for name in ['front_left','front_right','rear_left','rear_right']:
            self.subs.append(self.create_subscription(Contacts,f'/simulation/contact/{name}',
                lambda m,n=name:self.contact(n,m),100))

    def contact(self,name,msg):
        forces=[];depths=[]
        for contact in msg.contacts:
            depths.extend(contact.depths)
            for wrench in contact.wrenches:
                f=wrench.body_1_wrench.force
                forces.append([f.x,f.y,f.z])
        self.contacts[name][self.stamp(msg.header.stamp)]={'count':len(msg.contacts),
            'force_world':np.sum(forces,axis=0).tolist() if forces else [0.,0.,0.],
            'max_depth_m':max(depths,default=0.)}

    @staticmethod
    def stamp(stamp): return stamp.sec*1000000000+stamp.nanosec

    def record(self,cache,m):
        t=self.stamp(m.header.stamp);cache[t]=m
        if cache is self.base: self.samples.append(m)

    def joint(self,m): self.joints[self.stamp(m.header.stamp)]=dict(zip(m.name,zip(m.position,m.velocity)))

    def transforms(self,m):
        for t in m.transforms: self.tf[(self.stamp(t.header.stamp),t.child_frame_id)]=t

    def static_transforms(self,m):
        for t in m.transforms: self.static[t.child_frame_id]=t

    def run_for(self,seconds,command=(0,0,0),gimbal=.0):
        start=time.monotonic();nextpub=0
        while time.monotonic()-start<seconds:
            now=time.monotonic()
            if now>=nextpub:
                msg=Twist();msg.linear.x=float(command[0]);msg.linear.y=float(command[1]);msg.angular.z=float(command[2]);self.pub.publish(msg)
                self.gimbal.publish(Float64(data=float(gimbal)));nextpub=now+.03
            rclpy.spin_once(self,timeout_sec=.005)


def pose(p):
    t=np.eye(4);q=p.orientation;t[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    t[:3,3]=[p.position.x,p.position.y,p.position.z];return t


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--field',default='flat');parser.add_argument('--lidar',default='A');parser.add_argument('--profile',default='deformed')
    parser.add_argument('--spawn',default='0,0,0.002,0,0,0');parser.add_argument('--mode',choices=['matrix','drive'],default='matrix')
    parser.add_argument('--command',default='.25,0,0');parser.add_argument('--duration',type=float,default=8)
    parser.add_argument('--gimbal',type=float,default=.6)
    parser.add_argument('--physics-config',default='');parser.add_argument('--save-trace',action='store_true')
    parser.add_argument('--tunnel-section',default='B');parser.add_argument('--field-tolerance',default='nominal')
    parser.add_argument('--ground-bank-deg',type=float,default=1.5)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    scene=args.output/'scene';log=(args.output/'launch.log').open('w')
    launch_args=['ros2','launch','rm_new_car_simulation','baseline.launch.py',f'field:={args.field}',
        f'lidar:={args.lidar}',f'profile:={args.profile}',f'output:={scene}',f'spawn:={args.spawn}',
        f'tunnel_section:={args.tunnel_section}',f'field_tolerance:={args.field_tolerance}',
        f'ground_bank_deg:={args.ground_bank_deg}']
    if args.physics_config:launch_args.append(f'physics_config:={args.physics_config}')
    child=subprocess.Popen(launch_args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Recorder();results=[]
    try:
        deadline=time.monotonic()+45
        while len(node.base)<20 and time.monotonic()<deadline and child.poll() is None: rclpy.spin_once(node,timeout_sec=.1)
        if len(node.base)<20: raise RuntimeError('no physics truth; see launch.log')
        node.run_for(1)
        manifest=json.loads((scene/'manifest.json').read_text());matrix=np.array(manifest['wheel_matrix_rad_s_per_twist'])
        commands=[('stationary_gimbal',(0,0,0)),('vx',(.25,0,0)),('vy',(0,.25,0)),('wz',(0,0,.4)),('vx_vy',(.2,.15,0)),
                  ('vx_wz',(.2,0,.3)),('vx_vy_wz',(.2,.15,.3))]
        if args.mode=='drive': commands=[('drive',tuple(map(float,args.command.split(','))))]
        else: commands+= [(n+'_reverse',tuple(-v for v in c)) for n,c in list(commands) if n!='stationary_gimbal']
        for name,cmd in commands:
            node.run_for(.5)
            before=len(node.samples);node.run_for(args.duration if args.mode=='drive' else 2.0,cmd,args.gimbal)
            samples=node.samples[before:];late=samples[len(samples)//2:]
            values=np.array([[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z] for m in late])
            speeds=[]
            for m in late:
                j=node.joints.get(node.stamp(m.header.stamp))
                if j and all(n+'_wheel_joint' in j for n in ['front_left','front_right','rear_left','rear_right']):
                    speeds.append([j[n+'_wheel_joint'][1] for n in ['front_left','front_right','rear_left','rear_right']])
            mean=values.mean(axis=0);expected=matrix@cmd;actual=np.mean(speeds,axis=0) if speeds else np.full(4,np.nan)
            results.append(dict(name=name,command=list(cmd),measured_body_twist=mean.tolist(),
                body_twist_error=(mean-np.array(cmd)).tolist(),expected_wheel_rad_s=expected.tolist(),
                measured_wheel_rad_s=actual.tolist(),wheel_error_rad_s=(actual-expected).tolist(),
                wheel_matched_samples=len(speeds),truth_samples=len(samples),
                min_max_xyz=np.array([[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] for m in samples]).min(axis=0).tolist()+
                            np.array([[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] for m in samples]).max(axis=0).tolist(),
                max_abs_roll_pitch_rad=np.abs(np.array([Rotation.from_matrix(pose(m.pose.pose)[:3,:3]).as_euler('xyz')[:2] for m in samples])).max(axis=0).tolist(),
                simulation_elapsed_sec=(node.stamp(samples[-1].header.stamp)-node.stamp(samples[0].header.stamp))/1e9,
                first_pose_matrix=pose(samples[0].pose.pose).tolist(),last_pose_matrix=pose(samples[-1].pose.pose).tolist()))
        node.run_for(.5)
        # Compare physics sensor link to T_world_base * T_base_gimbal(actual joint) * T_gimbal_sensor.
        center=np.array(manifest['gimbal_center_m']);offset=np.array(manifest['sensor_offset_m'])
        errors=[];tf_errors=[];yaw_samples=[]
        for stamp,m in node.sensor.items():
            if stamp not in node.base or stamp not in node.joints: continue
            j=node.joints[stamp]
            if 'gimbal_yaw_joint' not in j: continue
            angle=j['gimbal_yaw_joint'][0];yaw_samples.append(angle)
            bg=np.eye(4);bg[:3,3]=center;bg[:3,:3]=Rotation.from_euler('z',angle).as_matrix()
            gs=np.eye(4);gs[:3,3]=offset
            expected=pose(node.base[stamp].pose.pose)@bg@gs;truth=pose(m.pose.pose)
            errors.append([float(np.linalg.norm(expected[:3,3]-truth[:3,3])),
                           float(Rotation.from_matrix(expected[:3,:3].T@truth[:3,:3]).magnitude())])
            bt=node.tf.get((stamp,'base_link'));gt=node.tf.get((stamp,'gimbal_yaw_link'))
            st=node.static.get('sim_lidar_link')
            if bt is not None and gt is not None and st is not None:
                def tp(t):
                    a=np.eye(4);q=t.transform.rotation;a[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                    p=t.transform.translation;a[:3,3]=[p.x,p.y,p.z];return a
                ros=tp(bt)@tp(gt)@tp(st)
                tf_errors.append([float(np.linalg.norm(ros[:3,3]-truth[:3,3])),
                                  float(Rotation.from_matrix(ros[:3,:3].T@truth[:3,:3]).magnitude())])
        evidence=dict(mode=args.mode,field=args.field,lidar=args.lidar,profile=args.profile,rows=results,
            matched_sensor_samples=len(errors),max_sensor_transform_error_m_rad=np.max(errors,axis=0).tolist() if errors else None,
            matched_ros_tf_samples=len(tf_errors),max_ros_tf_error_m_rad=np.max(tf_errors,axis=0).tolist() if tf_errors else None,
            gimbal_actual_range_rad=[min(yaw_samples),max(yaw_samples)] if yaw_samples else None,
            manifest=manifest,simulation_oracle=True,hardware_validated=False)
        evidence['contact_messages']={n:len(v) for n,v in node.contacts.items()}
        if args.save_trace:
            trace=[]
            for stamp,m in sorted(node.base.items()):
                p=m.pose.pose;q=p.orientation
                trace.append({'stamp_ns':stamp,'xyz':[p.position.x,p.position.y,p.position.z],
                    'xyzw':[q.x,q.y,q.z,q.w],'joint_state':node.joints.get(stamp),
                    'contacts':{n:v.get(stamp) for n,v in node.contacts.items()}})
            (args.output/'trace.json').write_text(json.dumps(trace,separators=(',',':'))+'\n')
        if args.mode=='matrix':
            evidence['kinematic_gate']=all(np.max(np.abs(r['wheel_error_rad_s']))<.05 and
                np.max(np.abs(r['body_twist_error']))<.05 and r['wheel_matched_samples']>10 for r in results)
            evidence['dynamic_tf_gate']=bool(len(errors)>100 and len(tf_errors)>100 and np.max(errors)<.002 and np.max(tf_errors)<.002)
        def clean(value):
            if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
            if isinstance(value,list): return [clean(v) for v in value]
            if isinstance(value,float) and not math.isfinite(value): return None
            return value
        evidence=clean(evidence)
        (args.output/'result.json').write_text(json.dumps(evidence,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in evidence.items() if k not in ['manifest','rows']},indent=2))
    finally:
        node.destroy_node();rclpy.try_shutdown();child.send_signal(signal.SIGINT)
        try: child.wait(timeout=12)
        except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close()


if __name__=='__main__': main()
