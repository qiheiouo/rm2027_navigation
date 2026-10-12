"""Real fixed bundle, DDS and synthetic plane/IMU input; not sensor acceptance.
Reuse the existing fastlio_deterministic/ros_probe plane and 200Hz IMU fixture.
Observe real output stamps for arrival skew, source skew and missing sources.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg,CustomPoint

parser=argparse.ArgumentParser();parser.add_argument('--binary',required=True);parser.add_argument('--output',required=True)
args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
ROOT=Path(__file__).resolve().parents[2]

def stamp(msg,t):
    ns=round(t*1e9);msg.sec=ns//10**9;msg.nanosec=ns%10**9

def cloud(t,right=False):
    m=CustomMsg();stamp(m.header.stamp,t);m.header.frame_id='mid360_right_frame' if right else 'mid360_left_frame';m.timebase=round(t*1e9);m.lidar_id=2 if right else 1
    for i in range(900):
        p=CustomPoint();x=5.;y=-3.+i%30*.2;z=-3.+i//30*.2
        # L1<-L2 = Rz(pi), t=(0,-.24,0), so both planes coincide in L1.
        p.x=-x if right else x;p.y=-(y+.24) if right else y;p.z=z
        p.offset_time=int(i*99900000/899);p.reflectivity=50;p.line=i%4;p.tag=0;m.points.append(p)
    m.point_num=len(m.points);return m

class Probe(Node):
    def __init__(self,name):
        super().__init__('dual_probe_'+name);self.rows=[];self.prefix='/dual/'+name
        self.left=self.create_publisher(CustomMsg,self.prefix+'/left',10);self.right=self.create_publisher(CustomMsg,self.prefix+'/right',10);self.imu=self.create_publisher(Imu,self.prefix+'/imu',100)
        self.sub=self.create_subscription(Odometry,self.prefix+'/odom',self.rows.append,100)
    def spin(self,secs):
        end=time.monotonic()+secs
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
    def imus(self,t,end):
        while t<=end+1e-6:
            m=Imu();stamp(m.header.stamp,t);m.header.frame_id='lio_imu_link';m.linear_acceleration.z=9.81;self.imu.publish(m);self.spin(.003);t+=.005
    def pair(self,t,offset=0.,delay=False,imus=True):
        if delay:self.right.publish(cloud(t+offset,True));self.spin(.04);self.left.publish(cloud(t))
        else:self.left.publish(cloud(t));self.spin(.02);self.right.publish(cloud(t+offset,True))
        self.spin(.035)
        if imus:self.imus(t,t+.105+max(0,offset))
        self.spin(.25)

from check_config import fixture
cfg=fixture();config=out/'software_fixture.yaml';config.write_text(yaml.safe_dump(cfg))
rclpy.init();result={'scope':__doc__,'cases':[]}
try:
    for name in ['steady','arrival_delay','source_skew','right_gap','imu_gap']:
        n=Probe(name);phases=[];log=(out/(name+'.log')).open('w')
        p=subprocess.Popen([args.binary,'--ros-args','--params-file',str(config),'-p','common.lid_topic:='+n.prefix+'/left','-p','common.lid_topic2:='+n.prefix+'/right','-p','common.imu_topic:='+n.prefix+'/imu','-r','/Odometry:='+n.prefix+'/odom','-r','/tf:='+n.prefix+'/quarantine_tf'],stdout=log,stderr=subprocess.STDOUT)
        try:
            end=time.monotonic()+10
            while time.monotonic()<end and (n.left.get_subscription_count()<1 or n.right.get_subscription_count()<1 or n.imu.get_subscription_count()<1):n.spin(.03)
            assert p.poll() is None and n.left.get_subscription_count()>=1 and n.right.get_subscription_count()>=1 and n.imu.get_subscription_count()>=1
            n.spin(.5)
            offset=.025 if name=='source_skew' else 0.
            for k in range(8):
                n.pair(1000.1+k*.15,offset,name=='arrival_delay');phases.append({'phase':'normal','k':k,'odoms':len(n.rows)})
            assert len(n.rows)>=6,(name,len(n.rows))
            before=len(n.rows)
            if name=='right_gap':
                for k in range(3):
                    t=1001.3+k*.15;n.left.publish(cloud(t));n.imus(t,t+.105);n.spin(.25)
                assert len(n.rows)==before,'no silent single-lidar continuation permitted by current backend'
                phases.append({'phase':'missing_right','odoms':len(n.rows)})
                n.right.publish(cloud(1001.6,True));n.imus(1001.71,1001.72);n.spin(.5)
                assert len(n.rows)>before
                phases.append({'phase':'right_resumed','odoms':len(n.rows),'last_stamp':n.rows[-1].header.stamp.sec+n.rows[-1].header.stamp.nanosec*1e-9})
            if name=='imu_gap':
                for k in range(3):n.pair(1001.3+k*.15,imus=False)
                assert len(n.rows)==before,'no update without IMU coverage'
                phases.append({'phase':'missing_imu','odoms':len(n.rows)})
                n.imus(1001.26,1001.71);n.spin(.5);assert len(n.rows)>before
                phases.append({'phase':'imu_resumed','odoms':len(n.rows),'last_stamp':n.rows[-1].header.stamp.sec+n.rows[-1].header.stamp.nanosec*1e-9})
            values=[]
            for m in n.rows:
                q=m.pose.pose;v=[q.position.x,q.position.y,q.position.z,q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w]
                assert all(math.isfinite(x) for x in v);values.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':v,'frames':[m.header.frame_id,m.child_frame_id]})
            assert p.poll() is None
            result['cases'].append({'name':name,'samples':values,'phases':phases})
        finally:
            if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=6)
            log.close();n.destroy_node()
        assert p.returncode==0
finally:
    rclpy.shutdown();(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'cases':[{'name':c['name'],'odoms':len(c['samples']),'phases':c['phases'][-2:]} for c in result['cases']]}))
