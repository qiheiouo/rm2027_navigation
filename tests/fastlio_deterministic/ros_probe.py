"""Reuse the audit's 900-point plane/200 Hz IMU fixture with continuous frames.

Complete production bundle executable, real ROS messages and DDS. Synthetic
software regression input only: no physical sensor, canonical TF or Nav2.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint

parser=argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--expect-abort', action='store_true')
parser.add_argument('--frames', type=int, default=8)
args=parser.parse_args()
output=Path(args.output); output.mkdir(parents=True,exist_ok=True)

def stamp(s,t):
    ns=round(t*1e9); s.sec=ns//1000000000; s.nanosec=ns%1000000000

def cloud(t,x):
    m=CustomMsg(); stamp(m.header.stamp,t); m.header.frame_id='livox_frame'; m.timebase=round(t*1e9)
    for i in range(900):
        p=CustomPoint(); p.x=x; p.y=-3.+(i%30)*.2; p.z=-3.+(i//30)*.2
        p.offset_time=int(i*99900000/899); p.reflectivity=50; p.line=i%4; p.tag=0
        m.points.append(p)
    m.point_num=len(m.points); return m

class Observer(Node):
    def __init__(self,name):
        super().__init__('fastlio_fix_'+name); self.samples=[]
        self.lidar=self.create_publisher(CustomMsg,'/fix/'+name+'/lidar',10)
        self.imu=self.create_publisher(Imu,'/fix/'+name+'/imu',100)
        self.sub=self.create_subscription(Odometry,'/fix/'+name+'/odom',self.samples.append,10)
    def spin(self,d):
        end=time.monotonic()+d
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=min(.01,max(0.,end-time.monotonic())))

def execute(name,matched):
    n=Observer(name); rows=[]
    with (output/(name+'.stdout')).open('w') as out, (output/(name+'.stderr')).open('w') as err:
        child=subprocess.Popen([args.binary,'--ros-args','--params-file','/workspace/src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml',
            '-p','common.lid_topic:=/fix/'+name+'/lidar','-p','common.imu_topic:=/fix/'+name+'/imu',
            '-r','/Odometry:=/fix/'+name+'/odom','-r','/tf:=/fix/'+name+'/backend_tf'],stdout=out,stderr=err,start_new_session=True)
        try:
            end=time.monotonic()+5
            while time.monotonic()<end and child.poll() is None and (n.lidar.get_subscription_count()<1 or n.imu.get_subscription_count()<1): n.spin(.03)
            assert child.poll() is None and n.lidar.get_subscription_count()>=1 and n.imu.get_subscription_count()>=1
            # Allow newly discovered DDS endpoints to finish connecting before
            # the one-shot initialization/map frames (as in the audit fixture).
            n.spin(.5)
            for k in range(args.frames):
                t=1000.1+k*.11; n.lidar.publish(cloud(t,5. if matched or k<5 else 25.)); n.spin(.035)
                for j in range(22):
                    m=Imu(); stamp(m.header.stamp,t+j*.005); m.header.frame_id='livox_imu'; m.linear_acceleration.z=9.81
                    n.imu.publish(m); n.spin(.003)
                    if child.poll() is not None: break
                n.spin(.22); rows.append({'frame':k,'odom_count':len(n.samples),'child_exit':child.poll()})
                if k>=2:
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline and child.poll() is None and len(n.samples)<k-1: n.spin(.05)
                if child.poll() is not None: break
            deadline=time.monotonic()+15
            while time.monotonic()<deadline and child.poll() is None and len(n.samples)<args.frames-2: n.spin(.05)
            before=child.poll()
            samples=[]
            for m in n.samples:
                p=m.pose.pose
                v=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
                assert all(math.isfinite(x) for x in v)
                samples.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':v,'parent':m.header.frame_id,'child':m.child_frame_id})
            (output/(name+'_observed.json')).write_text(json.dumps({'phases':rows,'before_cleanup_exit':before,'samples':samples},indent=2)+'\n')
            if args.expect_abort and matched: assert before==-signal.SIGABRT
            else: assert before is None and len(samples)>=args.frames-2
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait(); raise
            n.destroy_node()
    if not (args.expect_abort and matched): assert child.returncode==0
    no_effective=(output/(name+'.stderr')).read_text().count('No Effective Points!')
    if not matched: assert no_effective>0, 'Unmatched control must reach actual rejected measurement'
    return {'case':name,'phases':rows,'before_cleanup_exit':before,'final_exit':child.returncode,'samples':samples,
            'initializations':(output/(name+'.stdout')).read_text().count('IMU Initial Done'),
            'no_effective_points':no_effective}

rclpy.init()
try: result={'scope':__doc__,'cases':[execute('matched',True),execute('unmatched',False)]}
finally: rclpy.shutdown()
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'cases':[{k:v for k,v in r.items() if k!='samples'} for r in result['cases']]}))
