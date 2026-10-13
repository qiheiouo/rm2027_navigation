"""Actual bundle/DDS fault pairs. No real MID360 or serial/hardware output."""
import argparse, json, math, signal, subprocess, time, sys
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/new_car_dual'))
from check_config import fixture
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
def stamp(s,t):
    ns=round(t*1e9);s.sec=ns//10**9;s.nanosec=ns%10**9
def cloud(t,right=False):
    m=CustomMsg();stamp(m.header.stamp,t);m.timebase=round(t*1e9);m.lidar_id=2 if right else 1
    for i in range(900):
        c=CustomPoint();y=-3.+i%30*.2;c.x=-5. if right else 5.;c.y=-(y+.24) if right else y;c.z=-3.+i//30*.2
        c.offset_time=int(i*99900000/899);c.reflectivity=50;c.line=i%4;m.points.append(c)
    m.point_num=len(m.points);return m
class Probe(Node):
    def __init__(self,name):
        super().__init__('epoch_probe_'+name);self.prefix='/epoch/'+name;self.rows=[]
        self.left=self.create_publisher(CustomMsg,self.prefix+'/left',10);self.right=self.create_publisher(CustomMsg,self.prefix+'/right',10)
        self.imu=self.create_publisher(Imu,self.prefix+'/imu',100);self.create_subscription(Odometry,self.prefix+'/odom',self.rows.append,100)
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
    def imus(self,t,end):
        while t<=end+1e-7:
            m=Imu();stamp(m.header.stamp,t);m.linear_acceleration.z=9.81;self.imu.publish(m);self.spin(.003);t+=.005
    def pair(self,t):
        self.left.publish(cloud(t));self.spin(.02);self.right.publish(cloud(t,True));self.spin(.04)
    def warm(self,t):
        for k in range(8):
            q=t+k*.15;self.pair(q);self.imus(q,q+.105);self.spin(.2)
        assert len(self.rows)>=6
cfg=fixture();cfg['/**']['ros__parameters']['common']['imu_max_gap_sec']=.1
# Stable source parent; keep ESKF settings unchanged.
cfg['/**']['ros__parameters']['mapping']['zero_start_pose']=False
config=out/'fixture.yaml';config.write_text(yaml.safe_dump(cfg));result={'scope':__doc__,'cases':[]};rclpy.init()
try:
    for name in ['backfill','imu_gap','imu_rollback','lidar_restart','both_restart','duplicate','new_epoch']:
        n=Probe(name);log=(out/(name+'.log')).open('w');start=900.1 if name=='new_epoch' else 1000.1
        proc=subprocess.Popen([args.binary,'--ros-args','--params-file',str(config),'-p','common.lid_topic:='+n.prefix+'/left',
             '-p','common.lid_topic2:='+n.prefix+'/right','-p','common.imu_topic:='+n.prefix+'/imu','-r','/Odometry:='+n.prefix+'/odom'],stdout=log,stderr=subprocess.STDOUT)
        try:
            end=time.monotonic()+10
            while time.monotonic()<end and min(n.left.get_subscription_count(),n.right.get_subscription_count(),n.imu.get_subscription_count())<1:n.spin(.02)
            assert proc.poll() is None;n.spin(.3);n.warm(start);before=len(n.rows)
            if name=='backfill':
                for k in range(3):n.pair(1001.3+k*.15);n.spin(.1)
                assert len(n.rows)==before;n.imus(1001.26,1001.71);n.spin(.3);assert len(n.rows)>before
            elif name=='duplicate':
                n.imus(start+1.155,start+1.155);n.left.publish(cloud(start+1.05));n.right.publish(cloud(start+1.05,True));n.spin(.3)
                assert len(n.rows)==before
                n.pair(start+1.2);n.imus(start+1.2,start+1.305);n.spin(.3);assert len(n.rows)>before
            elif name!='new_epoch':
                n.pair(1001.3)
                if name=='imu_gap':n.imus(1001.71,1001.71)
                if name=='imu_rollback':n.imus(900.1,900.1)
                if name=='lidar_restart':n.right.publish(cloud(900.1,True))
                if name=='both_restart':n.pair(900.1);n.imus(900.1,900.205)
                n.spin(.1);assert len(n.rows)==before
                # Fresh valid-looking data cannot undo a latched epoch failure.
                n.pair(1001.8);n.imus(1001.8,1001.905);n.spin(.3);assert len(n.rows)==before
            assert proc.poll() is None
            values=[]
            for m in n.rows:
                p=m.pose.pose;v=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
                assert all(math.isfinite(x) for x in v);values.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':v})
            result['cases'].append({'name':name,'warm_odoms':before,'final_odoms':len(n.rows),'samples':values})
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT);proc.wait(timeout=6)
            log.close();n.destroy_node()
        assert proc.returncode==0
finally:
    rclpy.shutdown();(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps([{'case':c['name'],'warm':c['warm_odoms'],'final':c['final_odoms']} for c in result['cases']]))
