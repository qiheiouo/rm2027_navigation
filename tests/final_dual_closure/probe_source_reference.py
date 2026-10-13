"""Actual RSP/gimbal adapter/lio adapter; synthetic SE(3) source input only.
Expected odom is base(t_ref)<-base(t), never an external heading estimate.
Original dynamic sensor_initial negative probe remains unchanged.
"""
import json, math, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import GimbalState
from tf2_ros import Buffer, TransformListener
ROOT=Path(__file__).resolve().parents[2]
out=Path(os.environ['DUAL_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
install=Path(os.environ['DUAL_INSTALL']);rclpy.init();n=Node('source_reference_probe')
buf=Buffer();listener=TransformListener(buf,n)
gimbal=n.create_publisher(GimbalState,'/gimbal/state',10)
raw=n.create_publisher(Odometry,'/odometry/fast_lio_raw',10);rows=[]
n.create_subscription(Odometry,'/odometry/lio',rows.append,100)
processes=[];logs=[];result={'scope':__doc__,'cases':[],'faults':[]}
def spin(sec):
    end=time.monotonic()+sec
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
def start(pkg,binary,args=()):
    f=(out/(binary+str(len(processes))+'.log')).open('w');logs.append(f)
    exe=('/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher' if pkg=='robot_state_publisher'
         else os.environ['DUAL_TF_PROBE'] if pkg=='tf_probe' else str(install/pkg/'lib'/pkg/binary))
    p=subprocess.Popen([exe,'--ros-args',*args],stdout=f,stderr=subprocess.STDOUT);processes.append(p);return p
def stop(p):
    if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=5)
def rot(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def pose_matrix(p):
    m=np.eye(4);m[:3,:3]=rot(p.orientation);m[:3,3]=[p.position.x,p.position.y,p.position.z];return m
# Fixture rotations are yaw-only; the actual RSP lever arm and vertical offset remain.
def base(yaw,xyz=(0.,0.,0.)):
    m=np.eye(4);c,s=math.cos(yaw),math.sin(yaw);m[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];m[:3,3]=xyz;return m
def mount(yaw):
    g=GimbalState();g.header.stamp=n.get_clock().now().to_msg();g.valid=g.online=True;g.relative_yaw_rad=yaw;gimbal.publish(g)
    end=time.monotonic()+1.
    while time.monotonic()<end:
        spin(.005)
        if buf.can_transform('base_link','lio_imu_link',Time.from_msg(g.header.stamp)):
            t=buf.lookup_transform('base_link','lio_imu_link',Time.from_msg(g.header.stamp)).transform
            m=np.eye(4);m[:3,:3]=rot(t.rotation);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return g.header.stamp,m
    raise AssertionError('missing source-time mechanical TF')
def message(ts,m):
    o=Odometry();o.header.stamp=ts;o.header.frame_id='odom';o.child_frame_id='lio_imu_link'
    o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,m[:3,3])
    yaw=math.atan2(m[1,0],m[0,0]);o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2)
    return o
def no_output(o,label):
    count=len(rows);raw.publish(o);spin(.08);assert len(rows)==count,(label,'unexpected output')
    result['faults'].append({'case':label,'new_odoms':len(rows)-count})
def accept(label,ts,b,g,g0,w,anchor=False):
    r=np.linalg.inv(w)@g@b;expected=np.linalg.inv(g0)@g
    mount(math.atan2(b[1,0],b[0,0])+.2) # latest TF must not contaminate source-time lookup
    before=len(rows);o=message(ts,r);raw.publish(o);spin(.08)
    if anchor:
        # Existing finite-difference policy intentionally consumes the first
        # pose without publishing until it has a valid dt. Keep that behavior.
        assert len(rows)==before,(label,'initial twist should not be fabricated')
        result.setdefault('anchors',[]).append({'case':label,'raw_pose':r.tolist(),
          'base_to_sensor':b.tolist(),'expected':expected.tolist(),'published':False})
        return o
    assert len(rows)>before,(label,'no output')
    actual=pose_matrix(rows[-1].pose.pose)
    pe=float(np.linalg.norm(actual[:3,3]-expected[:3,3]));de=float(np.linalg.norm(actual[:3,:3]-expected[:3,:3]))
    assert pe<1e-7 and de<1e-7,(label,pe,de)
    assert rows[-1].header.stamp==ts and rows[-1].child_frame_id=='base_link'
    result['cases'].append({'case':label,'source_stamp':[ts.sec,ts.nanosec],'base_world':g.tolist(),
      'base_reference_world':g0.tolist(),'raw_parent_world':w.tolist(),'base_to_sensor':b.tolist(),
      'raw_pose':r.tolist(),'expected':expected.tolist(),'actual':actual.tolist(),
      'position_error_m':pe,'rotation_matrix_error':de,'source_stamp_preserved':True})
    return o
try:
    urdf=subprocess.check_output(['xacro',str(ROOT/'src/rm_description/urdf/rm_sentry_2027.urdf.xacro')],text=True)
    config=out/'rsp.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':urdf,'publish_frequency':200.0}}}))
    start('robot_state_publisher','robot_state_publisher',['--params-file',str(config)])
    start('rm_localization_adapters','gimbal_state_adapter',['-p','use_input:=true']);spin(.7)
    result['tf_owner_counts_by_epoch']=[]
    previous=None
    for epoch,initial_gimbal,initial_yaw in [(0,0.,0.),(1,.35,.7)]:
        gid_path=out/f'epoch{epoch}_tf_gids.json'
        tfp=start('tf_probe','s2_tf_owner_probe',['-p','output:='+str(gid_path),'-p','edges:=[map>odom,odom>base_link,base_link>gimbal_yaw_link]'])
        pre_start,_=mount(initial_gimbal)
        p=start('rm_localization_adapters','lio_adapter',['--params-file',str(ROOT/'src/rm_lio_bringup/config/lio_adapter_dual_source_reference.yaml')]);spin(.3);rows.clear()
        # A replay generated before this adapter instance is never its reference.
        no_output(message(pre_start,np.eye(4)),f'epoch{epoch}_pre_start_replay')
        ts,b=mount(initial_gimbal)
        bad=message(ts,np.eye(4));bad.header.stamp.sec=0;bad.header.stamp.nanosec=0;no_output(bad,'zero_stamp')
        bad=message(ts,np.eye(4));bad.header.stamp.sec-=10;no_output(bad,'stale_first')
        bad=message(ts,np.eye(4));bad.header.stamp.sec+=10;no_output(bad,'future_first')
        g0=base(initial_yaw,(.4,.1,.2));w=base(.8,(1.2,-.8,.4))
        ts,b=mount(initial_gimbal);first=accept(f'epoch{epoch}_initialize_nonidentity',ts,b,g0,g0,w,anchor=True)
        ts,b=mount(initial_gimbal);accept(f'epoch{epoch}_initialize_first_output',ts,b,g0,g0,w)
        for label,by,gy in [('gimbal_only',0.,.6),('base_only',.4,0.),('same_direction',.4,.4),('opposite',.4,-.4)]:
            for repeat in range(3):
                ts,b=mount(initial_gimbal+gy);g=g0@base(by,(.05*repeat,0.,0.))
                previous=accept(f'epoch{epoch}_{label}_{repeat}',ts,b,g,g0,w)
        no_output(previous,'duplicate_source')
        no_output(first,'clock_rollback_blocks')
        ts,b=mount(.1);no_output(message(ts,np.linalg.inv(w)@g0@b),'fresh_after_block')
        assert p.poll() is None;stop(p);spin(.08)
        gids=json.loads(gid_path.read_text());assert len(gids.get('odom>base_link',[]))==1,gids
        assert len(gids.get('base_link>gimbal_yaw_link',[]))==1 and not gids.get('map>odom'),gids
        result['tf_owner_counts_by_epoch'].append({k:len(v) for k,v in gids.items()});stop(tfp)
finally:
    for p in reversed(processes):stop(p)
    for f in logs:f.close()
    n.destroy_node();rclpy.shutdown();(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'normal_samples':len(result['cases']),'fault_checks':result['faults'],'tf_owner_counts':result.get('tf_owner_counts_by_epoch')}))
