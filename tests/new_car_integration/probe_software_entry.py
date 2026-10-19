"""Humble software entry checks; no Nav2-to-serial chain or hardware authority."""
import json,math,os,signal,struct,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import SerialConnectionState

def crc(data):
    v=65535
    for b in data:
        v^=b
        for _ in range(8):v=(v>>1)^0xa001 if v&1 else v>>1
    return v

out=Path(os.environ['ENTRY_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=Node('newcar_entry_probe');frames=[];connection=[];odoms=[]
def tx(m):
    b=bytes(m.data);assert crc(b[2:-2])==struct.unpack_from('<H',b,len(b)-2)[0]
    if b[3]==1:frames.append((time.monotonic(),struct.unpack('<fffB',b[8:-2])))
n.create_subscription(UInt8MultiArray,'/serial/v2/mock_tx',tx,100)
n.create_subscription(SerialConnectionState,'/serial/connection_state',connection.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Odometry,'/odometry/fast_lio_raw',odoms.append,10)
pub=n.create_publisher(Twist,'/cmd_vel',1)
process=None;logs=[];result={'scope':'isolated Humble software entry; MCU fixture and codec only; no hardware authorization'}
def spin(sec,command=None,predicate=None):
    end=time.monotonic()+sec;last=0.
    while time.monotonic()<end:
        if command and time.monotonic()-last>.04:pub.publish(command);last=time.monotonic()
        rclpy.spin_once(n,timeout_sec=.005)
        if predicate and predicate():return True
    return predicate is None or predicate()
def start(mode,enable=True):
    global process
    f=(out/(mode+'.log')).open('w');logs.append(f)
    process=subprocess.Popen(['ros2','launch','rm_navigation_bringup','new_car_software.launch.py','mode:='+mode,'enable_test:='+str(enable).lower()],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
def stop():
    global process
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    process=None
try:
    start('inventory',False);assert spin(6,predicate=lambda:process.poll() is not None);assert process.returncode==0;stop();result['inventory']=True
    start('bundle_no_input',False);assert spin(6,predicate=lambda:process.poll() is not None);assert process.returncode!=0;stop();result['opt_in_required']=True
    start('hardware');assert spin(6,predicate=lambda:process.poll() is not None);assert process.returncode!=0;stop();result['hardware_mode_rejected']=True
    start('bundle_no_input');assert spin(10,predicate=lambda:any(name=='fast_lio_multi_backend' for name,ns in n.get_node_names_and_namespaces()))
    spin(1);result['bundle_no_input']={'raw_odom_count':len(odoms),'nodes':n.get_node_names_and_namespaces()};assert not odoms
    stop();spin(.5)
    from ament_index_python.packages import get_package_prefix
    exe=Path(get_package_prefix('rm_localization_adapters'))/'lib/rm_localization_adapters/lio_adapter'
    p=subprocess.run([str(exe),'--ros-args','--params-file','src/rm_lio_bringup/config/lio_adapter_chassis_heading_fusion.yaml'],capture_output=True,text=True,timeout=8)
    (out/'alignment_rejection.log').write_text(p.stdout+p.stderr);assert p.returncode!=0 and 'initial_alignment_confirmed=true' in p.stdout+p.stderr
    result['unmeasured_alignment_rejected']=True
    start('v2_protocol');assert spin(12,predicate=lambda:connection and connection[-1].online)
    spin(.4);result['protocol_initial_disabled']=all(v[3]==0 for t,v in frames if t>time.monotonic()-.3);assert result['protocol_initial_disabled']
    command=Twist();command.linear.x=100.;command.linear.y=-100.;command.angular.z=100.
    spin(1.2,command);vs=[v for t,v in frames if t>time.monotonic()-.4]
    result['protocol_generic_cmd_rejected']={'frames':len(vs),'last':vs[-1]}
    assert len(vs)>20 and all(v==(0.,0.,0.,0) for v in vs)
    spin(.7);vs=[v for t,v in frames if t>time.monotonic()-.4];assert len(vs)>20 and all(v==(0.,0.,0.,0) for v in vs)
    result['protocol_guard_remains_disabled']=True
    result['protocol_graph']=n.get_node_names_and_namespaces();assert not any('controller_server'==name or 'mission' in name for name,ns in result['protocol_graph'])
    stop();result['normal_exit']=True;result['pass']=True
except Exception as e:result['pass']=False;result['error']=repr(e);raise
finally:
    stop()
    for f in logs:f.close()
    n.destroy_node();rclpy.shutdown();(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
