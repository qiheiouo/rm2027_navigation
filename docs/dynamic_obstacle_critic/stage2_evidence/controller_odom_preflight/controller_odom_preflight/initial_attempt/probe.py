import json,subprocess,sys,time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
sys.path.insert(0,'/repo/src/rm_dynamic_obstacle_critic/tools')
from controller_odom_preflight import ControllerOdomPreflight
rclpy.init();node=rclpy.create_node('controller_odom_fixture')
# This isolated fixture advertises canonical odometry, without TF or commands.
publisher=node.create_publisher(Odometry,'/odometry/lio',10)
root=Path('/out/controller_odom_preflight');cases=[]
try:
 for name,profile in [('legacy','/repo/src/rm_dynamic_obstacle_critic/config/nav2_cv_native_cycle_snapshot.yaml'),('canonical','/repo/src/rm_dynamic_obstacle_critic/config/nav2_cv_controller_odom.yaml')]:
  log=(root/(name+'.log')).open('w')
  process=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',profile,'-p','use_sim_time:=false','-p','FollowPath.NativeCycleSnapshotCritic.enabled:=false'],stdout=log,stderr=subprocess.STDOUT)
  try:
   client=node.create_client(ChangeState,'/controller_server/change_state')
   if not client.wait_for_service(timeout_sec=8):raise RuntimeError('configure service unavailable')
   request=ChangeState.Request();request.transition.id=1
   future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=15)
   if not future.done() or not future.result().success:raise RuntimeError('actual configure failed')
   check=ControllerOdomPreflight(node,'/odometry/lio')
   result=None;error=None
   while result is None and error is None:
    rclpy.spin_once(node,timeout_sec=.02)
    try:result=check.poll()
    except RuntimeError as failure:error=str(failure)
   case={'profile':name,'configured':True,'verdict':'PASS' if result else 'REJECTED','readback':result or check.evidence,'error':error,'lifecycle':'inactive, never activated','goals_sent':0,'commands_published':0}
   cases.append(case)
   assert (case['verdict']=='PASS')==(name=='canonical')
  finally:
   process.terminate()
   try:process.wait(timeout=5)
   except subprocess.TimeoutExpired:process.kill();process.wait()
   log.close();node.destroy_client(client)
   until=time.monotonic()+1
   while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
 result={'verdict':'PASS','cases':cases,'scope':'actual pinned controller lifecycle configure, parameter and subscriber checks; no activation, goal, motion or TF publication; effective speed still requires full physical capture'}
 (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:node.destroy_node();rclpy.shutdown()
