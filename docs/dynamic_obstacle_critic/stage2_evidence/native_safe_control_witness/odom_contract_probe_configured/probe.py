import json,subprocess
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import ChangeState
rclpy.init();node=rclpy.create_node('configured_controller_odom_probe');root=Path('/out/odom_contract_probe_configured')
log=(root/'controller.log').open('w')
process=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file','/out/gazebo_native_cycle_evidence/profile.yaml','-p','use_sim_time:=false','-p','FollowPath.NativeCycleSnapshotCritic.enabled:=false'],stdout=log,stderr=subprocess.STDOUT)
try:
 lifecycle=node.create_client(ChangeState,'/controller_server/change_state')
 if not lifecycle.wait_for_service(timeout_sec=8):raise RuntimeError('lifecycle service unavailable')
 req=ChangeState.Request();req.transition.id=1
 f=lifecycle.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=15)
 if not f.done():raise RuntimeError('configure timeout')
 configured=f.result().success
 client=node.create_client(GetParameters,'/controller_server/get_parameters');client.wait_for_service(timeout_sec=2)
 values={}
 for name in ['odom_topic','controller_frequency','min_x_velocity_threshold','min_y_velocity_threshold','min_theta_velocity_threshold']:
  req=GetParameters.Request();req.names=[name];future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=2)
  vals=future.result().values if future.done() else []
  values[name]=[{'type':v.type,'string':v.string_value,'double':v.double_value} for v in vals]
 result={'scope':'actual pinned controller configured but never activated; no goals or commands','configured':configured,'parameters':values,
 'subscriptions':node.get_subscriber_names_and_types_by_node('controller_server','/')}
 (root/'readback.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:
 process.terminate()
 try:process.wait(timeout=5)
 except subprocess.TimeoutExpired:process.kill();process.wait()
 log.close();node.destroy_node();rclpy.shutdown()
