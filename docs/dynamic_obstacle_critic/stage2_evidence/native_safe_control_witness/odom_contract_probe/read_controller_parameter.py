import json,time,subprocess
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init();node=rclpy.create_node('read_controller_odom_contract')
log=Path('/out/odom_contract_probe/controller.log').open('w')
process=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file','/out/gazebo_native_cycle_evidence/profile.yaml','-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT)
try:
 client=node.create_client(GetParameters,'/controller_server/get_parameters')
 if not client.wait_for_service(timeout_sec=8):raise RuntimeError('controller parameter service unavailable')
 req=GetParameters.Request();req.names=['odom_topic','controller_frequency','min_x_velocity_threshold','min_y_velocity_threshold','min_theta_velocity_threshold']
 future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
 if not future.done():raise RuntimeError('controller parameter readback timeout')
 values={name:{'type':v.type,'string':v.string_value,'double':v.double_value} for name,v in zip(req.names,future.result().values)}
 result={'scope':'pinned installed controller constructor readback, no lifecycle configure/activate, no goals or commands','parameters':values,
 'subscriptions':node.get_subscriber_names_and_types_by_node('controller_server','/')}
 Path('/out/odom_contract_probe/readback.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:
 process.terminate()
 try:process.wait(timeout=5)
 except subprocess.TimeoutExpired:process.kill();process.wait()
 log.close();node.destroy_node();rclpy.shutdown()
