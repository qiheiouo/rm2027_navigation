from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    scene = Path(os.environ["TEMPORAL_MPC_SCENE"])
    def include(package, name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package))/"launch"/name)),
            launch_arguments=arguments.items())
    return LaunchDescription([
        include("ros_gz_sim","gz_sim.launch.py",{"gz_args":f"-r -s --headless-rendering {scene/'world.sdf'}"}),
        Node(package="ros_gz_bridge",executable="parameter_bridge",name="simulation_bridge",
             parameters=[{"config_file":str(scene/"bridge.yaml")}]),
        Node(package="rm_simulation",executable="scan_frame_adapter",parameters=[{
            "use_sim_time":True,"input_topic":"/simulation/scan_raw","output_topic":"/scan",
            "output_frame":"sim_lidar_link"}]),
        include("rm_description","description.launch.py",{"use_sim_time":"true","use_sim_lidar":"true"}),
        include("rm_localization_adapters","localization_adapters.launch.py",{
            "use_sim_time":"true","raw_odom_topic":"/simulation/ground_truth/odom", "use_map_odom_stub":"false"}),
        # Authored simulation map has a known fixed origin in the odom world.
        Node(package="tf2_ros",executable="static_transform_publisher",name="experiment_map_origin",
             arguments=["--x","0","--y","0","--z","0","--yaw","0","--pitch","0","--roll","0",
                        "--frame-id","map","--child-frame-id","odom"],parameters=[{"use_sim_time":True}]),
        Node(package="rm_chassis_interface",executable="chassis_interface_stub",parameters=[{
            "use_sim_time":True,"cmd_vel_topic":"/cmd_vel",
            "mock_output_cmd_vel_topic":"/simulation/chassis/cmd_vel"}]),
    ])
