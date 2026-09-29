"""Simulation-only paired surface observation; no Nav2 or robot commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world_file")
    simulation = FindPackageShare("rm_simulation")
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    description = PathJoinSubstitution([FindPackageShare("rm_description"), "launch", "description.launch.py"])
    localization = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"), "launch", "localization_adapters.launch.py"])
    walls = [Node(package="ros_gz_sim", executable="create", name=f"spawn_wall_{side}",
                  arguments=["-world", "phase1_omni", "-file",
                             PathJoinSubstitution([simulation, "models", "course_wall.sdf"]),
                             "-name", f"course_wall_{side}", "-x", "3.0", "-y", y,
                             "-z", "0.0"], output="screen")
             for side, y in (("north", "0.525"), ("south", "-0.525"))]
    moving = Node(package="ros_gz_sim", executable="create", name="spawn_moving_obstacle",
                  arguments=["-world", "phase1_omni", "-file",
                             PathJoinSubstitution([simulation, "models", "moving_obstacle.sdf"]),
                             "-name", "moving_obstacle", "-x", "4.9", "-y", "0.0",
                             "-z", "0.0"], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("world_file"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": ["-r -s --headless-rendering ", world]}.items()),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="simulation_bridge",
             parameters=[{"config_file": PathJoinSubstitution([
                 simulation, "config", "ros_gz_bridge.yaml"])}], output="screen"),
        Node(package="rm_simulation", executable="scan_frame_adapter", name="scan_frame_adapter",
             parameters=[{"use_sim_time": True, "input_topic": "/simulation/scan_raw",
                          "output_topic": "/scan", "output_frame": "sim_lidar_link"}],
             output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description),
                                 launch_arguments={"use_sim_time": "true",
                                                   "use_sim_lidar": "true"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization),
                                 launch_arguments={"use_sim_time": "true",
                                                   "raw_odom_topic": "/simulation/ground_truth/odom"}.items()),
        TimerAction(period=2.0, actions=walls),
        TimerAction(period=3.0, actions=[moving,
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 name="course_dynamic_bridge", parameters=[{"config_file":
                     PathJoinSubstitution([simulation, "config", "course_dynamic_bridge.yaml"])}],
                 output="screen")]),
        TimerAction(period=4.0, actions=[
            Node(package="rm_simulation", executable="moving_obstacle_controller",
                 name="moving_obstacle_controller", parameters=[{
                     "use_sim_time": True, "amplitude": 0.9, "period": 8.0}],
                 output="screen")]),
    ])
