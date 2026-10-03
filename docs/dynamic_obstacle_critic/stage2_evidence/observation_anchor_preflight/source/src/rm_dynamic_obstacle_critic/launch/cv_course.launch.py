"""Opt-in Gazebo course: baseline MPPI + CV critic + final command guard."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = FindPackageShare("rm_dynamic_obstacle_critic")
    sim = FindPackageShare("rm_simulation")
    nav = FindPackageShare("nav2_bringup")
    params = LaunchConfiguration("params_file")
    enabled = LaunchConfiguration("enabled")
    use_guard = LaunchConfiguration("guard_enabled")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_cv_experiment.yaml"])),
        DeclareLaunchArgument("guard_params_file", default_value=PathJoinSubstitution([pkg, "config", "guard.yaml"])),
        DeclareLaunchArgument("guard_enabled", default_value="true"),
        DeclareLaunchArgument("nav2_autostart", default_value="true"),
        DeclareLaunchArgument("moving_period", default_value="8.0"),
        DeclareLaunchArgument("moving_amplitude", default_value="0.9"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([sim, "launch", "phase1_5_gazebo.launch.py"])),
            condition=IfCondition(enabled),
            launch_arguments={"headless": "true", "use_nav2": "false", "use_rviz": "false"}.items()),
        GroupAction(condition=IfCondition(enabled), actions=[
            # Node-qualified rules precede Nav2's own remaps; remapping the destination
            # /cmd_vel would not recursively remap cmd_vel_smoothed:=cmd_vel.
            SetRemap(src="behavior_server:cmd_vel", dst="/cmd_vel_nav"),
            SetRemap(src="velocity_smoother:cmd_vel_smoothed", dst=PythonExpression(["'/dynamic_test/cmd_vel_smoothed' if '", use_guard, "' == 'true' else '/cmd_vel'"])),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([nav, "launch", "navigation_launch.py"])),
                launch_arguments={"use_sim_time": "true", "params_file": params, "autostart": LaunchConfiguration("nav2_autostart"), "use_composition": "False"}.items()),
        ]),
        Node(condition=IfCondition(enabled), package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[{"use_sim_time": True, "yaml_filename": PathJoinSubstitution([pkg, "config", "course_static.yaml"])}]),
        Node(condition=IfCondition(enabled), package="nav2_lifecycle_manager", executable="lifecycle_manager", name="dynamic_map_lifecycle",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}]),
        Node(condition=IfCondition(enabled), package="rm_dynamic_obstacle_tracking", executable="dynamic_obstacle_tracker_node", name="dynamic_obstacle_tracker_shadow",
            parameters=[PathJoinSubstitution([pkg, "config", "tracker_cv.yaml"]), {"use_sim_time": True}]),
        Node(condition=IfCondition(PythonExpression(["'",enabled,"' == 'true' and '",use_guard,"' == 'true'"])), package="rm_dynamic_obstacle_critic", executable="dynamic_safety_guard", name="dynamic_safety_guard",
            parameters=[LaunchConfiguration("guard_params_file"), {"use_sim_time": True, "output_topic": "/cmd_vel"}]),
        TimerAction(period=3.0, actions=[
            Node(condition=IfCondition(enabled), package="ros_gz_sim", executable="create", name="spawn_moving_obstacle",
                arguments=["-world", "phase1_omni", "-file", PathJoinSubstitution([sim,"models","moving_obstacle.sdf"]), "-name","moving_obstacle","-x","4.9","-y","0.0","-z","0.0"]),
            Node(condition=IfCondition(enabled), package="ros_gz_bridge", executable="parameter_bridge", name="course_dynamic_bridge",
                parameters=[{"config_file": PathJoinSubstitution([sim,"config","course_dynamic_bridge.yaml"])}]),
        ]),
        TimerAction(period=4.0, actions=[Node(condition=IfCondition(enabled), package="rm_simulation", executable="moving_obstacle_controller", name="moving_obstacle_controller",
            parameters=[{"use_sim_time": True, "amplitude": ParameterValue(LaunchConfiguration("moving_amplitude"),value_type=float),
                         "period": ParameterValue(LaunchConfiguration("moving_period"),value_type=float)}])]),
    ])
