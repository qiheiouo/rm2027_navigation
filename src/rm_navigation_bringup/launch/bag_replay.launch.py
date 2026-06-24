import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _start_bag(context, *args, **kwargs):
    if LaunchConfiguration("play_bag").perform(context).strip().lower() not in TRUE_VALUES:
        return [LogInfo(msg="[bag_replay] play_bag:=false; no rosbag process started.")]

    bag_path = LaunchConfiguration("bag_path").perform(context).strip()
    if not bag_path:
        raise RuntimeError("bag_path is required when play_bag:=true")
    if not os.path.exists(bag_path):
        raise RuntimeError(f"bag_path does not exist inside the runtime container: {bag_path}")

    command = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--clock",
        "--rate",
        LaunchConfiguration("bag_rate"),
    ]
    if LaunchConfiguration("loop_bag").perform(context).strip().lower() in TRUE_VALUES:
        command.append("--loop")

    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    replay_level = LaunchConfiguration("replay_level")
    sensor_mode = LaunchConfiguration("sensor_mode")
    global_localization_mode = LaunchConfiguration("global_localization_mode")
    selected_side = LaunchConfiguration("selected_side")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_chassis_stub = LaunchConfiguration("use_chassis_stub")
    use_rviz = LaunchConfiguration("use_rviz")
    nav2_params = LaunchConfiguration("nav2_params")

    navigation_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "navigation.launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1_5_mppi.yaml",
    ])
    run_lio_backend = PythonExpression(["'", replay_level, "' == 'raw_sensors'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "replay_level",
            default_value="raw_odom",
            choices=["raw_sensors", "raw_odom"],
            description=(
                "raw_sensors runs FAST-LIO; raw_odom expects the bag to provide "
                "/odometry/fast_lio_raw. Canonical odometry/TF must not be in the bag."
            ),
        ),
        DeclareLaunchArgument("bag_path", default_value=""),
        DeclareLaunchArgument("play_bag", default_value="false"),
        DeclareLaunchArgument("loop_bag", default_value="false"),
        DeclareLaunchArgument("bag_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "sensor_mode",
            default_value="single",
            choices=["single", "dual"],
        ),
        DeclareLaunchArgument(
            "global_localization_mode",
            default_value="stub",
            choices=["stub", "external_pose"],
        ),
        DeclareLaunchArgument(
            "selected_side",
            default_value="left",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_chassis_stub", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        LogInfo(msg=[
            "[bag_replay] replay_level=",
            replay_level,
            ". Driver is forced off; canonical TF remains owned by project adapters.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "sensor_mode": sensor_mode,
                "selected_side": selected_side,
                "global_localization_mode": global_localization_mode,
                "use_driver": "false",
                "use_lio_backend": run_lio_backend,
                "use_nav2": use_nav2,
                "use_chassis_stub": use_chassis_stub,
                "use_rviz": use_rviz,
                "use_sim_time": "true",
                "nav2_params": nav2_params,
            }.items(),
        ),
        OpaqueFunction(function=_start_bag),
    ])
