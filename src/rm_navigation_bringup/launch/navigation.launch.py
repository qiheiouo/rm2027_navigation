from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_mode = LaunchConfiguration("sensor_mode")
    selected_side = LaunchConfiguration("selected_side")
    global_localization_mode = LaunchConfiguration("global_localization_mode")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_chassis_stub = LaunchConfiguration("use_chassis_stub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")
    nav2_params = LaunchConfiguration("nav2_params")
    rviz_config = LaunchConfiguration("rviz_config")

    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2c_relocalization_bringup.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    chassis_launch = PathJoinSubstitution([
        FindPackageShare("rm_chassis_interface"),
        "launch",
        "chassis_interface_stub.launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1_5_mppi.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_mode",
            default_value="single",
            choices=["single", "dual"],
            description="Single MID360 is the safe default until dual-sensor calibration.",
        ),
        DeclareLaunchArgument(
            "selected_side",
            default_value="left",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "global_localization_mode",
            default_value="stub",
            choices=["stub", "external_pose"],
            description="Select exactly one map->odom owner.",
        ),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument(
            "use_nav2",
            default_value="false",
            description=(
                "Disabled by default until the real obstacle input and deployment "
                "map profile are selected."
            ),
        ),
        DeclareLaunchArgument("use_chassis_stub", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "update_method",
            default_value="bundle",
            choices=["bundle", "async", "adaptive"],
        ),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        LogInfo(msg=[
            "[navigation] Canonical stack entry. Hardware, FAST-LIO, and Nav2 ",
            "remain explicit opt-ins. global_localization_mode=",
            global_localization_mode,
            ", sensor_mode=",
            sensor_mode,
            ".",
        ]),
        LogInfo(msg=(
            "[navigation] The default Nav2 file is the simulation-proven MPPI "
            "baseline, not a real-robot deployment acceptance. Real 3D obstacle "
            "input and an approved map bundle are still required."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "sensor_mode": sensor_mode,
                "selected_side": selected_side,
                "global_localization_mode": global_localization_mode,
                "use_driver": use_driver,
                "use_lio_backend": use_lio_backend,
                "update_method": update_method,
                "use_rviz": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(chassis_launch),
            condition=IfCondition(use_chassis_stub),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
                "autostart": "true",
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
