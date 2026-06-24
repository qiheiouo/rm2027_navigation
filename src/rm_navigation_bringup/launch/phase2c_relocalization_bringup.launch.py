from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    global_localization_mode = LaunchConfiguration("global_localization_mode")
    sensor_mode = LaunchConfiguration("sensor_mode")
    selected_side = LaunchConfiguration("selected_side")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")

    phase2a_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2a_lio_bringup.launch.py",
    ])
    global_pose_bridge_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "launch",
        "global_pose_bridge.launch.py",
    ])

    stub_mode = PythonExpression([
        "'", global_localization_mode, "' == 'stub'",
    ])
    external_pose_mode = PythonExpression([
        "'", global_localization_mode, "' == 'external_pose'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "global_localization_mode",
            default_value="stub",
            choices=["stub", "external_pose"],
            description=(
                "Exactly one map->odom owner: Phase 1 stub or the canonical "
                "external global-pose bridge."
            ),
        ),
        DeclareLaunchArgument("sensor_mode", default_value="single"),
        DeclareLaunchArgument("selected_side", default_value="left"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        LogInfo(msg=[
            "Phase 2C global localization mode: ",
            global_localization_mode,
            ". Stub and external-pose bridge are mutually exclusive.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phase2a_launch),
            launch_arguments={
                "sensor_mode": sensor_mode,
                "selected_side": selected_side,
                "use_driver": use_driver,
                "use_lio_backend": use_lio_backend,
                "update_method": update_method,
                "use_rviz": use_rviz,
                "use_sim_time": use_sim_time,
                "use_map_odom_stub": stub_mode,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(global_pose_bridge_launch),
            condition=IfCondition(external_pose_mode),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])
