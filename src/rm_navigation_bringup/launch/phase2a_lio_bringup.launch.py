from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_mode = LaunchConfiguration("sensor_mode")
    selected_side = LaunchConfiguration("selected_side")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_map_odom_stub = LaunchConfiguration("use_map_odom_stub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")

    description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "description.launch.py",
    ])
    dual_driver_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "launch",
        "dual_mid360_driver.launch.py",
    ])
    single_driver_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "launch",
        "single_mid360_driver.launch.py",
    ])
    lio_backend_launch = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "launch",
        "fast_lio_multi_phase2a.launch.py",
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "launch",
        "localization_adapters.launch.py",
    ])
    default_lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "config",
        "lio_adapter_fast_lio_multi.yaml",
    ])
    lio_adapter_config = LaunchConfiguration("lio_adapter_config")
    default_gimbal_state_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "gimbal_state_adapter.yaml",
    ])
    gimbal_state_adapter_config = LaunchConfiguration("gimbal_state_adapter_config")
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    single_condition = IfCondition(
        PythonExpression(["'", sensor_mode, "' == 'single'"])
    )
    dual_condition = IfCondition(
        PythonExpression(["'", sensor_mode, "' == 'dual'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument("sensor_mode", default_value="single"),
        DeclareLaunchArgument("selected_side", default_value="left"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_map_odom_stub", default_value="true"),
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "lio_adapter_config",
            default_value=default_lio_adapter_config,
        ),
        DeclareLaunchArgument(
            "gimbal_state_adapter_config",
            default_value=default_gimbal_state_adapter_config,
        ),
        LogInfo(msg=[
            "Phase 2A LIO boundary. Safe defaults keep MID360 driver and ",
            "FAST-LIO disabled. No serial, referee, Nav2, or competition BT."
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_driver_launch),
            condition=single_condition,
            launch_arguments={
                "use_driver": use_driver,
                "side": selected_side,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_driver_launch),
            condition=dual_condition,
            launch_arguments={
                "use_driver": use_driver,
                "lio_imu_source": selected_side,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_backend_launch),
            launch_arguments={
                "use_backend": use_lio_backend,
                "sensor_mode": sensor_mode,
                "selected_side": selected_side,
                "update_method": update_method,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "raw_odom_topic": "/odometry/fast_lio_raw",
                "lio_adapter_config": lio_adapter_config,
                "gimbal_state_adapter_config": gimbal_state_adapter_config,
                "use_map_odom_stub": use_map_odom_stub,
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
