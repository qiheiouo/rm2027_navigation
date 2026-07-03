from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    selected_side = LaunchConfiguration("selected_side")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_map_odom_stub = LaunchConfiguration("use_map_odom_stub")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_serial_dry_run = LaunchConfiguration("use_serial_dry_run")
    serial_protocol_profile = LaunchConfiguration("serial_protocol_profile")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")
    nav2_params = LaunchConfiguration("nav2_params")

    old_description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "old_car_2026_description.launch.py",
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
    old_fast_lio_config = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "config",
        "fast_lio_multi_old_car_2026.yaml",
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "launch",
        "localization_adapters.launch.py",
    ])
    old_lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "config",
        "lio_adapter_old_car_2026.yaml",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase2g_pointcloud.yaml",
    ])
    serial_dry_run_launch = PathJoinSubstitution([
        FindPackageShare("rm_serial_driver"),
        "launch",
        "serial_dry_run.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("selected_side", default_value="left"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_map_odom_stub", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_serial_dry_run", default_value="false"),
        DeclareLaunchArgument(
            "serial_protocol_profile",
            default_value="hpm_crc_v1",
            choices=["legacy_v1_no_crc", "hpm_crc_v1"],
        ),
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        LogInfo(msg=(
            "[old_car_2026_validation] Experiment-only bringup for the 2026 "
            "car. Safe defaults start no real driver, no FAST-LIO, no Nav2, "
            "and no serial transport."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(old_description_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_driver_launch),
            launch_arguments={
                "use_driver": use_driver,
                "side": selected_side,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_backend_launch),
            launch_arguments={
                "use_backend": use_lio_backend,
                "sensor_mode": "single",
                "selected_side": selected_side,
                "update_method": update_method,
                "config_file": old_fast_lio_config,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "raw_odom_topic": "/odometry/fast_lio_raw",
                "lio_adapter_config": old_lio_adapter_config,
                "use_map_odom_stub": use_map_odom_stub,
            }.items(),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_dry_run_launch),
            condition=IfCondition(use_serial_dry_run),
            launch_arguments={
                "protocol_profile": serial_protocol_profile,
                "cmd_vel_topic": "/cmd_vel",
                "mock_tx_topic": "/serial/mock_tx",
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
