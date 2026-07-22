"""High-risk three-point patrol and Spin field-test entry point.

Safe defaults start nothing. The caller must opt into every hardware and motion
layer, provide the fresh03 candidate manifest, publish an initial pose, verify
localization in RViz, and enable the mission by service.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    competition_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "old_car_2026_competition.launch.py",
    ])
    nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_left_stvl_three_point_spin_test.yaml",
    ])
    amcl_params = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d_spin_robust_candidate.yaml",
    ])
    scan_params = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
    ])
    mission_config = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "config",
        "mission_fresh03_three_point_spin_test.yaml",
    ])
    mission_tree = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "trees",
        "competition_three_point_spin_test.xml",
    ])

    enable_stack = LaunchConfiguration("enable_competition_stack")
    use_driver = LaunchConfiguration("use_driver")
    use_lio = LaunchConfiguration("use_lio_backend")
    use_map_server = LaunchConfiguration("use_map_server")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_real_serial = LaunchConfiguration("use_real_serial")
    use_referee = LaunchConfiguration("use_referee_interface")
    use_referee_mock = LaunchConfiguration("use_referee_mock")
    use_mission = LaunchConfiguration("use_mission")
    allow_field_debug = LaunchConfiguration("allow_field_debug_inputs")
    use_operator_authority = LaunchConfiguration("use_operator_chassis_authority")
    use_chassis_mode = LaunchConfiguration("use_chassis_mode_interface")
    use_rviz = LaunchConfiguration("use_rviz")
    serial_device = LaunchConfiguration("serial_device")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    map_manifest = LaunchConfiguration("map_bundle_manifest")

    return LaunchDescription([
        DeclareLaunchArgument("enable_competition_stack", default_value="false"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_map_server", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_real_serial", default_value="false"),
        DeclareLaunchArgument("use_referee_interface", default_value="false"),
        DeclareLaunchArgument("use_referee_mock", default_value="false"),
        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("allow_field_debug_inputs", default_value="false"),
        DeclareLaunchArgument("use_operator_chassis_authority", default_value="false"),
        DeclareLaunchArgument("use_chassis_mode_interface", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("map_bundle_manifest", default_value=""),
        LogInfo(msg=(
            "[three_point_spin_test] HIGH-RISK candidate: requested limits are "
            "vx/vy=3 m/s and wz=10 rad/s. Safe defaults start no hardware, "
            "serial, Nav2, or mission; mission startup remains disabled."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(competition_launch),
            launch_arguments={
                "enable_competition_stack": enable_stack,
                "use_driver": use_driver,
                "use_lio_backend": use_lio,
                "use_map_server": use_map_server,
                "relocalization_backend": "amcl_2d",
                "map_bundle_manifest": map_manifest,
                "map_acceptance_policy": "allow_candidate",
                "relocalization_params": amcl_params,
                "scan_projection_params": scan_params,
                "use_nav2": use_nav2,
                "nav2_params": nav2_params,
                "use_real_serial": use_real_serial,
                "serial_device": serial_device,
                "serial_baudrate": serial_baudrate,
                "serial_max_vx": "3.0",
                "serial_max_vy": "3.0",
                "serial_max_wz": "10.0",
                "use_rviz": use_rviz,
                "use_referee_interface": use_referee,
                "use_referee_mock": use_referee_mock,
                "use_mission": use_mission,
                "mission_startup_enabled": "false",
                "mission_config": mission_config,
                "mission_tree_xml": mission_tree,
                "allow_field_debug_inputs": allow_field_debug,
                "use_operator_chassis_authority": use_operator_authority,
                "use_chassis_mode_interface": use_chassis_mode,
                "use_mission_safety_mock": "false",
                "use_pursuit": "false",
                "use_target_mock": "false",
                "use_dual_obstacle_fusion": "false",
                "use_right_driver": "false",
            }.items(),
        ),
    ])
