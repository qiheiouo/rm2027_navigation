from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    phase2c_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2c_relocalization_bringup.launch.py",
    ])
    amcl_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "launch",
        "amcl_2d_backend.launch.py",
    ])
    amcl_params = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d.yaml",
    ])

    return LaunchDescription([
        LogInfo(msg=(
            "Phase 2J no-hardware test: synthetic occupancy map and LaserScan -> "
            "AMCL pose gate -> Phase 2C map->odom bridge. No driver, Nav2, serial, "
            "referee, mission, or competition BT."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phase2c_launch),
            launch_arguments={
                "global_localization_mode": "external_pose",
                "sensor_mode": "single",
                "selected_side": "left",
                "use_driver": "false",
                "use_lio_backend": "false",
                "use_rviz": "false",
                "use_sim_time": "false",
            }.items(),
        ),
        Node(
            package="rm_localization_adapters",
            executable="fake_lio_odom_publisher",
            name="phase2j_fake_lio_odom_publisher",
            output="screen",
            parameters=[{
                "output_topic": "/odometry/fast_lio_raw",
                "cmd_vel_topic": "/phase2j/unused_cmd_vel",
                "odom_frame": "odom",
                "child_frame": "body",
                "motion_mode": "cmd_vel",
                "publish_rate_hz": 50.0,
                "cmd_vel_timeout_sec": 0.5,
                "sensor_offset.x": 0.0,
                "sensor_offset.y": 0.12,
                "sensor_offset.z": 0.35,
            }],
        ),
        Node(
            package="rm_relocalization_bridge",
            executable="fake_2d_localization_world",
            name="phase2j_fake_2d_localization_world",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch),
            launch_arguments={
                "enable_backend": "true",
                "params_file": amcl_params,
                "use_pointcloud_to_scan": "false",
                "scan_topic": "/localization/scan",
                "map_topic": "/map",
                "set_initial_pose": "true",
                "initial_pose_x": "0.0",
                "initial_pose_y": "0.0",
                "initial_pose_yaw": "0.0",
                "update_min_d": "0.0",
                "update_min_a": "0.0",
                "use_sim_time": "false",
            }.items(),
        ),
    ])
