from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    phase2a_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2a_lio_bringup.launch.py",
    ])

    return LaunchDescription([
        LogInfo(msg=[
            "Phase 2B no-hardware test: fake sensor-frame pose -> lio_adapter ",
            "finite-difference base twist. No FAST-LIO, driver, Nav2, or serial."
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phase2a_launch),
            launch_arguments={
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
            name="phase2b_fake_lio_odom_publisher",
            output="screen",
            parameters=[{
                "output_topic": "/odometry/fast_lio_raw",
                "cmd_vel_topic": "/cmd_vel",
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
    ])
