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

    return LaunchDescription([
        LogInfo(msg=[
            "Phase 2C no-hardware test: fake LIO + synchronized fake global ",
            "pose -> canonical map->odom. No small_gicp, driver, Nav2, or serial.",
        ]),
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
            name="phase2c_fake_lio_odom_publisher",
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
        Node(
            package="rm_relocalization_bridge",
            executable="fake_global_pose_publisher",
            name="phase2c_fake_global_pose_publisher",
            output="screen",
            parameters=[{
                "odom_topic": "/odometry/lio",
                "output_topic": "/localization/global_pose",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_divider": 10,
                "max_publications": 1,
                "startup_delay_sec": 1.0,
                "map_to_odom.x": 3.0,
                "map_to_odom.y": -1.0,
                "map_to_odom.z": 0.0,
                "map_to_odom.roll": 0.0,
                "map_to_odom.pitch": 0.0,
                "map_to_odom.yaw": 0.35,
            }],
        ),
    ])
