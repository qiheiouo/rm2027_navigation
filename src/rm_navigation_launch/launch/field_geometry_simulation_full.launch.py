"""Launch the high-performance RMUC 2026 field simulation profile."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lightweight_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_launch"),
        "launch",
        "field_geometry_simulation.launch.py",
    ])
    dog_hole_config = PathJoinSubstitution([
        FindPackageShare("rm_dog_hole"),
        "config",
        "dog_hole_sim.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument(
            "robot_geometry_profile",
            default_value="deformed",
            choices=["deformed", "undeformed"],
        ),
        DeclareLaunchArgument(
            "gimbal_motion_mode",
            default_value="continuous",
            choices=["fixed", "sine", "continuous"],
        ),
        DeclareLaunchArgument(
            "gimbal_angular_velocity", default_value="0.60"
        ),
        DeclareLaunchArgument("active_ramp_filter", default_value="true"),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="path_aligned",
            choices=["baseline", "path_aligned"],
        ),
        DeclareLaunchArgument(
            "dog_hole_config", default_value=dog_hole_config
        ),
        LogInfo(msg=[
            "[field_geometry_simulation_full] High-performance profile: ",
            "720 beams at 15 Hz, 1 ms physics, 0.05 m global costmap, ",
            "MPPI batch 1000. The runtime field geometry is shared with ",
            "the lightweight entry; the original STEP remains disabled.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lightweight_launch),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "auto_start": LaunchConfiguration("auto_start"),
                "robot_geometry_profile": LaunchConfiguration(
                    "robot_geometry_profile"
                ),
                "gimbal_motion_mode": LaunchConfiguration(
                    "gimbal_motion_mode"
                ),
                "gimbal_angular_velocity": LaunchConfiguration(
                    "gimbal_angular_velocity"
                ),
                "active_ramp_filter": LaunchConfiguration(
                    "active_ramp_filter"
                ),
                "heading_policy": LaunchConfiguration("heading_policy"),
                "dog_hole_config": LaunchConfiguration("dog_hole_config"),
                "global_costmap_update_frequency": "2.0",
                "global_costmap_resolution": "0.05",
                "mppi_batch_size": "1000",
                "physics_max_step_size": "0.001",
                "lidar_update_rate": "15.0",
                "lidar_samples": "720",
                "bt_loop_duration": "10",
            }.items(),
        ),
    ])
