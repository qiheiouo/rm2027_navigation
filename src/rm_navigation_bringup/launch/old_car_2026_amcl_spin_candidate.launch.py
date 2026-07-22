"""No-motion field-validation entry point for the AMCL spin-robust candidate.

This wrapper cannot start Nav2, serial transport, a controller, or a mission. It
only selects the frozen alpha4=0.02 and strict SE(3) scan profile for explicit
localization observation. The normal old-car and competition launches keep
their baseline defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    relocalization_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "old_car_2026_amcl_relocalization.launch.py",
    ])
    candidate_amcl = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d_spin_robust_candidate.yaml",
    ])
    candidate_projection = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_relocalization", default_value="true"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_bundle_manifest", default_value=""),
        DeclareLaunchArgument(
            "map_acceptance_policy",
            default_value="allow_candidate",
            choices=["approved_only", "allow_candidate", "allow_test"],
        ),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        LogInfo(msg=[
            "[amcl_spin_candidate] EXPERIMENTAL no-motion validation only: ",
            "alpha4=0.02 + strict per-point SE(3) deskew. ",
            "Serial, controller, mission and Nav2 motion remain disabled.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(relocalization_launch),
            launch_arguments={
                "enable_relocalization": LaunchConfiguration("enable_relocalization"),
                "use_driver": LaunchConfiguration("use_driver"),
                "use_lio_backend": LaunchConfiguration("use_lio_backend"),
                "selected_side": "left",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_bundle_manifest": LaunchConfiguration("map_bundle_manifest"),
                "map_acceptance_policy": LaunchConfiguration("map_acceptance_policy"),
                "pointcloud_topic": "/livox/left/pointcloud_filtered",
                "amcl_params_file": candidate_amcl,
                "scan_projection_params": candidate_projection,
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            }.items(),
        ),
    ])
