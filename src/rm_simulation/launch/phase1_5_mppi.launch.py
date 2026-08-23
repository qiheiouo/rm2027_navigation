from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    heading_policy = LaunchConfiguration("heading_policy")

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_gazebo.launch.py",
    ])
    mppi_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1_5_mppi.yaml",
    ])
    path_aligned_params = RewrittenYaml(
        source_file=mppi_params,
        param_rewrites={
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.forward_preference": "true",
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.cost_weight": "6.0",
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.max_angle_to_furthest": "0.20",
            "controller_server.ros__parameters.FollowPath."
            "TwirlingCritic.enabled": "true",
            "controller_server.ros__parameters.FollowPath."
            "PreferForwardCritic.enabled": "true",
        },
        convert_types=True,
    )
    baseline_mode = IfCondition(
        PythonExpression(["'", heading_policy, "' == 'baseline'"])
    )
    path_aligned_mode = IfCondition(
        PythonExpression(["'", heading_policy, "' == 'path_aligned'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="baseline",
            choices=["baseline", "path_aligned"],
        ),
        LogInfo(msg=[
            "[phase1_5_mppi] Simulation-only Nav2 MPPI comparison. ",
            "heading_policy=", heading_policy, ". ",
            "DWB remains the Phase 1 default controller.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=baseline_mode,
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "nav2_params": mppi_params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=path_aligned_mode,
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "nav2_params": path_aligned_params,
            }.items(),
        ),
    ])
