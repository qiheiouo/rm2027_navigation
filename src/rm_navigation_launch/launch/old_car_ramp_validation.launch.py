"""Isolated old-car integration for the existing map-bound ramp filter.

The default is shadow-only: normal full navigation keeps its original point
cloud topics while two filter instances publish comparison clouds. Explicit
active mode routes the filtered left cloud to AMCL/global obstacle projection
and the filtered fused cloud to local STVL. Contract mismatch and TF failure
remain passthrough, so the ramp continues to appear as an obstacle.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


TRUE_VALUES = {"1", "true", "yes", "on"}


def _launch_validation(context, *args, **kwargs):
    del args, kwargs
    active = (
        LaunchConfiguration("activate_filter").perform(context).strip().lower()
        in TRUE_VALUES
    )
    regions_file = LaunchConfiguration("regions_file").perform(context).strip()
    if not regions_file or not Path(regions_file).is_file():
        raise RuntimeError(
            f"regions_file must be a regular YAML file, got: {regions_file!r}"
        )

    active_map_id = LaunchConfiguration("active_map_id")
    active_map_revision = LaunchConfiguration("active_map_revision")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")

    full_navigation_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_launch"),
        "launch",
        "old_car_full_navigation.launch.py",
    ])
    base_nav2 = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_dual_stvl.yaml",
    ])
    ramp_nav2 = RewrittenYaml(
        source_file=base_nav2,
        param_rewrites={
            "controller_server.ros__parameters.FollowPath.motion_model": "DiffDrive",
            "controller_server.ros__parameters.FollowPath.vx_max": max_forward_speed,
            "controller_server.ros__parameters.FollowPath.wz_max": max_yaw_rate,
            "controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight": "6.0",
            "controller_server.ros__parameters.FollowPath.PathAngleCritic.max_angle_to_furthest": "0.35",
            "controller_server.ros__parameters.FollowPath.PathAngleCritic.forward_preference": "true",
            "behavior_server.ros__parameters.max_rotational_vel": max_yaw_rate,
            "local_costmap.local_costmap.ros__parameters.stvl_layer.fused_mid360_mark.topic": "/points/obstacles_ramp_filtered",
        },
        convert_types=True,
    )

    common_parameters = [
        regions_file,
        {
            "active_map_id": active_map_id,
            "active_map_revision": active_map_revision,
            "shadow_only": not active,
        },
    ]
    localization_filter = Node(
        package="rm_mid360_driver_bridge",
        executable="ramp_plane_filter_node",
        name="old_car_localization_ramp_filter",
        output="screen",
        parameters=common_parameters + [{
            "input_topic": "/livox/left/pointcloud_filtered",
            "output_topic": (
                "/livox/left/pointcloud_ramp_filtered"
                if active
                else "/perception/ramp/localization_filtered_shadow"
            ),
        }],
    )
    obstacle_filter = Node(
        package="rm_mid360_driver_bridge",
        executable="ramp_plane_filter_node",
        name="old_car_obstacle_ramp_filter",
        output="screen",
        parameters=common_parameters + [{
            "input_topic": "/points/obstacles_fused",
            "output_topic": (
                "/points/obstacles_ramp_filtered"
                if active
                else "/perception/ramp/obstacles_filtered_shadow"
            ),
        }],
    )

    include_arguments = {}
    map_bundle_yaml = LaunchConfiguration("map_bundle_yaml").perform(context).strip()
    if map_bundle_yaml:
        include_arguments["map_bundle_yaml"] = map_bundle_yaml
    if active:
        include_arguments.update({
            "nav2_config_yaml": ramp_nav2,
            "localization_pointcloud_topic": "/livox/left/pointcloud_ramp_filtered",
        })

    mode_text = "ACTIVE A/B" if active else "SHADOW"
    return [
        LogInfo(msg=(
            f"[old_car_ramp_validation] {mode_text}: reusing the existing "
            "map-bound ramp plane filter. Active mode is permitted only on a "
            "measured, clear test ramp with physical stop authority."
        )),
        localization_filter,
        obstacle_filter,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_navigation_launch),
            launch_arguments=include_arguments.items(),
        ),
    ]


def generate_launch_description():
    default_regions = PathJoinSubstitution([
        FindPackageShare("rm_navigation_launch"),
        "config",
        "old_car_ramp_regions.example.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("activate_filter", default_value="false"),
        DeclareLaunchArgument("regions_file", default_value=default_regions),
        DeclareLaunchArgument("active_map_id", default_value="unset"),
        DeclareLaunchArgument("active_map_revision", default_value="unset"),
        DeclareLaunchArgument(
            "map_bundle_yaml",
            default_value="",
            description="Empty inherits the map configured by old_car_full_navigation.",
        ),
        DeclareLaunchArgument("max_forward_speed", default_value="0.35"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
        OpaqueFunction(function=_launch_validation),
    ])
