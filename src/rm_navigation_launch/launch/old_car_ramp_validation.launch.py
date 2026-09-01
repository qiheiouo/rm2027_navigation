"""Isolated old-car integration for automatic or map-bound ramp filtering.

The default is unlabelled automatic detection in shadow-only mode: normal full
navigation keeps its original point-cloud topics while one fused-cloud tracker
publishes obstacle and localization comparison clouds from the same confirmed
ramp model. A manually measured map contract remains an optional deterministic
fallback. Explicit active mode routes filtered clouds to Nav2.
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
    detection_mode = (
        LaunchConfiguration("detection_mode").perform(context).strip().lower()
    )
    if detection_mode not in {"automatic", "map_regions"}:
        raise RuntimeError(
            "detection_mode must be 'automatic' or 'map_regions', "
            f"got: {detection_mode!r}"
        )
    regions_file = LaunchConfiguration("regions_file").perform(context).strip()
    if detection_mode == "map_regions" and (
        not regions_file or not Path(regions_file).is_file()
    ):
        raise RuntimeError(
            "map_regions mode requires a regular regions_file, "
            f"got: {regions_file!r}"
        )
    automatic_params_file = (
        LaunchConfiguration("automatic_params_file").perform(context).strip()
    )
    if detection_mode == "automatic" and (
        not automatic_params_file or not Path(automatic_params_file).is_file()
    ):
        raise RuntimeError(
            "automatic mode requires a regular automatic_params_file, "
            f"got: {automatic_params_file!r}"
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
    base_nav2 = LaunchConfiguration("nav2_base_config_yaml")
    ramp_nav2 = RewrittenYaml(
        source_file=base_nav2,
        param_rewrites={
            "controller_server.ros__parameters.FollowPath.motion_model": "DiffDrive",
            "controller_server.ros__parameters.FollowPath.vx_max": max_forward_speed,
            "controller_server.ros__parameters.FollowPath.wz_max": max_yaw_rate,
            "controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight": "6.0",
            (
                "controller_server.ros__parameters.FollowPath.PathAngleCritic."
                "max_angle_to_furthest"
            ): "0.35",
            (
                "controller_server.ros__parameters.FollowPath.PathAngleCritic."
                "forward_preference"
            ): "true",
            "behavior_server.ros__parameters.max_rotational_vel": max_yaw_rate,
            (
                "local_costmap.local_costmap.ros__parameters.stvl_layer."
                "fused_mid360_mark.topic"
            ): "/points/obstacles_ramp_filtered",
        },
        convert_types=True,
    )

    if detection_mode == "automatic":
        executable = "automatic_ramp_filter_node"
        common_parameters = [
            automatic_params_file,
            {
                "detection_frame": LaunchConfiguration(
                    "automatic_detection_frame"
                ),
                "shadow_only": not active,
                "tracking.confirmation_frames": ParameterValue(
                    LaunchConfiguration("automatic_confirmation_frames"),
                    value_type=int,
                ),
            },
        ]
        filter_nodes = [Node(
            package="rm_mid360_driver_bridge",
            executable=executable,
            name="old_car_shared_ramp_filter",
            output="screen",
            parameters=common_parameters + [{
                # The denser fused stream is the only detection/tracking source.
                # Its confirmed odom-frame model is reused to filter the left
                # localization cloud, avoiding two trackers that disagree.
                "input_topic": "/points/obstacles_fused",
                "output_topic": (
                    "/points/obstacles_ramp_filtered"
                    if active
                    else "/perception/ramp/obstacles_filtered_shadow"
                ),
                "secondary_input_topic": "/livox/left/pointcloud_filtered",
                "secondary_output_topic": (
                    "/livox/left/pointcloud_ramp_filtered"
                    if active
                    else "/perception/ramp/localization_filtered_shadow"
                ),
            }],
        )]
    else:
        executable = "ramp_plane_filter_node"
        common_parameters = [
            regions_file,
            {
                "active_map_id": active_map_id,
                "active_map_revision": active_map_revision,
                "shadow_only": not active,
            },
        ]
        filter_nodes = [
            Node(
                package="rm_mid360_driver_bridge",
                executable=executable,
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
            ),
            Node(
                package="rm_mid360_driver_bridge",
                executable=executable,
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
            ),
        ]

    include_arguments = {
        "nav2_config_yaml": base_nav2,
        "serial_cmd_vel_topic": LaunchConfiguration("serial_cmd_vel_topic"),
        "serial_max_vx": LaunchConfiguration("serial_max_vx"),
        "navigate_to_pose_action": LaunchConfiguration(
            "navigate_to_pose_action"
        ),
        "goal_pose_topic": LaunchConfiguration("goal_pose_topic"),
    }
    map_bundle_override = (
        LaunchConfiguration("map_bundle_override").perform(context).strip()
    )
    if map_bundle_override:
        include_arguments["map_bundle_yaml"] = map_bundle_override
    if active:
        include_arguments.update({
            "nav2_config_yaml": ramp_nav2,
            "localization_pointcloud_topic": "/livox/left/pointcloud_ramp_filtered",
        })

    mode_text = "ACTIVE A/B" if active else "SHADOW"
    return [
        LogInfo(msg=(
            f"[old_car_ramp_validation] {mode_text}/{detection_mode}: "
            "ramp filtering is fail-closed until geometry is confirmed. "
            "Active mode is permitted only on a clear test ramp with physical "
            "stop authority."
        )),
        *filter_nodes,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_navigation_launch),
            launch_arguments=include_arguments.items(),
        ),
    ]


def generate_launch_description():
    default_automatic_params = PathJoinSubstitution([
        FindPackageShare("rm_navigation_launch"),
        "config",
        "old_car_automatic_ramp_filter.yaml",
    ])
    default_nav2_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_dual_stvl.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("activate_filter", default_value="false"),
        DeclareLaunchArgument("detection_mode", default_value="automatic"),
        DeclareLaunchArgument("regions_file", default_value=""),
        DeclareLaunchArgument("active_map_id", default_value="unset"),
        DeclareLaunchArgument("active_map_revision", default_value="unset"),
        DeclareLaunchArgument(
            "automatic_params_file", default_value=default_automatic_params
        ),
        DeclareLaunchArgument("automatic_detection_frame", default_value="odom"),
        DeclareLaunchArgument("automatic_confirmation_frames", default_value="5"),
        DeclareLaunchArgument(
            "map_bundle_override",
            default_value="",
            description="Empty inherits the map configured by old_car_full_navigation.",
        ),
        DeclareLaunchArgument(
            "nav2_base_config_yaml", default_value=default_nav2_config
        ),
        DeclareLaunchArgument("max_forward_speed", default_value="0.35"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.50"),
        DeclareLaunchArgument("serial_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("serial_max_vx", default_value="3.0"),
        DeclareLaunchArgument(
            "navigate_to_pose_action", default_value="/navigate_to_pose"
        ),
        DeclareLaunchArgument("goal_pose_topic", default_value="/goal_pose"),
        OpaqueFunction(function=_launch_validation),
    ])
