"""Old-car full stack with dog-hole entry pause and automatic-ramp profiles.

The wrapper includes the full hardware/navigation stack exactly once. Ramp
perception starts in shadow unless explicitly activated. The legacy old-car
dog-hole MPPI/costmap profile is also opt-in because its local height ceiling
and global scan-marking suppression are whole-run settings, not semantic
runtime switches.
"""

import hashlib
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


TRUE_VALUES = {"1", "true", "yes", "on"}


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _map_binding(bundle_path_text):
    bundle_path = Path(bundle_path_text)
    if not bundle_path.is_file():
        raise RuntimeError(
            "dog-hole entry pause requires an existing map bundle, got: "
            f"{bundle_path_text!r}"
        )
    try:
        payload = bundle_path.read_bytes()
    except OSError as error:
        raise RuntimeError(f"cannot read map bundle: {error}") from error
    if len(payload) > 1024 * 1024:
        raise RuntimeError("map bundle manifest exceeds the 1 MiB safety bound")
    try:
        document = yaml.safe_load(payload)
    except yaml.YAMLError as error:
        raise RuntimeError(f"cannot parse map bundle YAML: {error}") from error
    if not isinstance(document, dict):
        raise RuntimeError("map bundle root must be a mapping")
    map_id = document.get("map_id")
    revision = document.get("revision")
    frame_id = document.get("frame_id")
    if not isinstance(map_id, str) or not map_id.strip():
        raise RuntimeError("map bundle map_id must be a non-empty string")
    if not isinstance(revision, str) or not revision.strip():
        raise RuntimeError("map bundle revision must be a non-empty string")
    if frame_id != "map":
        raise RuntimeError("dog-hole entry pause requires map bundle frame_id=map")
    return map_id.strip(), revision.strip(), hashlib.sha256(payload).hexdigest()


def _launch_full_terrain(context, *args, **kwargs):
    del args, kwargs
    pause_enabled = _enabled(context, "enable_dog_hole_entry_pause")
    dog_hole_enabled = _enabled(context, "enable_dog_hole_profile") or pause_enabled
    ramp_active = _enabled(context, "activate_ramp_filter")
    map_override_text = (
        LaunchConfiguration("map_bundle_override").perform(context).strip()
    )

    gate_nodes = []
    serial_cmd_vel_topic = "/cmd_vel"
    if pause_enabled:
        if not map_override_text:
            raise RuntimeError(
                "enable_dog_hole_entry_pause:=true requires an explicit "
                "map_bundle_override so semantics cannot bind to a hidden default"
            )
        regions_file = (
            LaunchConfiguration("dog_hole_regions_file").perform(context).strip()
        )
        if not regions_file or not Path(regions_file).is_file():
            raise RuntimeError(
                "enable_dog_hole_entry_pause:=true requires a regular "
                f"dog_hole_regions_file, got: {regions_file!r}"
            )
        map_id, map_revision, manifest_sha256 = _map_binding(map_override_text)
        serial_cmd_vel_topic = "/cmd_vel_dog_hole_gated"
        gate_nodes = [Node(
            package="rm_dog_hole_entry_gate",
            executable="dog_hole_entry_pause_gate",
            name="old_car_dog_hole_entry_pause_gate",
            output="screen",
            parameters=[{
                "regions_file": regions_file,
                "expected_map_id": map_id,
                "expected_map_revision": map_revision,
                "expected_manifest_sha256": manifest_sha256,
                "hold_sec": ParameterValue(
                    LaunchConfiguration("dog_hole_hold_sec"), value_type=float
                ),
                "brake_settle_sec": ParameterValue(
                    LaunchConfiguration("dog_hole_brake_settle_sec"),
                    value_type=float,
                ),
                "pose_timeout_sec": ParameterValue(
                    LaunchConfiguration("dog_hole_pose_timeout_sec"),
                    value_type=float,
                ),
                "input_cmd_vel_topic": "/cmd_vel",
                "output_cmd_vel_topic": serial_cmd_vel_topic,
            }],
        )]

    base_nav2 = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_dual_stvl.yaml",
    ])
    selected_nav2 = base_nav2
    if dog_hole_enabled:
        dog_vx = LaunchConfiguration("dog_hole_max_forward_speed")
        dog_wz = LaunchConfiguration("dog_hole_max_yaw_rate")
        obstacle_height = LaunchConfiguration("dog_hole_obstacle_ceiling_height")
        local_inflation = LaunchConfiguration("dog_hole_local_inflation_radius")
        global_inflation = LaunchConfiguration("dog_hole_global_inflation_radius")
        selected_nav2 = RewrittenYaml(
            source_file=base_nav2,
            param_rewrites={
                "controller_server.ros__parameters.FollowPath.motion_model": (
                    "DiffDrive"
                ),
                "controller_server.ros__parameters.FollowPath.vx_max": dog_vx,
                "controller_server.ros__parameters.FollowPath.vx_min": "-0.30",
                "controller_server.ros__parameters.FollowPath.wz_max": dog_wz,
                (
                    "controller_server.ros__parameters.FollowPath."
                    "PathAngleCritic.cost_weight"
                ): "6.0",
                (
                    "controller_server.ros__parameters.FollowPath."
                    "PathAngleCritic.max_angle_to_furthest"
                ): "0.20",
                (
                    "controller_server.ros__parameters.FollowPath."
                    "PathAngleCritic.forward_preference"
                ): "true",
                "behavior_server.ros__parameters.max_rotational_vel": dog_wz,
                "behavior_server.ros__parameters.min_rotational_vel": "0.10",
                "behavior_server.ros__parameters.rotational_acc_lim": "1.50",
                (
                    "local_costmap.local_costmap.ros__parameters.stvl_layer."
                    "max_obstacle_height"
                ): obstacle_height,
                (
                    "local_costmap.local_costmap.ros__parameters.stvl_layer."
                    "fused_mid360_mark.max_obstacle_height"
                ): obstacle_height,
                (
                    "local_costmap.local_costmap.ros__parameters."
                    "inflation_layer.inflation_radius"
                ): local_inflation,
                (
                    "global_costmap.global_costmap.ros__parameters."
                    "obstacle_layer.localization_scan.marking"
                ): "false",
                (
                    "global_costmap.global_costmap.ros__parameters."
                    "inflation_layer.inflation_radius"
                ): global_inflation,
            },
            convert_types=True,
        )

    ramp_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_launch"),
        "launch",
        "old_car_ramp_validation.launch.py",
    ])
    map_override = LaunchConfiguration("map_bundle_override")
    return [
        LogInfo(msg=(
            "[old_car_full_terrain_navigation] full stack + automatic ramp "
            f"{'ACTIVE' if ramp_active else 'SHADOW'}; legacy dog-hole profile "
            f"{'ENABLED' if dog_hole_enabled else 'disabled'}; entry pause "
            f"{'ACTIVE' if pause_enabled else 'disabled'}."
        )),
        LogInfo(
            msg=(
                "[old_car_full_terrain_navigation] WARNING: dog-hole height "
                "ceiling and global scan-marking suppression are active for "
                "this whole run. The static global map and local STVL remain "
                "active. Use only on a cleared dog-hole test route; AMCL scan "
                "height remains unchanged."
            ),
        ) if dog_hole_enabled else LogInfo(
            msg=(
                "[old_car_full_terrain_navigation] Dog-hole profile is opt-in; "
                "normal obstacle height limits remain active."
            ),
        ),
        *gate_nodes,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ramp_launch),
            launch_arguments={
                "activate_filter": LaunchConfiguration("activate_ramp_filter"),
                "detection_mode": "automatic",
                "map_bundle_override": map_override,
                "nav2_base_config_yaml": selected_nav2,
                "serial_cmd_vel_topic": serial_cmd_vel_topic,
                "max_forward_speed": LaunchConfiguration(
                    "ramp_max_forward_speed"
                ),
                "max_yaw_rate": LaunchConfiguration("ramp_max_yaw_rate"),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_bundle_override",
            default_value="",
            description="Empty inherits old_car_full_navigation configured map.",
        ),
        DeclareLaunchArgument("activate_ramp_filter", default_value="false"),
        DeclareLaunchArgument("ramp_max_forward_speed", default_value="0.20"),
        DeclareLaunchArgument("ramp_max_yaw_rate", default_value="0.40"),
        DeclareLaunchArgument("enable_dog_hole_profile", default_value="false"),
        DeclareLaunchArgument(
            "enable_dog_hole_entry_pause",
            default_value="false",
            description=(
                "Route final Nav2 velocity through the map-bound 5 s entry gate. "
                "Enabling this also enables the legacy dog-hole Nav2 profile."
            ),
        ),
        DeclareLaunchArgument("dog_hole_regions_file", default_value=""),
        DeclareLaunchArgument("dog_hole_hold_sec", default_value="5.0"),
        DeclareLaunchArgument("dog_hole_brake_settle_sec", default_value="0.5"),
        DeclareLaunchArgument("dog_hole_pose_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument(
            "dog_hole_max_forward_speed", default_value="0.80"
        ),
        DeclareLaunchArgument("dog_hole_max_yaw_rate", default_value="0.80"),
        DeclareLaunchArgument(
            "dog_hole_obstacle_ceiling_height", default_value="0.55"
        ),
        DeclareLaunchArgument(
            "dog_hole_local_inflation_radius", default_value="0.10"
        ),
        DeclareLaunchArgument(
            "dog_hole_global_inflation_radius", default_value="0.10"
        ),
        OpaqueFunction(function=_launch_full_terrain),
    ])
