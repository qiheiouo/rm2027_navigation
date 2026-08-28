"""Old-car full stack with optional dog-hole and automatic-ramp profiles.

The wrapper includes the full hardware/navigation stack exactly once. Ramp
perception starts in shadow unless explicitly activated. The legacy old-car
dog-hole MPPI/costmap profile is also opt-in because its local height ceiling
is a whole-run setting, not a semantic runtime switch.
"""

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
from nav2_common.launch import RewrittenYaml


TRUE_VALUES = {"1", "true", "yes", "on"}


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _launch_full_terrain(context, *args, **kwargs):
    del args, kwargs
    dog_hole_enabled = _enabled(context, "enable_dog_hole_profile")
    ramp_active = _enabled(context, "activate_ramp_filter")

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
            f"{'ENABLED' if dog_hole_enabled else 'disabled'}."
        )),
        LogInfo(
            msg=(
                "[old_car_full_terrain_navigation] WARNING: dog-hole height "
                "ceiling is global for this run. Use it only on a cleared "
                "dog-hole test route; AMCL scan height remains unchanged."
            ),
        ) if dog_hole_enabled else LogInfo(
            msg=(
                "[old_car_full_terrain_navigation] Dog-hole profile is opt-in; "
                "normal obstacle height limits remain active."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ramp_launch),
            launch_arguments={
                "activate_filter": LaunchConfiguration("activate_ramp_filter"),
                "detection_mode": "automatic",
                "map_bundle_override": map_override,
                "nav2_base_config_yaml": selected_nav2,
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
