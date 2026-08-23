"""Old-car full navigation with only the dog-hole traversal overrides.

All hardware, localization, mission, serial, referee, and safety ownership is
inherited from old_car_full_navigation.launch.py.  This wrapper selects the
dog-hole map and rewrites only the perception height gate and forward-motion
parameters needed for the temporary old-car traversal test.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


DOG_HOLE_MAP_BUNDLE = (
    "/data/rm27_maps/lab_dog_hole/20260820T040437Z/"
    "lab_dog_hole.bundle.yaml"
)


def generate_launch_description():
    map_bundle_yaml = LaunchConfiguration("map_bundle_yaml")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")
    obstacle_ceiling_height = LaunchConfiguration("obstacle_ceiling_height")
    local_inflation_radius = LaunchConfiguration("local_inflation_radius")
    global_inflation_radius = LaunchConfiguration("global_inflation_radius")

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
    base_scan_projection = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
    ])

    dog_hole_nav2 = RewrittenYaml(
        source_file=base_nav2,
        param_rewrites={
            "controller_server.ros__parameters.FollowPath.motion_model": (
                "DiffDrive"
            ),
            "controller_server.ros__parameters.FollowPath.vx_max": (
                max_forward_speed
            ),
            "controller_server.ros__parameters.FollowPath.vx_min": "-0.30",
            "controller_server.ros__parameters.FollowPath.wz_max": max_yaw_rate,
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.cost_weight": "6.0",
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.max_angle_to_furthest": "0.20",
            "controller_server.ros__parameters.FollowPath."
            "PathAngleCritic.forward_preference": "true",
            "behavior_server.ros__parameters.max_rotational_vel": max_yaw_rate,
            "behavior_server.ros__parameters.min_rotational_vel": "0.10",
            "behavior_server.ros__parameters.rotational_acc_lim": "1.50",
            "local_costmap.local_costmap.ros__parameters.stvl_layer."
            "max_obstacle_height": obstacle_ceiling_height,
            "local_costmap.local_costmap.ros__parameters.stvl_layer."
            "fused_mid360_mark.max_obstacle_height": obstacle_ceiling_height,
            "local_costmap.local_costmap.ros__parameters.inflation_layer."
            "inflation_radius": local_inflation_radius,
            "global_costmap.global_costmap.ros__parameters.inflation_layer."
            "inflation_radius": global_inflation_radius,
        },
        convert_types=True,
    )
    dog_hole_scan_projection = RewrittenYaml(
        source_file=base_scan_projection,
        param_rewrites={
            "pointcloud_to_laserscan_node.ros__parameters.max_height": (
                obstacle_ceiling_height
            ),
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_bundle_yaml",
            default_value=DOG_HOLE_MAP_BUNDLE,
            description="Validated candidate map bundle containing the dog-hole opening.",
        ),
        DeclareLaunchArgument(
            "max_forward_speed",
            default_value="0.80",
            description=(
                "Dog-hole MPPI forward speed limit; smoother headroom is 3.0 m/s."
            ),
        ),
        DeclareLaunchArgument(
            "max_yaw_rate",
            default_value="0.80",
            description=(
                "Dog-hole MPPI and recovery yaw limit; smoother headroom is "
                "1.2 rad/s."
            ),
        ),
        DeclareLaunchArgument(
            "obstacle_ceiling_height",
            default_value="0.55",
            description=(
                "Maximum base-frame point height used for localization scan and "
                "local costmap marking during this temporary traversal test."
            ),
        ),
        DeclareLaunchArgument(
            "local_inflation_radius",
            default_value="0.10",
            description=(
                "Temporary dog-hole local inflation radius. The configured "
                "robot footprint remains unchanged."
            ),
        ),
        DeclareLaunchArgument(
            "global_inflation_radius",
            default_value="0.10",
            description=(
                "Temporary dog-hole global inflation radius. The configured "
                "robot footprint remains unchanged."
            ),
        ),
        LogInfo(msg=[
            "[old_car_dog_hole_navigation] Full-navigation baseline with ",
            "dog-hole-only overrides: map=",
            map_bundle_yaml,
            ", vx_max=",
            max_forward_speed,
            " m/s, wz_max=",
            max_yaw_rate,
            " rad/s, point_height_max=",
            obstacle_ceiling_height,
            " m, local/global inflation=",
            local_inflation_radius,
            "/",
            global_inflation_radius,
            " m.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_navigation_launch),
            launch_arguments={
                "map_bundle_yaml": map_bundle_yaml,
                "nav2_config_yaml": dog_hole_nav2,
                "scan_projection_config_yaml": dog_hole_scan_projection,
            }.items(),
        ),
    ])
