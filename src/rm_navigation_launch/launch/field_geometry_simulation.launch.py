"""Launch the lightweight RMUC 2026 navigation field simulation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "dog_hole_sim.launch.py",
    ])
    dog_hole_config = PathJoinSubstitution([
        FindPackageShare("rm_dog_hole"),
        "config",
        "dog_hole_sim.yaml",
    ])
    official_field = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "models",
        "rmuc2026_navigation_field.sdf",
    ])
    surface_filter_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "rmuc2026_navigation_surface_filter_sim.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description=(
                "Keep the robot stopped for geometry inspection by default; "
                "set true to run the dog-hole sequence."
            ),
        ),
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
        DeclareLaunchArgument("gimbal_angular_velocity", default_value="0.60"),
        DeclareLaunchArgument("active_ramp_filter", default_value="true"),
        DeclareLaunchArgument(
            "global_costmap_update_frequency", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "global_costmap_resolution", default_value="0.10"
        ),
        DeclareLaunchArgument("mppi_batch_size", default_value="400"),
        DeclareLaunchArgument(
            "physics_max_step_size", default_value="0.002"
        ),
        DeclareLaunchArgument("lidar_update_rate", default_value="10.0"),
        DeclareLaunchArgument("lidar_samples", default_value="360"),
        DeclareLaunchArgument("bt_loop_duration", default_value="50"),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="path_aligned",
            choices=["baseline", "path_aligned"],
        ),
        DeclareLaunchArgument(
            "dog_hole_config", default_value=dog_hole_config
        ),
        LogInfo(msg=[
            "[field_geometry_simulation] Lightweight RMUC 2026 navigation ",
            "field: roofed dog holes + physical 10.5/11/15 degree ramps. ",
            "Real hardware and the full STEP model remain disabled.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
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
                "heading_policy": LaunchConfiguration("heading_policy"),
                "active_ramp_filter": LaunchConfiguration(
                    "active_ramp_filter"
                ),
                "dog_hole_config": LaunchConfiguration("dog_hole_config"),
                "spawn_dog_hole_scene": "false",
                "spawn_ramp_scene": "false",
                "ramp_filter_config": surface_filter_config,
                # Gazebo tilts the physical lidar with the chassis while the
                # heading-fusion base frame is intentionally planar. This
                # simulation-only frame keeps scan projection physically
                # consistent without changing the real localization contract.
                "scan_output_frame": "sim_lidar_physics_frame",
                "publish_sensor_truth_tf": "true",
                "robot_spawn_x": "-11.0",
                "robot_spawn_y": "-3.0",
                "robot_spawn_yaw": "0.0",
                # A fixed 30 x 17 m, 0.10 m/cell grid covers the complete
                # field while keeping roughly the same cell count as the
                # former 12 x 12 m, 0.05 m/cell rolling window.
                "global_costmap_width": "30.0",
                "global_costmap_height": "17.0",
                "global_costmap_update_frequency": LaunchConfiguration(
                    "global_costmap_update_frequency"
                ),
                "global_costmap_resolution": LaunchConfiguration(
                    "global_costmap_resolution"
                ),
                "global_costmap_rolling_window": "false",
                "global_costmap_origin_x": "-15.0",
                "global_costmap_origin_y": "-8.5",
                "mppi_batch_size": LaunchConfiguration("mppi_batch_size"),
                "physics_max_step_size": LaunchConfiguration(
                    "physics_max_step_size"
                ),
                "lidar_update_rate": LaunchConfiguration(
                    "lidar_update_rate"
                ),
                "lidar_samples": LaunchConfiguration("lidar_samples"),
                "bt_loop_duration": LaunchConfiguration(
                    "bt_loop_duration"
                ),
                # Red road tunnel: traverse south-to-north under the road.
                "dog_hole_center_x": "-3.0",
                "dog_hole_center_y": "-5.5",
                "dog_hole_yaw": "1.5707963267948966",
                "dog_hole_width": "0.80",
                "dog_hole_length": "1.80",
                "dog_hole_final_goal_x": "-3.0",
                "dog_hole_final_goal_y": "-3.8",
                "dog_hole_final_goal_yaw": "1.5707963267948966",
            }.items(),
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_rmuc2026_navigation_field",
                    output="screen",
                    arguments=[
                        "-world",
                        "phase1_omni",
                        "-file",
                        official_field,
                        "-name",
                        "rmuc2026_navigation_field",
                    ],
                )
            ],
        ),
    ])
