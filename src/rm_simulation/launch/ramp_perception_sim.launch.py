from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    gimbal_motion_mode = LaunchConfiguration("gimbal_motion_mode")
    gimbal_angular_velocity = LaunchConfiguration("gimbal_angular_velocity")
    active_map_revision = LaunchConfiguration("active_map_revision")
    include_obstacle = LaunchConfiguration("include_obstacle")

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_gazebo.launch.py",
    ])
    publisher_config = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "config",
        "ramp_pointcloud_sim.yaml",
    ])
    filter_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "ramp_plane_filter_sim.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "config",
        "ramp_perception.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "gimbal_motion_mode",
            default_value="continuous",
            choices=["fixed", "sine", "continuous"],
        ),
        DeclareLaunchArgument(
            "gimbal_angular_velocity", default_value="0.60"
        ),
        DeclareLaunchArgument(
            "active_map_revision", default_value="candidate_11_15_v1"
        ),
        DeclareLaunchArgument("include_obstacle", default_value="true"),
        LogInfo(msg=[
            "[ramp_perception_sim] Shadow-only 11/15 degree ramp filtering; ",
            "raw=/simulation/ramp/points_raw, filtered=",
            "/simulation/ramp/points_filtered_shadow, map_revision=",
            active_map_revision,
            ". No costmap input is remapped.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "headless": headless,
                "use_nav2": "false",
                "use_rviz": "false",
                "use_scan_adapter": "false",
                # Keep this perception scenario independent from the
                # chassis-heading fusion experiment.  The standard adapter
                # still publishes the timestamped rotating-gimbal TF chain.
                "use_chassis_heading_fusion": "false",
                "gimbal_motion_mode": gimbal_motion_mode,
                "gimbal_angular_velocity": gimbal_angular_velocity,
            }.items(),
        ),
        Node(
            package="rm_simulation",
            executable="synthetic_ramp_pointcloud_publisher",
            name="synthetic_ramp_pointcloud_publisher",
            output="screen",
            parameters=[
                publisher_config,
                {
                    "obstacle.enabled": ParameterValue(
                        include_obstacle, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="rm_mid360_driver_bridge",
            executable="ramp_plane_filter_node",
            name="ramp_plane_filter_node",
            output="screen",
            parameters=[
                filter_config,
                {"active_map_revision": active_map_revision},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="ramp_perception_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
