from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_gazebo.launch.py",
    ])
    pointcloud_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase2g_pointcloud.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        LogInfo(msg=[
            "[phase2g_pointcloud_obstacle] Simulation-only PointCloud2 obstacle ",
            "boundary. /scan is disabled; Nav2 costmaps must consume /points/obstacles.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "use_scan_adapter": "false",
                "nav2_params": pointcloud_params,
            }.items(),
        ),
        Node(
            package="rm_simulation",
            executable="fake_pointcloud_obstacle_publisher",
            name="fake_pointcloud_obstacle_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "output_topic": "/points/obstacles",
                "frame_id": "sim_lidar_link",
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "obstacle_x": 1.4,
                "obstacle_y": 0.0,
                "obstacle_z": 0.0,
                "width_y": 0.55,
                "height_z": 0.70,
                "y_samples": 13,
                "z_samples": 8,
            }],
        ),
    ])
