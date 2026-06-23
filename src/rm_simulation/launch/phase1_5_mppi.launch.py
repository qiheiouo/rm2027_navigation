from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

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

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        LogInfo(msg=[
            "[phase1_5_mppi] Simulation-only Nav2 MPPI comparison. ",
            "DWB remains the Phase 1 default controller.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "nav2_params": mppi_params,
            }.items(),
        ),
    ])
