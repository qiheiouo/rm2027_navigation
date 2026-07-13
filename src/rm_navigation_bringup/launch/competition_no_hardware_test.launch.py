from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enabled = LaunchConfiguration("enable_test")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    simulation_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"), "launch", "simulation.launch.py"
    ])
    referee_launch = PathJoinSubstitution([
        FindPackageShare("rm_referee_interface"), "launch", "referee_interface.launch.py"
    ])
    pursuit_launch = PathJoinSubstitution([
        FindPackageShare("rm_pursuit"), "launch", "pursuit.launch.py"
    ])
    mission_launch = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"), "launch", "competition_mission.launch.py"
    ])
    mission_config = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "config",
        "mission_no_hardware_example.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_test", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        LogInfo(msg=(
            "[competition_no_hardware_test] Disabled by default. The enabled profile "
            "uses Gazebo plus mock competition inputs and opens no serial device."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "scenario": "course_static",
                "headless": headless,
                "use_rviz": use_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(referee_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "enable_referee_interface": "true",
                "use_mock": "true",
                "use_sim_time": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pursuit_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "enable_pursuit_boundary": "true",
                "use_mock_target": "true",
                "use_sim_time": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "enable_mission_node": "true",
                "startup_enabled": "false",
                "use_safety_mock": "true",
                "use_sim_time": "true",
                "mission_config": mission_config,
            }.items(),
        ),
        Node(
            condition=IfCondition(enabled),
            package="rm_system_monitor",
            executable="readiness_monitor",
            name="competition_test_readiness_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "profile": "competition_no_hardware_test",
                "odom_topic": "/odometry/lio",
                "obstacle_topic": "/scan",
                "obstacle_type": "laserscan",
                "require_lio": True,
                "require_obstacle_input": True,
                "require_localization": True,
                "require_nav2": True,
                "require_referee": True,
                "require_chassis_mode": True,
                "require_serial_transport": False,
            }],
        ),
    ])
