from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_fake_lio = LaunchConfiguration("use_fake_lio")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_chassis_stub = LaunchConfiguration("use_chassis_stub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")
    rviz_config = LaunchConfiguration("rviz_config")

    description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "description.launch.py",
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "launch",
        "localization_adapters.launch.py",
    ])
    chassis_launch = PathJoinSubstitution([
        FindPackageShare("rm_chassis_interface"),
        "launch",
        "chassis_interface_stub.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_lio", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_chassis_stub", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        LogInfo(msg=[
            "[phase1_bringup] No-hardware loop: fake LIO -> lio_adapter -> Nav2 -> ",
            "/cmd_vel -> chassis_interface_stub. FAST-LIO, real serial, referee, and BT are not launched."
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
        ),
        Node(
            condition=IfCondition(use_fake_lio),
            package="rm_localization_adapters",
            executable="fake_lio_odom_publisher",
            name="fake_lio_odom_publisher",
            output="screen",
            parameters=[{
                "motion_mode": "cmd_vel",
                "cmd_vel_topic": "/cmd_vel",
                "publish_rate_hz": 30.0,
                "cmd_vel_timeout_sec": 0.5,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(chassis_launch),
            condition=IfCondition(use_chassis_stub),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
                "autostart": "true",
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
