from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    nav2_params = LaunchConfiguration("nav2_params")
    rviz_config = LaunchConfiguration("rviz_config")

    world = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "worlds",
        "phase1_omni.sdf",
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "config",
        "ros_gz_bridge.yaml",
    ])
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
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1_5_gazebo.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        LogInfo(msg=[
            "[phase1_5_gazebo] Gazebo scan + ground truth -> Nav2 -> ",
            "/cmd_vel -> chassis_interface_stub -> Gazebo. No Gazebo TF is bridged.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(headless),
            launch_arguments={
                "gz_args": ["-r -s --headless-rendering ", world],
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=UnlessCondition(headless),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="simulation_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ),
        Node(
            package="rm_simulation",
            executable="scan_frame_adapter",
            name="scan_frame_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/simulation/scan_raw",
                "output_topic": "/scan",
                "output_frame": "sim_lidar_link",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim_time": "true",
                "use_sim_lidar": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": "true",
                "raw_odom_topic": "/simulation/ground_truth/odom",
            }.items(),
        ),
        Node(
            package="rm_chassis_interface",
            executable="chassis_interface_stub",
            name="chassis_interface_stub",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "cmd_vel_topic": "/cmd_vel",
                "mock_output_cmd_vel_topic": "/simulation/chassis/cmd_vel",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": nav2_params,
                "autostart": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
