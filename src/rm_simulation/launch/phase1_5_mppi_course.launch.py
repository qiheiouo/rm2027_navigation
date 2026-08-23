from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    heading_policy = LaunchConfiguration("heading_policy")
    moving_obstacle = LaunchConfiguration("moving_obstacle")
    moving_amplitude = LaunchConfiguration("moving_amplitude")
    moving_period = LaunchConfiguration("moving_period")

    mppi_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_mppi.launch.py",
    ])
    wall_model = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "models",
        "course_wall.sdf",
    ])
    moving_model = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "models",
        "moving_obstacle.sdf",
    ])
    dynamic_bridge_config = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "config",
        "course_dynamic_bridge.yaml",
    ])

    north_wall = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_course_wall_north",
        output="screen",
        arguments=[
            "-world", "phase1_omni",
            "-file", wall_model,
            "-name", "course_wall_north",
            "-x", "3.0", "-y", "0.525", "-z", "0.0",
        ],
    )
    south_wall = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_course_wall_south",
        output="screen",
        arguments=[
            "-world", "phase1_omni",
            "-file", wall_model,
            "-name", "course_wall_south",
            "-x", "3.0", "-y", "-0.525", "-z", "0.0",
        ],
    )
    moving_block = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_moving_obstacle",
        output="screen",
        condition=IfCondition(moving_obstacle),
        arguments=[
            "-world", "phase1_omni",
            "-file", moving_model,
            "-name", "moving_obstacle",
            "-x", "4.9", "-y", "0.0", "-z", "0.0",
        ],
    )
    moving_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="course_dynamic_bridge",
        output="screen",
        condition=IfCondition(moving_obstacle),
        parameters=[{"config_file": dynamic_bridge_config}],
    )
    moving_controller = Node(
        package="rm_simulation",
        executable="moving_obstacle_controller",
        name="moving_obstacle_controller",
        output="screen",
        condition=IfCondition(moving_obstacle),
        parameters=[{
            "use_sim_time": True,
            "amplitude": ParameterValue(moving_amplitude, value_type=float),
            "period": ParameterValue(moving_period, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="baseline",
            choices=["baseline", "path_aligned"],
        ),
        DeclareLaunchArgument("moving_obstacle", default_value="true"),
        DeclareLaunchArgument("moving_amplitude", default_value="0.9"),
        DeclareLaunchArgument("moving_period", default_value="8.0"),
        LogInfo(msg=[
            "[phase1_5_mppi_course] Multi-obstacle course with a 0.8 m ",
            "static passage and optional moving obstacle. No Gazebo TF is bridged.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mppi_launch),
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "heading_policy": heading_policy,
            }.items(),
        ),
        TimerAction(period=2.0, actions=[north_wall, south_wall]),
        TimerAction(period=3.0, actions=[moving_block, moving_bridge]),
        TimerAction(period=4.0, actions=[moving_controller]),
    ])
