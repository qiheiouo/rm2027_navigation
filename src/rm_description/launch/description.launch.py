from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_lidar = LaunchConfiguration("use_sim_lidar")
    model = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "urdf",
        "rm_sentry_2027.urdf.xacro",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_sim_lidar", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        model,
                        " use_sim_lidar:=",
                        use_sim_lidar,
                    ]),
                    value_type=str,
                ),
            }],
        ),
    ])
