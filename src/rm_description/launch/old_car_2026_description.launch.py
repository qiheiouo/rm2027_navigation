from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    dynamic_tf_publish_frequency = LaunchConfiguration("dynamic_tf_publish_frequency")
    model = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "urdf",
        "rm_old_car_2026.urdf.xacro",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("dynamic_tf_publish_frequency", default_value="100.0"),
        LogInfo(msg=(
            "[old_car_2026_description] Old-car-only extrinsics are loaded for "
            "validation. They are not 2027 robot calibration data."
        )),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "publish_frequency": ParameterValue(
                    dynamic_tf_publish_frequency,
                    value_type=float,
                ),
                "ignore_timestamp": False,
                "robot_description": ParameterValue(
                    Command(["xacro ", model]),
                    value_type=str,
                ),
            }],
        ),
    ])
