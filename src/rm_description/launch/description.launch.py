from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_lidar = LaunchConfiguration("use_sim_lidar")
    gimbal_joint_height = LaunchConfiguration("gimbal_joint_height")
    sim_lidar_x = LaunchConfiguration("sim_lidar_x")
    sim_lidar_y = LaunchConfiguration("sim_lidar_y")
    sim_lidar_z = LaunchConfiguration("sim_lidar_z")
    dynamic_tf_publish_frequency = LaunchConfiguration("dynamic_tf_publish_frequency")
    model = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "urdf",
        "rm_sentry_2027.urdf.xacro",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_sim_lidar", default_value="false"),
        DeclareLaunchArgument("gimbal_joint_height", default_value="0.115"),
        DeclareLaunchArgument("sim_lidar_x", default_value="0.12"),
        DeclareLaunchArgument("sim_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("sim_lidar_z", default_value="0.065"),
        DeclareLaunchArgument("dynamic_tf_publish_frequency", default_value="100.0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                # robot_state_publisher defaults to 20 Hz and would throttle
                # the 50 Hz gimbal joint stream used by timestamped LIO.
                "publish_frequency": ParameterValue(
                    dynamic_tf_publish_frequency,
                    value_type=float,
                ),
                "ignore_timestamp": False,
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        model,
                        " use_sim_lidar:=",
                        use_sim_lidar,
                        " gimbal_joint_height:=",
                        gimbal_joint_height,
                        " sim_lidar_x:=",
                        sim_lidar_x,
                        " sim_lidar_y:=",
                        sim_lidar_y,
                        " sim_lidar_z:=",
                        sim_lidar_z,
                    ]),
                    value_type=str,
                ),
            }],
        ),
    ])
