"""统一的新车场地几何仿真入口。

默认生成有碰撞顶板的狗洞以及 11/15 度实体斜坡，启动 Gazebo GUI 和 RViz，
但不会启动真实雷达、串口或旧车硬件。底层节点继续由 rm_simulation 所有。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "dog_hole_sim.launch.py",
    ])
    dog_hole_config = PathJoinSubstitution([
        FindPackageShare("rm_dog_hole"),
        "config",
        "dog_hole_sim.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description=(
                "Keep the robot stopped for geometry inspection by default; "
                "set true to run the dog-hole sequence."
            ),
        ),
        DeclareLaunchArgument(
            "robot_geometry_profile",
            default_value="deformed",
            choices=["deformed", "undeformed"],
        ),
        DeclareLaunchArgument(
            "gimbal_motion_mode",
            default_value="continuous",
            choices=["fixed", "sine", "continuous"],
        ),
        DeclareLaunchArgument("gimbal_angular_velocity", default_value="0.60"),
        DeclareLaunchArgument("active_ramp_filter", default_value="true"),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="path_aligned",
            choices=["baseline", "path_aligned"],
        ),
        DeclareLaunchArgument(
            "dog_hole_config", default_value=dog_hole_config
        ),
        LogInfo(msg=[
            "[field_geometry_simulation] New-car simulation: roofed dog hole ",
            "+ physical 11/15 degree ramps. Real hardware remains disabled.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "auto_start": LaunchConfiguration("auto_start"),
                "robot_geometry_profile": LaunchConfiguration(
                    "robot_geometry_profile"
                ),
                "gimbal_motion_mode": LaunchConfiguration(
                    "gimbal_motion_mode"
                ),
                "gimbal_angular_velocity": LaunchConfiguration(
                    "gimbal_angular_velocity"
                ),
                "heading_policy": LaunchConfiguration("heading_policy"),
                "active_ramp_filter": LaunchConfiguration(
                    "active_ramp_filter"
                ),
                "dog_hole_config": LaunchConfiguration("dog_hole_config"),
                "spawn_ramp_scene": "true",
            }.items(),
        ),
    ])
