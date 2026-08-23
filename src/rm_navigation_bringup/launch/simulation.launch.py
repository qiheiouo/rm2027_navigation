from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_start = LaunchConfiguration("auto_start")
    robot_geometry_profile = LaunchConfiguration("robot_geometry_profile")
    gimbal_yaw = LaunchConfiguration("gimbal_yaw")
    gimbal_motion_mode = LaunchConfiguration("gimbal_motion_mode")
    gimbal_amplitude = LaunchConfiguration("gimbal_amplitude")
    gimbal_frequency = LaunchConfiguration("gimbal_frequency")
    gimbal_angular_velocity = LaunchConfiguration("gimbal_angular_velocity")
    heading_policy = LaunchConfiguration("heading_policy")
    ramp_active_map_revision = LaunchConfiguration(
        "ramp_active_map_revision"
    )
    ramp_include_obstacle = LaunchConfiguration("ramp_include_obstacle")

    basic_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_mppi.launch.py",
    ])
    course_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase1_5_mppi_course.launch.py",
    ])
    pointcloud_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "phase2g_pointcloud_obstacle.launch.py",
    ])
    dog_hole_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "dog_hole_sim.launch.py",
    ])
    ramp_perception_launch = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "launch",
        "ramp_perception_sim.launch.py",
    ])

    basic_mode = IfCondition(PythonExpression(["'", scenario, "' == 'basic'"]))
    static_course_mode = IfCondition(
        PythonExpression(["'", scenario, "' == 'course_static'"])
    )
    dynamic_course_mode = IfCondition(
        PythonExpression(["'", scenario, "' == 'course_dynamic'"])
    )
    pointcloud_mode = IfCondition(
        PythonExpression(["'", scenario, "' == 'pointcloud'"])
    )
    dog_hole_mode = IfCondition(
        PythonExpression(["'", scenario, "' == 'dog_hole'"])
    )
    ramp_perception_mode = IfCondition(
        PythonExpression(["'", scenario, "' == 'ramp_perception'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="basic",
            choices=[
                "basic",
                "course_static",
                "course_dynamic",
                "pointcloud",
                "dog_hole",
                "ramp_perception",
            ],
        ),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument(
            "robot_geometry_profile",
            default_value="deformed",
            choices=["deformed", "undeformed"],
        ),
        DeclareLaunchArgument("gimbal_yaw", default_value="0.65"),
        DeclareLaunchArgument(
            "gimbal_motion_mode",
            default_value="continuous",
            choices=["fixed", "sine", "continuous"],
        ),
        DeclareLaunchArgument("gimbal_amplitude", default_value="0.8"),
        DeclareLaunchArgument("gimbal_frequency", default_value="0.10"),
        DeclareLaunchArgument(
            "gimbal_angular_velocity", default_value="0.60"
        ),
        DeclareLaunchArgument(
            "heading_policy",
            default_value="baseline",
            choices=["baseline", "path_aligned"],
        ),
        DeclareLaunchArgument(
            "ramp_active_map_revision",
            default_value="candidate_11_15_v1",
        ),
        DeclareLaunchArgument("ramp_include_obstacle", default_value="true"),
        LogInfo(msg=[
            "[simulation] scenario=",
            scenario,
            ". This profile never starts real MID360, serial, or referee IO.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basic_launch),
            condition=basic_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "heading_policy": heading_policy,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(course_launch),
            condition=static_course_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "moving_obstacle": "false",
                "heading_policy": heading_policy,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(course_launch),
            condition=dynamic_course_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "moving_obstacle": "true",
                "heading_policy": heading_policy,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointcloud_launch),
            condition=pointcloud_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dog_hole_launch),
            condition=dog_hole_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "auto_start": auto_start,
                "robot_geometry_profile": robot_geometry_profile,
                "gimbal_yaw": gimbal_yaw,
                "gimbal_motion_mode": gimbal_motion_mode,
                "gimbal_amplitude": gimbal_amplitude,
                "gimbal_frequency": gimbal_frequency,
                "gimbal_angular_velocity": gimbal_angular_velocity,
                "heading_policy": heading_policy,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ramp_perception_launch),
            condition=ramp_perception_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "gimbal_motion_mode": gimbal_motion_mode,
                "gimbal_angular_velocity": gimbal_angular_velocity,
                "active_map_revision": ramp_active_map_revision,
                "include_obstacle": ramp_include_obstacle,
            }.items(),
        ),
    ])
