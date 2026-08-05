from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

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
            ],
        ),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
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
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(course_launch),
            condition=static_course_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "moving_obstacle": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(course_launch),
            condition=dynamic_course_mode,
            launch_arguments={
                "headless": headless,
                "use_rviz": use_rviz,
                "moving_obstacle": "true",
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
            }.items(),
        ),
    ])
