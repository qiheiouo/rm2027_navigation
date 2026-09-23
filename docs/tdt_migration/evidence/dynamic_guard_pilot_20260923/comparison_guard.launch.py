"""Explicit simulation-only reuse of the Phase 1.5 world and static course."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
                            RegisterEventHandler, LogInfo, EmitEvent)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("rm_simulation")
    walls = []
    for name, y in (("north", "0.525"), ("south", "-0.525")):
        walls.append(Node(
            package="ros_gz_sim", executable="create", name=f"spawn_course_wall_{name}",
            arguments=["-world", "phase1_omni", "-file",
                       PathJoinSubstitution([share, "models", "course_wall.sdf"]),
                       "-name", f"course_wall_{name}", "-x", "3.0", "-y", y, "-z", "0.0"],
            output="screen"))
    handlers = []
    for wall, side in zip(walls, ("north", "south")):
        def finished(event, context, side=side):
            actions = [LogInfo(msg=f"[TDT_P2B] spawn_course_wall_{side} exit={event.returncode}")]
            if event.returncode != 0:
                actions.append(EmitEvent(event=Shutdown(reason="P2B static fixture spawn failed")))
            return actions
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=wall, on_exit=finished)))
    return LaunchDescription([
        *handlers,
        DeclareLaunchArgument("params_file", description="New complete simulation comparison YAML"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/ws/docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/phase1_guard.launch.py"),
            launch_arguments={"headless": "true", "use_rviz": "false", "use_nav2": "true",
                              "nav2_params": LaunchConfiguration("params_file")}.items()),
        TimerAction(period=2.0, actions=walls),
    ])
