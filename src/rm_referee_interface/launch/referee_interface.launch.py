from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration("enable_referee_interface")
    use_mock = LaunchConfiguration("use_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mock_game_progress = LaunchConfiguration("mock_game_progress")
    mock_stage_remain_time = LaunchConfiguration("mock_stage_remain_time")
    mock_robot_id = LaunchConfiguration("mock_robot_id")
    mock_current_hp = LaunchConfiguration("mock_current_hp")
    mock_self_outpost_hp = LaunchConfiguration("mock_self_outpost_hp")
    mock_enemy_outpost_hp = LaunchConfiguration("mock_enemy_outpost_hp")
    mock_projectiles = LaunchConfiguration("mock_projectile_allowance_17mm")
    mock_coins = LaunchConfiguration("mock_remaining_gold_coin")

    return LaunchDescription([
        DeclareLaunchArgument("enable_referee_interface", default_value="false"),
        DeclareLaunchArgument("use_mock", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mock_game_progress", default_value="4"),
        DeclareLaunchArgument("mock_stage_remain_time", default_value="300"),
        DeclareLaunchArgument("mock_robot_id", default_value="7"),
        DeclareLaunchArgument("mock_current_hp", default_value="400"),
        DeclareLaunchArgument("mock_self_outpost_hp", default_value="1500"),
        DeclareLaunchArgument("mock_enemy_outpost_hp", default_value="1500"),
        DeclareLaunchArgument("mock_projectile_allowance_17mm", default_value="100"),
        DeclareLaunchArgument("mock_remaining_gold_coin", default_value="0"),
        Node(
            condition=IfCondition(enabled),
            package="rm_referee_interface",
            executable="referee_state_gate",
            name="referee_state_gate",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "game_progress": ParameterValue(mock_game_progress, value_type=int),
                "stage_remain_time": ParameterValue(
                    mock_stage_remain_time, value_type=int
                ),
                "robot_id": ParameterValue(mock_robot_id, value_type=int),
                "current_hp": ParameterValue(mock_current_hp, value_type=int),
                "self_outpost_hp": ParameterValue(
                    mock_self_outpost_hp, value_type=int
                ),
                "enemy_outpost_hp": ParameterValue(
                    mock_enemy_outpost_hp, value_type=int
                ),
                "projectile_allowance_17mm": ParameterValue(
                    mock_projectiles, value_type=int
                ),
                "remaining_gold_coin": ParameterValue(mock_coins, value_type=int),
            }],
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "'", enabled, "'.lower() in ['1','true','yes','on'] and '",
                use_mock, "'.lower() in ['1','true','yes','on']",
            ])),
            package="rm_referee_interface",
            executable="referee_state_mock",
            name="referee_state_mock",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
