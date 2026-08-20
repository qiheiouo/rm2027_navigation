from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_is_default_off_and_has_no_control_outputs() -> None:
    launch_text = (
        PACKAGE_ROOT / "launch" / "semantic_path_annotation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enabled", default_value="false")' in launch_text
    assert "IfCondition(LaunchConfiguration(\"enabled\"))" in launch_text
    assert "/cmd_vel" not in launch_text
    assert "NavigateToPose" not in launch_text
    assert "tf2_ros" not in launch_text
