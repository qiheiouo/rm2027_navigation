from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_launch_is_default_off_and_node_has_no_control_outputs():
    launch = (
        PACKAGE_ROOT / "launch" / "dynamic_clearance_shadow.launch.py"
    ).read_text(encoding="utf-8")
    node = (
        PACKAGE_ROOT
        / "rm_dynamic_clearance"
        / "dynamic_clearance_node.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enabled", default_value="false")' in launch
    assert "IfCondition(LaunchConfiguration(\"enabled\"))" in launch
    assert "create_publisher(\n            DynamicClearanceReport" in node
    assert "prediction_stamp.nanoseconds <= 0" in node
    assert "lookup_transform(" in node
    assert "Time()" not in node
    for forbidden_type in (
        "Twist",
        "NavigateToPose",
        "PostureRequest",
        "OccupancyGrid",
        "TransformBroadcaster",
    ):
        assert forbidden_type not in node


def test_tracker_prediction_output_is_versioned_and_declares_horizon():
    interface = (
        PACKAGE_ROOT.parent
        / "rm_competition_interfaces"
        / "msg"
        / "DynamicObstaclePredictionArray.msg"
    ).read_text(encoding="utf-8")
    tracker = (
        PACKAGE_ROOT.parent
        / "rm_dynamic_obstacle_tracking"
        / "rm_dynamic_obstacle_tracking"
        / "dynamic_obstacle_tracker_node.py"
    ).read_text(encoding="utf-8")
    assert "rm_dynamic_obstacle_predictions/v1" in interface
    assert "float64 prediction_dt" in interface
    assert "uint32 prediction_steps" in interface
    assert "bool complete" in interface
    assert "uint32 total_track_count" in interface
    assert "DynamicObstaclePredictionArray" in tracker
    assert "output.prediction_steps = self._prediction_steps" in tracker
    assert "output.complete = len(tracks) <= self._prediction_max_tracks" in tracker
    assert "ordered_tracks[: self._prediction_max_tracks]" in tracker
