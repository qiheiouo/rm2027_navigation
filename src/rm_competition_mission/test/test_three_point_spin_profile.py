import math
import os
from pathlib import Path
from xml.etree import ElementTree

import yaml


SOURCE = Path(os.environ["RM_COMPETITION_MISSION_SOURCE_DIR"].lstrip(os.pathsep))
MISSION = SOURCE / "config" / "mission_fresh03_three_point_spin_test.yaml"
TREE = SOURCE / "trees" / "competition_three_point_spin_test.xml"
NAV2 = (
    SOURCE.parent
    / "rm_nav_config"
    / "config"
    / "nav2_old_car_2026_left_stvl_three_point_spin_test.yaml"
)
COMPETITION_LAUNCH = (
    SOURCE.parent
    / "rm_navigation_bringup"
    / "launch"
    / "old_car_2026_competition.launch.py"
)
OLD_CAR_LAUNCH = COMPETITION_LAUNCH.with_name("old_car_2026_validation.launch.py")
TEST_LAUNCH = COMPETITION_LAUNCH.with_name(
    "old_car_2026_three_point_spin_test.launch.py"
)


def _mission_parameters():
    document = yaml.safe_load(MISSION.read_text(encoding="utf-8"))
    return document["competition_mission_node"]["ros__parameters"]


def test_tree_prioritizes_low_hp_home_before_spin_patrol():
    root = ElementTree.parse(TREE).getroot()
    strategy = root.find(".//ReactiveFallback[@name='strategy_priority']")
    assert strategy is not None
    sequences = strategy.findall("Sequence")
    assert [node.get("name") for node in sequences] == [
        "low_hp_return_home",
        "three_point_patrol_with_spin",
    ]
    assert sequences[0].find("ShouldReturnHome") is not None
    assert sequences[0].find("SelectHome") is not None
    assert sequences[1].find("SelectPatrolWithSpin") is not None


def test_mission_profile_is_disabled_and_expresses_requested_contract():
    params = _mission_parameters()
    assert params["startup_enabled"] is False
    assert params["default_mode"] == "hold"
    assert params["spin_action"] == "/spin"
    assert params["retreat_hp_threshold"] == 199
    assert params["patrol_spin_angular_velocity"] == 10.0
    assert params["patrol_spin_duration_sec"] == 10.0
    assert params["spin_time_allowance_sec"] >= 10.0
    assert math.isclose(
        params["patrol_spin_angular_velocity"]
        * params["patrol_spin_duration_sec"],
        100.0,
    )
    assert len(params["home_pose"]) == 3
    assert len(params["patrol_waypoints"]) == 9


def test_nav2_candidate_has_requested_limits_without_changing_baseline():
    candidate = yaml.safe_load(NAV2.read_text(encoding="utf-8"))
    controller = candidate["controller_server"]["ros__parameters"]["FollowPath"]
    behavior = candidate["behavior_server"]["ros__parameters"]
    smoother = candidate["velocity_smoother"]["ros__parameters"]
    assert controller["vx_max"] == 3.0
    assert controller["vx_min"] == -3.0
    assert controller["vy_max"] == 3.0
    assert controller["wz_max"] == 10.0
    assert behavior["max_rotational_vel"] == 10.0
    assert smoother["max_velocity"] == [3.0, 3.0, 10.0]
    assert smoother["min_velocity"] == [-3.0, -3.0, -10.0]

    baseline = yaml.safe_load(
        (SOURCE.parent / "rm_nav_config" / "config" / "nav2_old_car_2026_left_stvl.yaml")
        .read_text(encoding="utf-8")
    )
    baseline_controller = baseline["controller_server"]["ros__parameters"]["FollowPath"]
    assert baseline_controller["vx_max"] == 0.5
    assert baseline_controller["wz_max"] == 1.2


def test_mission_never_publishes_velocity_directly():
    source = (SOURCE / "src" / "competition_mission_node.cpp").read_text(
        encoding="utf-8"
    )
    assert "geometry_msgs/msg/twist" not in source
    assert 'create_publisher<geometry_msgs::msg::Twist' not in source
    assert '"/cmd_vel"' not in source
    assert "nav2_msgs/action/spin.hpp" in source


def test_competition_launch_exposes_candidate_tree_and_projection_overrides():
    launch = COMPETITION_LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("mission_tree_xml"' in launch
    assert '"tree_xml": mission_tree_xml' in launch
    assert '"scan_projection_params": scan_projection_params' in launch
    for key in ("serial_max_vx", "serial_max_vy", "serial_max_wz"):
        assert f'DeclareLaunchArgument("{key}"' in launch
        assert f'"{key}": {key}' in launch

    old_car_launch = OLD_CAR_LAUNCH.read_text(encoding="utf-8")
    for key, default in (
        ("serial_max_vx", "0.50"),
        ("serial_max_vy", "0.50"),
        ("serial_max_wz", "1.20"),
    ):
        assert f'DeclareLaunchArgument("{key}", default_value="{default}")' in old_car_launch
        assert f'"max_{key[-2:]}": {key}' in old_car_launch


def test_test_launch_is_opt_in_and_binds_all_high_spin_profiles():
    launch = TEST_LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enable_competition_stack", default_value="false")' in launch
    assert 'DeclareLaunchArgument("use_real_serial", default_value="false")' in launch
    assert 'DeclareLaunchArgument("use_mission", default_value="false")' in launch
    assert '"mission_startup_enabled": "false"' in launch
    assert '"serial_max_vx": "3.0"' in launch
    assert '"serial_max_vy": "3.0"' in launch
    assert '"serial_max_wz": "10.0"' in launch
    for filename in (
        "nav2_old_car_2026_left_stvl_three_point_spin_test.yaml",
        "amcl_2d_spin_robust_candidate.yaml",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
        "mission_fresh03_three_point_spin_test.yaml",
        "competition_three_point_spin_test.xml",
    ):
        assert f'"{filename}"' in launch
