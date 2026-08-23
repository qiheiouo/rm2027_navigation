import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


SOURCE = Path(os.environ["RM_NAV_CONFIG_SOURCE_DIR"].lstrip(os.pathsep))
CONFIG = SOURCE / "config"
WORKSPACE_SOURCE = SOURCE.parent

STVL_PROFILES = (
    "nav2_old_car_2026_left_stvl.yaml",
    "nav2_old_car_2026_left_stvl_three_point_spin_test.yaml",
    "nav2_old_car_2026_dual_stvl.yaml",
)


def _parameters(filename):
    document = yaml.safe_load((CONFIG / filename).read_text(encoding="utf-8"))
    return document["global_costmap"]["global_costmap"]["ros__parameters"]


def _document(filename):
    return yaml.safe_load((CONFIG / filename).read_text(encoding="utf-8"))


def test_stvl_profiles_make_dynamic_obstacles_visible_to_global_planner():
    for filename in STVL_PROFILES:
        params = _parameters(filename)
        assert params["rolling_window"] is False, filename
        assert params["track_unknown_space"] is True, filename
        assert "width" not in params, filename
        assert "height" not in params, filename
        assert params["plugins"] == [
            "static_layer",
            "obstacle_layer",
            "inflation_layer",
        ], filename

        static = params["static_layer"]
        assert static["plugin"] == "nav2_costmap_2d::StaticLayer", filename
        assert static["enabled"] is True, filename

        obstacle = params["obstacle_layer"]
        assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer", filename
        assert obstacle["enabled"] is True, filename
        assert obstacle["footprint_clearing_enabled"] is True, filename
        assert obstacle["combination_method"] == 1, filename
        assert obstacle["observation_sources"] == "localization_scan", filename

        scan = obstacle["localization_scan"]
        assert scan["topic"] == "/localization/scan", filename
        assert scan["data_type"] == "LaserScan", filename
        assert scan["marking"] is True, filename
        assert scan["clearing"] is True, filename
        assert scan["inf_is_valid"] is True, filename
        assert scan["observation_persistence"] >= 2.0, filename
        assert scan["expected_update_rate"] > 0.0, filename
        assert scan["obstacle_max_range"] == scan["raytrace_max_range"], filename


def test_stvl_profiles_keep_the_existing_navfn_and_rectangular_footprint():
    for filename in STVL_PROFILES:
        document = _document(filename)
        planner = document["planner_server"]["ros__parameters"]["GridBased"]
        assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner", filename
        assert planner["use_astar"] is False, filename
        assert planner["allow_unknown"] is True, filename

        global_params = document["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        global_footprint = yaml.safe_load(global_params["footprint"])
        local_params = document["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
        local_footprint = yaml.safe_load(local_params["footprint"])
        expected_footprint = [
            [-0.32, -0.27],
            [-0.32, 0.27],
            [0.32, 0.27],
            [0.32, -0.27],
        ]
        assert global_footprint == expected_footprint, filename
        assert local_footprint == expected_footprint, filename
        assert (
            global_params["footprint_padding"]
            == local_params["footprint_padding"]
        ), filename


def test_stvl_profiles_reject_small_obstacle_front_oscillation_as_progress():
    for filename in STVL_PROFILES:
        document = _document(filename)
        progress = document["controller_server"]["ros__parameters"][
            "progress_checker"
        ]
        assert progress["plugin"] == (
            "nav2_controller::PoseProgressChecker"
        ), filename
        assert progress["required_movement_radius"] >= 0.15, filename
        assert progress["required_movement_angle"] >= 0.50, filename
        assert progress["movement_time_allowance"] <= 4.0, filename


def test_stvl_profiles_replan_only_after_follow_failure_or_goal_update():
    expected = (
        "$(find-pkg-share rm_nav_config)/behavior_trees/"
        "old_car_replan_on_follow_failure.xml"
    )
    for filename in STVL_PROFILES:
        document = _document(filename)
        params = document["bt_navigator"]["ros__parameters"]
        assert params["default_nav_to_pose_bt_xml"] == expected, filename

    tree_path = (
        SOURCE / "behavior_trees" / "old_car_replan_on_follow_failure.xml"
    )
    root = ET.parse(tree_path).getroot()
    assert root.attrib["main_tree_to_execute"] == "MainTree"

    recovery = root.find("./BehaviorTree/RecoveryNode")
    assert recovery is not None
    assert int(recovery.attrib["number_of_retries"]) == 6
    recovery_children = list(recovery)
    assert [child.tag for child in recovery_children] == [
        "ReactiveSequence",
        "Wait",
    ]

    event_driven = recovery_children[0]
    assert event_driven.find("./Inverter/GlobalUpdatedGoal") is not None
    plan_and_follow = event_driven.find("./Sequence[@name='PlanOnceAndFollow']")
    assert plan_and_follow is not None
    assert [child.tag for child in list(plan_and_follow)] == [
        "ComputePathToPose",
        "FollowPath",
    ]
    assert plan_and_follow.find("./ComputePathToPose").attrib["path"] == "{path}"
    assert plan_and_follow.find("./FollowPath").attrib["path"] == "{path}"

    wait = recovery_children[1]
    assert float(wait.attrib["wait_duration"]) >= 1.0

    # Planning is event-driven and all automatic recovery is motionless.
    compute_nodes = root.findall(".//ComputePathToPose")
    assert compute_nodes == [plan_and_follow.find("./ComputePathToPose")]
    assert root.findall(".//RateController") == []
    assert root.findall(".//IsPathValid") == []
    assert root.findall(".//ClearEntireCostmap") == []
    assert root.findall(".//Spin") == []
    assert root.findall(".//BackUp") == []


def test_amcl_projection_supplies_full_horizontal_clearing_scan():
    projection_dir = WORKSPACE_SOURCE / "rm_relocalization_bridge" / "config"
    for filename in (
        "pointcloud_to_scan_2d.yaml",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
    ):
        document = yaml.safe_load(
            (projection_dir / filename).read_text(encoding="utf-8")
        )
        params = document["pointcloud_to_laserscan_node"]["ros__parameters"]
        assert params["output_topic"] == "/localization/scan", filename
        assert params["angle_min"] <= -math.pi, filename
        assert params["angle_max"] >= math.pi, filename
        assert params["range_min"] <= 0.45, filename
        assert params["range_max"] >= 6.0, filename

    converter = (
        WORKSPACE_SOURCE
        / "rm_mid360_driver_bridge"
        / "src"
        / "pointcloud_to_laserscan_node.cpp"
    ).read_text(encoding="utf-8")
    assert (
        "ranges.assign(bin_count_, std::numeric_limits<float>::infinity())"
        in converter
    )


def test_full_old_car_entry_uses_dual_local_avoidance_and_amcl_global_scan():
    launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_launch"
        / "launch"
        / "old_car_full_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'RELOCALIZATION_BACKEND = "amcl_2d"' in launch
    assert '"pointcloud_to_scan_2d_spin_robust_candidate.yaml"' in launch
    assert '"nav2_old_car_2026_dual_stvl.yaml"' in launch
    assert 'FEATURES.add("dual_fusion")' in launch
    assert 'FEATURES.add("right_lidar")' in launch
    assert "ALLOW_PROVISIONAL_DUAL_EXTRINSIC = True" in launch

    dog_hole_launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_launch"
        / "launch"
        / "old_car_dog_hole_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert '"nav2_old_car_2026_dual_stvl.yaml"' in dog_hole_launch
    assert '"fused_mid360_mark.max_obstacle_height"' in dog_hole_launch
    assert '"local_inflation_radius"' in dog_hole_launch
    assert '"global_inflation_radius"' in dog_hole_launch
    assert '"inflation_radius": local_inflation_radius' in dog_hole_launch
    assert '"inflation_radius": global_inflation_radius' in dog_hole_launch

    dual = _document("nav2_old_car_2026_dual_stvl.yaml")
    controller = dual["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = dual["velocity_smoother"]["ros__parameters"]
    assert controller["vx_max"] == 0.50
    assert smoother["max_velocity"][0] == 3.00
    assert smoother["max_velocity"][2] >= 0.80
