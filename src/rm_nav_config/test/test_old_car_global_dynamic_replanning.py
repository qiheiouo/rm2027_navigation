import math
import os
from pathlib import Path
import re
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
    assert "dog_hole_scan_projection" not in dog_hole_launch
    assert "pointcloud_to_laserscan_node.ros__parameters.max_height" not in dog_hole_launch

    dual = _document("nav2_old_car_2026_dual_stvl.yaml")
    controller = dual["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = dual["velocity_smoother"]["ros__parameters"]
    assert controller["vx_max"] == 0.50
    assert smoother["max_velocity"][0] == 3.00
    assert smoother["max_velocity"][2] >= 0.80


def test_old_car_ramp_entry_defaults_to_unlabelled_shadow_and_keeps_map_fallback():
    launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_launch"
        / "launch"
        / "old_car_ramp_validation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("activate_filter", default_value="false")' in launch
    assert 'DeclareLaunchArgument("detection_mode", default_value="automatic")' in launch
    assert '"map_bundle_override"' in launch
    assert '"nav2_base_config_yaml"' in launch
    assert 'LaunchConfiguration("map_bundle_yaml")' not in launch
    assert 'executable = "automatic_ramp_filter_node"' in launch
    assert 'executable = "ramp_plane_filter_node"' in launch
    assert '"/perception/ramp/localization_filtered_shadow"' in launch
    assert '"/perception/ramp/obstacles_filtered_shadow"' in launch
    assert '"/livox/left/pointcloud_ramp_filtered"' in launch
    assert '"/points/obstacles_ramp_filtered"' in launch
    assert '"controller_server.ros__parameters.FollowPath.motion_model": "DiffDrive"' in launch
    assert "fused_mid360_mark.topic" in launch
    assert 'DeclareLaunchArgument("serial_cmd_vel_topic", default_value="/cmd_vel")' in launch
    assert '"serial_cmd_vel_topic": LaunchConfiguration("serial_cmd_vel_topic")' in launch

    automatic = yaml.safe_load(
        (
            WORKSPACE_SOURCE
            / "rm_navigation_launch"
            / "config"
            / "old_car_automatic_ramp_filter.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert 0.0 < automatic["detection.min_slope_deg"]
    assert automatic["detection.max_slope_deg"] < 45.0
    assert automatic["detection.min_width"] >= 0.55
    assert automatic["tracking.confirmation_frames"] >= 3

    full_launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_launch"
        / "launch"
        / "old_car_full_navigation.launch.py"
    ).read_text(encoding="utf-8")
    competition_launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_bringup"
        / "launch"
        / "old_car_2026_competition.launch.py"
    ).read_text(encoding="utf-8")
    assert '"localization_pointcloud_topic"' in full_launch
    assert '"amcl_pointcloud_topic"' in full_launch
    assert '"amcl_pointcloud_topic"' in competition_launch
    assert 'pointcloud_topic": amcl_pointcloud_topic' in competition_launch

    regions = yaml.safe_load(
        (
            WORKSPACE_SOURCE
            / "rm_navigation_launch"
            / "config"
            / "old_car_ramp_regions.example.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert regions["expected_map_id"].startswith("replace_with")
    assert regions["expected_map_revision"].startswith("replace_with")
    assert regions["region_names"] == ["portable_ramp"]
    assert 0.0 < regions["regions.portable_ramp.surface_tolerance"] <= 0.20


def test_old_car_full_terrain_entry_combines_profiles_without_duplicate_stack():
    launch = (
        WORKSPACE_SOURCE
        / "rm_navigation_launch"
        / "launch"
        / "old_car_full_terrain_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert '"old_car_ramp_validation.launch.py"' in launch
    assert 'DeclareLaunchArgument("activate_ramp_filter", default_value="false")' in launch
    assert 'DeclareLaunchArgument("enable_dog_hole_profile", default_value="false")' in launch
    assert '"nav2_base_config_yaml": selected_nav2' in launch
    assert '"map_bundle_override": map_override' in launch
    assert "fused_mid360_mark.max_obstacle_height" in launch
    assert "obstacle_layer.localization_scan.marking" in launch
    assert re.search(
        r'"obstacle_layer\.localization_scan\.marking"\s*\): "false"',
        launch,
    )
    assert "pointcloud_to_laserscan_node" not in launch
    assert launch.count("IncludeLaunchDescription(") == 1
    assert 'serial_cmd_vel_topic = "/cmd_vel"' in launch
    assert 'serial_cmd_vel_topic = "/cmd_vel_dog_hole_gated"' in launch
    assert '"serial_cmd_vel_topic": serial_cmd_vel_topic' in launch
