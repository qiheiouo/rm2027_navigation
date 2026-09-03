import hashlib
import os
from pathlib import Path
import sys
from xml.etree import ElementTree

import pytest
import yaml


SOURCE = Path(os.environ["RM_COMPETITION_MISSION_SOURCE_DIR"].lstrip(os.pathsep))
sys.path.insert(0, str(SOURCE))

from rm_competition_mission.strategy_profiles import (  # noqa: E402
    StrategyError,
    list_profiles,
    make_field_strategy_template,
    resolve_strategy,
)


def _map_bundle(tmp_path: Path, occupied: tuple[int, int] | None = None) -> Path:
    width = 20
    height = 20
    pixels = [254] * (width * height)
    if occupied is not None:
        column, map_row = occupied
        image_row = height - 1 - map_row
        pixels[image_row * width + column] = 0
    (tmp_path / "field.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode() + bytes(pixels)
    )
    (tmp_path / "field.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "field.pgm",
                "resolution": 1.0,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    manifest = tmp_path / "field.bundle.yaml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "map_id": "test_field",
                "revision": "rev1",
                "deployment_status": "candidate",
                "frame_id": "map",
                "artifacts": {
                    "occupancy": {
                        "yaml_path": "field.yaml",
                        "image_path": "field.pgm",
                    }
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return manifest


def _strategy(
    tmp_path: Path,
    manifest: Path,
    profile: str = "offense",
    patrol_x: float = 8.0,
) -> Path:
    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    strategy = tmp_path / f"{profile}.strategy.yaml"
    strategy.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "strategy_id": f"test_{profile}",
                "profile": profile,
                "map_binding": {
                    "map_id": "test_field",
                    "map_revision": "rev1",
                    "manifest_sha256": digest,
                },
                "minimum_goal_clearance_m": 1.0,
                "waypoints": {
                    "home": {"x": 5.0, "y": 5.0, "yaw": 0.0},
                    "patrol": [{"x": patrol_x, "y": 8.0, "yaw": 1.57}],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return strategy


def test_catalog_has_only_reviewed_profiles():
    assert set(list_profiles(SOURCE)) == {"safe", "offense", "defense", "patrol_spin"}


def test_safe_profile_is_coordinate_free_hold():
    result = resolve_strategy(SOURCE, "safe")
    assert result.tree_path.name == "competition_hold.xml"
    assert result.home_pose_count == 0
    assert result.patrol_waypoint_count == 0
    assert "home_pose" not in result.parameters
    assert "patrol_waypoints" not in result.parameters
    root = ElementTree.parse(result.tree_path).getroot()
    assert root.find(".//SelectHold") is not None
    assert root.find(".//SelectPatrol") is None


@pytest.mark.parametrize(
    ("profile", "tree", "allow_pursuit", "threshold"),
    [
        ("offense", "competition_default.xml", True, 100),
        ("defense", "competition_defense.xml", False, 250),
        ("patrol_spin", "competition_three_point_spin_test.xml", False, 199),
    ],
)
def test_field_profile_resolves_paired_tree_config_and_coordinates(
    tmp_path, profile, tree, allow_pursuit, threshold
):
    manifest = _map_bundle(tmp_path)
    strategy = _strategy(tmp_path, manifest, profile)
    result = resolve_strategy(SOURCE, profile, strategy, manifest)
    assert result.tree_path.name == tree
    assert result.parameters["allow_pursuit"] is allow_pursuit
    assert result.parameters["retreat_hp_threshold"] == threshold
    assert result.parameters["startup_enabled"] is False
    assert result.parameters["home_pose"] == [5.0, 5.0, 0.0]
    assert result.parameters["patrol_waypoints"] == [8.0, 8.0, 1.57]
    assert result.map_manifest_sha256 == hashlib.sha256(manifest.read_bytes()).hexdigest()


def test_field_profile_requires_exact_map_hash(tmp_path):
    manifest = _map_bundle(tmp_path)
    strategy = _strategy(tmp_path, manifest)
    document = yaml.safe_load(strategy.read_text(encoding="utf-8"))
    document["map_binding"]["manifest_sha256"] = "0" * 64
    strategy.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(StrategyError, match="map binding mismatch"):
        resolve_strategy(SOURCE, "offense", strategy, manifest)


def test_profile_name_mismatch_is_rejected(tmp_path):
    manifest = _map_bundle(tmp_path)
    strategy = _strategy(tmp_path, manifest, "offense")
    with pytest.raises(StrategyError, match="does not match selected profile"):
        resolve_strategy(SOURCE, "defense", strategy, manifest)


def test_occupied_or_insufficient_clearance_waypoint_is_rejected(tmp_path):
    manifest = _map_bundle(tmp_path, occupied=(8, 8))
    strategy = _strategy(tmp_path, manifest, patrol_x=8.0)
    with pytest.raises(StrategyError, match="lacks 1.000 m map clearance"):
        resolve_strategy(SOURCE, "offense", strategy, manifest)


def test_field_strategy_cannot_lower_profile_clearance(tmp_path):
    manifest = _map_bundle(tmp_path)
    strategy = _strategy(tmp_path, manifest)
    document = yaml.safe_load(strategy.read_text(encoding="utf-8"))
    document["minimum_goal_clearance_m"] = 0.1
    strategy.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(StrategyError, match=r"must be in \[0.35, 2\]"):
        resolve_strategy(SOURCE, "offense", strategy, manifest)


def test_unknown_field_key_is_rejected(tmp_path):
    manifest = _map_bundle(tmp_path)
    strategy = _strategy(tmp_path, manifest)
    document = yaml.safe_load(strategy.read_text(encoding="utf-8"))
    document["tree_xml"] = "/tmp/unreviewed.xml"
    strategy.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(StrategyError, match="unknown keys"):
        resolve_strategy(SOURCE, "offense", strategy, manifest)


def test_non_safe_profiles_require_field_strategy_and_safe_rejects_it(tmp_path):
    with pytest.raises(StrategyError, match="requires a field strategy"):
        resolve_strategy(SOURCE, "offense")
    field = tmp_path / "unused.yaml"
    field.write_text("{}\n", encoding="utf-8")
    with pytest.raises(StrategyError, match="does not accept"):
        resolve_strategy(SOURCE, "safe", field)


def test_template_generation_binds_exact_manifest(tmp_path):
    manifest = _map_bundle(tmp_path)
    template = make_field_strategy_template(SOURCE, "defense", manifest)
    assert template["profile"] == "defense"
    assert template["map_binding"] == {
        "map_id": "test_field",
        "map_revision": "rev1",
        "manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
    }
    assert template["minimum_goal_clearance_m"] == 0.35
    assert template["waypoints"]["home"]["x"] is None
    generated = tmp_path / "generated.strategy.yaml"
    generated.write_text(yaml.safe_dump(template), encoding="utf-8")
    with pytest.raises(StrategyError, match="must be numeric"):
        resolve_strategy(SOURCE, "defense", generated, manifest)
    with pytest.raises(StrategyError, match="does not use"):
        make_field_strategy_template(SOURCE, "safe", manifest)


def test_launch_chain_exposes_single_profile_and_strategy_file():
    mission_launch = (SOURCE / "launch" / "competition_mission.launch.py").read_text(
        encoding="utf-8"
    )
    bringup_launch = (
        SOURCE.parent
        / "rm_navigation_bringup"
        / "launch"
        / "old_car_2026_competition.launch.py"
    ).read_text(encoding="utf-8")
    user_launch = (
        SOURCE.parent
        / "rm_navigation_launch"
        / "launch"
        / "old_car_full_navigation.launch.py"
    ).read_text(encoding="utf-8")
    for text in (mission_launch, bringup_launch, user_launch):
        assert "mission_strategy_profile" in text or "strategy_profile" in text
        assert "mission_strategy_file" in text or "strategy_file" in text
    assert '"map_bundle_manifest": map_manifest' in bringup_launch
