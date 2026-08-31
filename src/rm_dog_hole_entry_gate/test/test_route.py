from pathlib import Path

import pytest

from rm_dog_hole_entry_gate.route import (
    load_route,
    parse_route,
    validate_route_map_binding,
)
from rm_path_annotations.core import RegionContractError


SHA256 = "2" * 64


def _document() -> dict:
    return {
        "schema": "rm_dog_hole_route/v1",
        "route_id": "dog_hole_a",
        "revision": "rev1",
        "map_binding": {
            "frame_id": "map",
            "map_id": "field_map",
            "map_revision": "map_rev1",
            "manifest_sha256": SHA256,
        },
        "goal_trigger_polygon": [
            [3.2, -1.0],
            [6.0, -1.0],
            [6.0, 1.0],
            [3.2, 1.0],
        ],
        "stop_pose": {"x": 1.0, "y": 0.0, "yaw": 0.0},
        "exit_pose": {"x": 3.0, "y": 0.0, "yaw": 0.0},
    }


def test_route_selects_only_map_goals_inside_trigger() -> None:
    route = parse_route(_document())
    assert route.stages_goal("map", 4.0, 0.0)
    assert route.stages_goal("map", 3.2, 0.0)  # Boundary is included.
    assert not route.stages_goal("map", 1.0, 0.0)
    assert not route.stages_goal("odom", 4.0, 0.0)


def test_route_rejects_stop_pose_inside_trigger() -> None:
    document = _document()
    document["stop_pose"] = {"x": 4.0, "y": 0.0, "yaw": 0.0}
    with pytest.raises(RegionContractError, match="stop_pose must be outside"):
        parse_route(document)


def test_route_rejects_exit_pose_inside_trigger() -> None:
    document = _document()
    document["exit_pose"] = {"x": 4.0, "y": 0.0, "yaw": 0.0}
    with pytest.raises(RegionContractError, match="exit_pose must be outside"):
        parse_route(document)


def test_route_rejects_unknown_keys_and_duplicate_yaml(tmp_path: Path) -> None:
    document = _document()
    document["unreviewed"] = True
    with pytest.raises(RegionContractError, match="unknown keys"):
        parse_route(document)

    route_file = tmp_path / "duplicate.yaml"
    route_file.write_text(
        "schema: rm_dog_hole_route/v1\nschema: duplicate\n",
        encoding="utf-8",
    )
    with pytest.raises(RegionContractError, match="duplicate YAML key"):
        load_route(route_file)


def test_route_binding_must_match_exact_manifest() -> None:
    route = parse_route(_document())
    validate_route_map_binding(
        route,
        expected_map_id="field_map",
        expected_map_revision="map_rev1",
        expected_manifest_sha256=SHA256,
    )
    with pytest.raises(RegionContractError, match="map binding mismatch"):
        validate_route_map_binding(
            route,
            expected_map_id="field_map",
            expected_map_revision="map_rev2",
            expected_manifest_sha256=SHA256,
        )


def test_route_rejects_symlink(tmp_path: Path) -> None:
    target = tmp_path / "target.yaml"
    target.write_text("schema: invalid\n", encoding="utf-8")
    link = tmp_path / "route.yaml"
    link.symlink_to(target)
    with pytest.raises(RegionContractError, match="non-symlink"):
        load_route(link)
