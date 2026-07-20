from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from rm_map_tools.analyze_map_quality import validate_output_location
from rm_map_tools.map_bundle import MapBundleError, validate_map_bundle
from rm_map_tools.map_export import write_candidate_map_bundle
from rm_map_tools.map_quality import (
    OccupancyImage,
    analyze_quality,
    connected_components_8,
    euclidean_distance_to_mask,
    evaluate_labels,
    load_occupancy_image,
    project_points,
    read_ascii_xyz_pcd,
    read_rosbag_metadata,
    sha256_file,
    world_to_pgm,
    write_quality_outputs,
)
from rm_map_tools.sweep_map_projection import (
    evaluate_candidate,
    generate_sweep_candidates,
    generate_sweep_plan,
)
from rm_map_tools.verify_map_server import (
    compare_occupancy_message,
    expected_grid_from_manifest,
)


def _make_bundle(tmp_path: Path) -> Path:
    return write_candidate_map_bundle(
        output_root=tmp_path,
        map_id="quality_fixture",
        revision="r1",
        points=np.asarray(
            [
                [0.025, 0.025, 0.15],
                [0.075, 0.075, 0.50],
                [0.125, 0.025, 2.60],
            ],
            dtype=np.float32,
        ),
        occupancy_values=[100, 0, -1, 0, 100, -1],
        occupancy_width=3,
        occupancy_height=2,
        occupancy_resolution=0.05,
        occupancy_origin=[0.0, 0.0, 0.0],
        source_method="unit_test",
        created_utc="2026-07-17T00:00:00+00:00",
    )


def test_world_to_pgm_handles_row_flip_and_nonzero_yaw() -> None:
    occupancy = OccupancyImage(
        pixels=np.zeros((2, 3), dtype=np.uint8),
        occupied=np.zeros((2, 3), dtype=bool),
        free=np.zeros((2, 3), dtype=bool),
        unknown=np.zeros((2, 3), dtype=bool),
        resolution=1.0,
        origin=(10.0, 20.0, math.pi / 2.0),
        yaml_path=Path("fixture.yaml"),
        image_path=Path("fixture.pgm"),
    )
    # Local cell (0, 0) center rotates to world (9.5, 20.5), and appears
    # on the final PGM row because OccupancyGrid y=0 is the low local-Y row.
    rows, columns, inside = world_to_pgm(occupancy, [9.5, 8.5], [20.5, 22.5])
    assert rows.tolist() == [1, 0]
    assert columns.tolist() == [0, 2]
    assert inside.tolist() == [True, True]


def test_projection_uses_half_open_z_layers(tmp_path: Path) -> None:
    manifest = _make_bundle(tmp_path)
    bundle = validate_map_bundle(manifest)
    occupancy = load_occupancy_image(bundle["occupancy"]["yaml_path"])
    points, _ = read_ascii_xyz_pcd(bundle["pcd"]["path"])
    counts, selected, inside = project_points(points, occupancy, 0.10, 1.80)
    assert selected == 2
    assert inside == 2
    assert int(counts.sum()) == 2
    assert counts[1, 0] == 1
    assert counts[0, 1] == 1


def test_connected_components_use_eight_neighbors_and_four_edge_perimeter() -> None:
    mask = np.asarray(
        [[True, False, False], [False, True, False], [False, False, True]],
        dtype=bool,
    )
    labels, components = connected_components_8(mask)
    assert labels.max() == 1
    assert len(components) == 1
    assert components[0].size == 3
    assert components[0].perimeter_4 == 12


def test_euclidean_distance_transform_matches_known_distances() -> None:
    mask = np.zeros((3, 4), dtype=bool)
    mask[1, 1] = True
    distances = euclidean_distance_to_mask(mask, 0.05)
    assert distances[1, 1] == pytest.approx(0.0)
    assert distances[1, 3] == pytest.approx(0.10)
    assert distances[0, 0] == pytest.approx(math.sqrt(2.0) * 0.05)


def test_binary_pcd_is_rejected_explicitly(tmp_path: Path) -> None:
    pcd = tmp_path / "binary.pcd"
    pcd.write_bytes(
        b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        b"COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n"
        b"\x00" * 12
    )
    with pytest.raises(MapBundleError, match="requires ASCII PCD"):
        read_ascii_xyz_pcd(pcd)


def test_labels_measure_known_free_obstacles_walls_and_landmarks(tmp_path: Path) -> None:
    manifest = _make_bundle(tmp_path)
    bundle = validate_map_bundle(manifest)
    occupancy = load_occupancy_image(bundle["occupancy"]["yaml_path"])
    labels = {
        "path": str(tmp_path / "labels.yaml"),
        "known_free": [{
            "name": "clear_cell",
            "safety_critical": True,
            "polygon": [[0.05, 0.0], [0.10, 0.0], [0.10, 0.05], [0.05, 0.05]],
        }],
        "protected_obstacles": [{
            "name": "short_wall",
            "kind": "wall",
            "critical": True,
            "tolerance_m": 0.05,
            "sample_spacing_m": 0.05,
            "points": [[0.025, 0.025], [0.075, 0.075]],
        }],
        "landmarks": [
            {"name": "a", "point": [0.025, 0.025], "tolerance_m": 0.10},
            {"name": "b", "point": [0.075, 0.075], "tolerance_m": 0.10},
            {"name": "c", "point": [0.025, 0.025], "tolerance_m": 0.10},
        ],
    }
    result = evaluate_labels(
        occupancy, labels, sha256_file(bundle["occupancy"]["image_path"])
    )
    assert result["known_free"]["safety_critical_occupied_cells"] == 0
    assert result["protected_obstacles"]["critical_recall"] == 1.0
    assert result["protected_obstacles"]["maximum_wall_gap_m"] == 0.0
    assert result["landmarks"]["within_tolerance"] == 3


def test_bag_metadata_reports_every_missing_required_topic(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump({
            "rosbag2_bagfile_information": {
                "duration": {"nanoseconds": 14_000_000_000},
                "message_count": 709,
                "topics_with_message_count": [{
                    "topic_metadata": {
                        "name": "/livox/left/pointcloud",
                        "type": "sensor_msgs/msg/PointCloud2",
                    },
                    "message_count": 709,
                }],
            }
        }),
        encoding="utf-8",
    )
    result = read_rosbag_metadata(bag)
    assert result["duration_sec"] == pytest.approx(14.0)
    assert not result["topic_coverage_complete"]
    assert not result["ready_for_full_mapping_replay"]
    assert "/lio/cloud_registered_transformed" in result[
        "missing_or_empty_required_topics"
    ]


def test_analysis_writes_new_machine_readable_evidence(tmp_path: Path) -> None:
    manifest = _make_bundle(tmp_path)
    summary, components, images = analyze_quality(manifest)
    assert summary["map"]["width"] == 3
    assert summary["map"]["height"] == 2
    assert summary["map"]["occupied_pixels"] == 2
    assert summary["pcd"]["points_total"] == 3
    assert summary["pcd"]["points_in_0_10_to_1_80m"] == 2
    assert len(summary["pcd"]["height_layers"]) == 8

    output = tmp_path / "evidence"
    write_quality_outputs(output, summary, components, images)
    decoded = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert decoded["map"]["coordinate_contract"]["grid_to_pgm_row"] == (
        "height - 1 - grid_y"
    )
    assert (output / "components.csv").is_file()
    assert (output / "support_overlay.png").read_bytes().startswith(b"\x89PNG")
    assert len(list((output / "layers").glob("*.png"))) == 8
    with pytest.raises(MapBundleError, match="already exists"):
        write_quality_outputs(output, summary, components, images)


def test_cli_output_guard_rejects_bundle_and_non_tmp_paths(tmp_path: Path) -> None:
    manifest = _make_bundle(tmp_path)
    with pytest.raises(MapBundleError, match="must be a new child"):
        validate_output_location(tmp_path / "outside", manifest)
    bundle_output = manifest.parent / "diagnostics"
    with pytest.raises(MapBundleError):
        validate_output_location(bundle_output, manifest)


def test_map_server_comparison_checks_every_grid_cell_and_origin(tmp_path: Path) -> None:
    manifest = _make_bundle(tmp_path)
    expected = expected_grid_from_manifest(manifest)
    yaw = expected["origin"][2]
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            width=expected["width"],
            height=expected["height"],
            resolution=expected["resolution"],
            origin=SimpleNamespace(
                position=SimpleNamespace(
                    x=expected["origin"][0],
                    y=expected["origin"][1],
                    z=0.0,
                ),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
        ),
        data=expected["data"].tolist(),
    )
    result = compare_occupancy_message(message, expected)
    assert result["matches"]
    assert result["data_mismatches"] == 0

    message.data[0] = 42
    mismatch = compare_occupancy_message(message, expected)
    assert not mismatch["matches"]
    assert mismatch["data_mismatches"] == 1


def _ranking_summary(
    *, known_free: int, small: int, single_frame: int, replicated: int
) -> dict:
    return {
        "schema_version": 1,
        "bundle": {"map_id": "field", "revision": "r1"},
        "map": {"known_fraction": 0.50},
        "components": {
            "support": {"0.10": {"small_unsupported_pixels_le_25": small}}
        },
        "labels": {
            "status": "evaluated",
            "known_free": {
                "cells": 100,
                "occupied_cells": known_free,
                "safety_critical_occupied_cells": 0,
            },
            "protected_obstacles": {
                "recall": 1.0,
                "critical_points": 1,
                "critical_recall": 1.0,
                "wall_entries": 1,
                "maximum_wall_gap_m": 0.10,
            },
            "landmarks": {"count": 3, "maximum_deviation_m": 0.08},
        },
        "temporal_evidence": {"single_frame_only_occupied_pixels": single_frame},
        "replication": {"independent_datasets_passed": replicated},
    }


def test_candidate_ranking_enforces_noise_and_retention_gates() -> None:
    baseline = _ranking_summary(
        known_free=10, small=100, single_frame=20, replicated=2
    )
    candidate = _ranking_summary(
        known_free=0, small=60, single_frame=10, replicated=2
    )
    result = evaluate_candidate(baseline, candidate)
    assert result["eligible_for_parameter_selection"]
    assert result["hard_gate_failures"] == []

    candidate["labels"]["protected_obstacles"]["critical_recall"] = 0.99
    rejected = evaluate_candidate(baseline, candidate)
    assert not rejected["eligible_for_parameter_selection"]
    assert "critical_low_obstacle_recall_below_100_percent" in rejected[
        "hard_gate_failures"
    ]


def test_sweep_is_single_variable_first_and_blocks_without_two_bags() -> None:
    candidates = generate_sweep_candidates()
    assert candidates[0]["candidate_id"] == "baseline"
    assert len({candidate["candidate_id"] for candidate in candidates}) == len(candidates)
    assert all(
        len(candidate["changed_parameters"]) == 1
        for candidate in candidates
        if candidate["candidate_id"].startswith("onevar__")
    )
    plan = generate_sweep_plan([])
    assert plan["execution_status"] == "blocked_need_two_topic_complete_bags"
    assert plan["fixed_invariants"]["resolution"] == 0.05
    assert not plan["fixed_invariants"]["incremental_2D_projection"]


def test_current_field_baseline_regression_signature() -> None:
    repository = Path(__file__).resolve().parents[3]
    manifest = repository / (
        "artifacts/maps/old_car_clean_20260715_field01/20260715T025927Z/"
        "old_car_clean_20260715_field01.bundle.yaml"
    )
    if not manifest.is_file():
        pytest.skip("repository field baseline is not packaged")
    summary, _, _ = analyze_quality(manifest)
    assert summary["bundle"]["pcd_sha256"] == (
        "f0232b2f2c1560e901af03bbec6b32b6d80f5d94dc1a226178220ddb237a5b47"
    )
    assert summary["bundle"]["occupancy_image_sha256"] == (
        "af476fe72d2eb790ce3ec67c17618d1b510761079aa1c7ff2baa4dd99478a155"
    )
    assert summary["map"]["width"] == 333
    assert summary["map"]["height"] == 398
    assert summary["map"]["pixel_values"] == {
        "0": 15310,
        "205": 68920,
        "254": 48304,
    }
    assert summary["components"]["count"] == 725
    assert summary["components"]["le_25"] == 686
    assert summary["components"]["pixels_in_components_le_25"] == 1686
    assert summary["pcd"]["points_total"] == 12750
    assert summary["pcd"]["points_in_0_10_to_1_80m"] == 5776
    assert summary["components"]["support"]["0.10"][
        "unsupported_occupied_pixels"
    ] == 7095
