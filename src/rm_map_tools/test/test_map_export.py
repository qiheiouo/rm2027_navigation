from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from rm_map_tools.map_bundle import MapBundleError, validate_map_bundle
from rm_map_tools.map_export import (
    occupancy_values_to_pgm,
    write_candidate_map_bundle,
)


def test_occupancy_pgm_flips_grid_rows_for_map_server() -> None:
    payload = occupancy_values_to_pgm(
        [0, 100, -1, 50],
        width=2,
        height=2,
    )
    assert payload.startswith(b"P5\n2 2\n255\n")
    pixels = payload.split(b"\n", 3)[3]
    assert pixels == bytes([205, 205, 254, 0])


def test_candidate_export_creates_self_consistent_bundle(tmp_path: Path) -> None:
    manifest = write_candidate_map_bundle(
        output_root=tmp_path,
        map_id="field_alpha",
        revision="r1",
        points=np.asarray([[0.0, 0.0, 0.1], [1.0, 2.0, 0.3]], dtype=np.float32),
        occupancy_values=[0, 100, -1, 0],
        occupancy_width=2,
        occupancy_height=2,
        occupancy_resolution=0.05,
        occupancy_origin=[-1.0, -2.0, 0.0],
        source_method="unit_test",
        created_utc="2026-07-10T00:00:00+00:00",
    )
    result = validate_map_bundle(manifest)
    assert result["deployment_status"] == "candidate"
    assert result["pcd"]["points"] == 2
    assert result["occupancy"]["width"] == 2
    assert result["occupancy"]["height"] == 2
    assert result["shared_origin_confirmed"] is True

    occupancy_yaml = yaml.safe_load(
        (manifest.parent / "field_alpha.yaml").read_text(encoding="utf-8")
    )
    unknown_probability = (255 - 205) / 255.0
    assert occupancy_yaml["free_thresh"] < unknown_probability
    assert unknown_probability < occupancy_yaml["occupied_thresh"]


def test_candidate_export_never_overwrites_revision(tmp_path: Path) -> None:
    arguments = dict(
        output_root=tmp_path,
        map_id="field_alpha",
        revision="r1",
        points=[[0.0, 0.0, 0.0]],
        occupancy_values=[0],
        occupancy_width=1,
        occupancy_height=1,
        occupancy_resolution=0.05,
        occupancy_origin=[0.0, 0.0, 0.0],
        source_method="unit_test",
    )
    write_candidate_map_bundle(**arguments)
    with pytest.raises(MapBundleError, match="already exists"):
        write_candidate_map_bundle(**arguments)


def test_candidate_export_rejects_all_unknown_occupancy(tmp_path: Path) -> None:
    with pytest.raises(MapBundleError, match="no known cells"):
        write_candidate_map_bundle(
            output_root=tmp_path,
            map_id="field_alpha",
            revision="all-unknown",
            points=[[0.0, 0.0, 0.0]],
            occupancy_values=[-1, -1, -1, -1],
            occupancy_width=2,
            occupancy_height=2,
            occupancy_resolution=0.05,
            occupancy_origin=[0.0, 0.0, 0.0],
            source_method="unit_test",
        )
