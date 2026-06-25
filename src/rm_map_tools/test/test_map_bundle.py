from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import pytest
import yaml

from rm_map_tools import (
    MapBundleError,
    resolve_map_bundle_for_runtime,
    validate_map_bundle,
)


FIXTURE = Path(__file__).parents[1] / "maps" / "phase2e_test"


def _copy_fixture(tmp_path: Path) -> Path:
    target = tmp_path / "bundle"
    shutil.copytree(FIXTURE, target)
    return target / "phase2e_test.bundle.yaml"


def _load_manifest(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _save_manifest(path: Path, data: dict) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_validates_synthetic_bundle() -> None:
    result = validate_map_bundle(FIXTURE / "phase2e_test.bundle.yaml")
    assert result["map_id"] == "phase2e_synthetic"
    assert result["frame_id"] == "map"
    assert result["pcd"]["points"] == 4
    assert result["occupancy"]["width"] == 4
    assert result["occupancy"]["height"] == 4


def test_require_approved_rejects_test_fixture() -> None:
    with pytest.raises(MapBundleError, match="not approved"):
        validate_map_bundle(
            FIXTURE / "phase2e_test.bundle.yaml",
            require_approved=True,
        )


def test_require_approved_accepts_reviewed_copy(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["deployment_status"] = "approved"
    _save_manifest(manifest, data)
    result = validate_map_bundle(manifest, require_approved=True)
    assert result["deployment_status"] == "approved"


def test_runtime_resolver_rejects_test_fixture_by_default() -> None:
    with pytest.raises(MapBundleError, match="not approved"):
        resolve_map_bundle_for_runtime(FIXTURE / "phase2e_test.bundle.yaml")


def test_runtime_resolver_returns_nav2_and_pcd_paths_when_allowed() -> None:
    result = resolve_map_bundle_for_runtime(
        FIXTURE / "phase2e_test.bundle.yaml",
        allow_test_map=True,
    )
    assert result["deployment_status"] == "test_only"
    assert result["frame_id"] == "map"
    assert result["pcd_path"].endswith("phase2e_test.pcd")
    assert result["occupancy_yaml_path"].endswith("phase2e_test.yaml")


def test_rejects_hash_mismatch(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    with (manifest.parent / "phase2e_test.pcd").open("a", encoding="ascii") as stream:
        stream.write("2.0 2.0 0.0 1.0\n")
    with pytest.raises(MapBundleError, match="sha256 mismatch"):
        validate_map_bundle(manifest)


def test_rejects_artifact_path_escape(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    outside = tmp_path / "outside.pcd"
    outside.write_text("not a pcd", encoding="ascii")
    data = _load_manifest(manifest)
    data["artifacts"]["pcd"]["path"] = "../outside.pcd"
    data["artifacts"]["pcd"]["sha256"] = _hash(outside)
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="escapes"):
        validate_map_bundle(manifest)


def test_rejects_pcd_without_xyz(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    pcd = manifest.parent / "phase2e_test.pcd"
    text = pcd.read_text(encoding="ascii").replace(
        "FIELDS x y z intensity", "FIELDS x y intensity ring"
    )
    pcd.write_text(text, encoding="ascii")
    data = _load_manifest(manifest)
    data["artifacts"]["pcd"]["sha256"] = _hash(pcd)
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="include x, y and z"):
        validate_map_bundle(manifest)


def test_rejects_invalid_occupancy_thresholds(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    occupancy_yaml = manifest.parent / "phase2e_test.yaml"
    occupancy = yaml.safe_load(occupancy_yaml.read_text(encoding="utf-8"))
    occupancy["free_thresh"] = 0.8
    occupancy_yaml.write_text(
        yaml.safe_dump(occupancy, sort_keys=False), encoding="utf-8"
    )
    data = _load_manifest(manifest)
    data["artifacts"]["occupancy"]["yaml_sha256"] = _hash(occupancy_yaml)
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="0 <= free < occupied <= 1"):
        validate_map_bundle(manifest)
