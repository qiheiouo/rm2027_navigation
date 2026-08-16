from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import pytest
import yaml

import rm_map_tools.map_bundle as map_bundle_module
from rm_map_tools import (
    MapBundleError,
    resolve_map_bundle_for_runtime,
    validate_map_bundle,
)
from rm_map_tools.map_export import write_candidate_map_bundle
from rm_map_tools.ray_observations import RaySidecarRecorder


FIXTURE = Path(__file__).parents[1] / "maps" / "phase2e_test"
CAPTURE_ID = "0123456789abcdef0123456789abcdef"
OTHER_CAPTURE_ID = "fedcba9876543210fedcba9876543210"


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


def _write_ray_bundle(tmp_path: Path) -> Path:
    sidecar = tmp_path / "source.rays.jsonl"
    with RaySidecarRecorder(
        tmp_path / ".source.rays.partial",
        source_frame="lidar_frame",
        min_sample_period_ns=0,
        metadata={"capture_id": CAPTURE_ID},
    ) as recorder:
        assert recorder.record_observation(
            stamp_ns=1_000_000_000,
            source_frame="lidar_frame",
            origin=(0.0, 0.0, 1.0),
            endpoints=((1.0, 0.0, 1.0),),
        )
        recorder.snapshot(sidecar)
    return write_candidate_map_bundle(
        output_root=tmp_path / "maps",
        map_id="field_alpha",
        revision="r1",
        points=[[0.0, 0.0, 0.1], [1.0, 2.0, 0.3]],
        occupancy_values=[0, 100, -1, 0],
        occupancy_width=2,
        occupancy_height=2,
        occupancy_resolution=0.05,
        occupancy_origin=[-1.0, -2.0, 0.0],
        source_method="unit_test",
        source_details={
            "ray_observations": {
                "capture_id": CAPTURE_ID,
                "source_frame": "lidar_frame",
            },
        },
        created_utc="2026-07-10T00:00:00+00:00",
        ray_observations_path=sidecar,
    )


def _rewrite_sidecar_capture_id(
    manifest: Path, capture_id: object, *, remove: bool = False
) -> None:
    data = _load_manifest(manifest)
    artifact = data["artifacts"]["ray_observations"]
    sidecar = manifest.parent / artifact["path"]
    lines = sidecar.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    if remove:
        header["metadata"].pop("capture_id", None)
    else:
        header["metadata"]["capture_id"] = capture_id
    lines[0] = json.dumps(
        header, allow_nan=False, separators=(",", ":"), sort_keys=True
    )
    sidecar.write_text("\n".join(lines) + "\n", encoding="utf-8")
    artifact["bytes"] = sidecar.stat().st_size
    artifact["sha256"] = _hash(sidecar)
    _save_manifest(manifest, data)


def _rewrite_embedded_sidecar(
    manifest: Path,
    *,
    header_update=None,
    observation_update=None,
) -> None:
    data = _load_manifest(manifest)
    artifact = data["artifacts"]["ray_observations"]
    sidecar = manifest.parent / artifact["path"]
    lines = sidecar.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    if header_update is not None:
        header_update(header)
    lines[0] = json.dumps(
        header, allow_nan=False, separators=(",", ":"), sort_keys=True
    )
    if observation_update is not None:
        observation = json.loads(lines[1])
        observation_update(observation)
        lines[1] = json.dumps(
            observation,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    sidecar.write_text("\n".join(lines) + "\n", encoding="utf-8")
    artifact["bytes"] = sidecar.stat().st_size
    artifact["sha256"] = _hash(sidecar)
    _save_manifest(manifest, data)


def test_validates_synthetic_bundle() -> None:
    result = validate_map_bundle(FIXTURE / "phase2e_test.bundle.yaml")
    assert result["map_id"] == "phase2e_synthetic"
    assert result["frame_id"] == "map"
    assert result["pcd"]["points"] == 4
    assert result["occupancy"]["width"] == 4
    assert result["occupancy"]["height"] == 4
    assert result["map_type"] == "occupancy_with_pcd"


def test_validates_occupancy_only_bundle() -> None:
    result = validate_map_bundle(FIXTURE / "phase2j_occupancy_only.bundle.yaml")
    assert result["schema_version"] == 2
    assert result["map_type"] == "occupancy_only"
    assert result["pcd"] is None
    assert result["occupancy"]["width"] == 4
    assert result["occupancy_origin_reviewed"] is False


def test_validates_phase2j_3d_bundle() -> None:
    result = validate_map_bundle(FIXTURE / "phase2j_3d_test.bundle.yaml")
    assert result["map_type"] == "occupancy_with_pcd"
    assert result["pcd"]["points"] == 522
    assert result["shared_origin_confirmed"] is True


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
    assert result["acceptance_policy"] == "allow_test"


def test_runtime_skips_offline_ray_artifact_but_full_validation_rejects_it(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = _copy_fixture(tmp_path)
    sidecar = manifest.parent / "corrupt.rays.jsonl"
    sidecar.write_bytes(b"not-json\n")
    data = _load_manifest(manifest)
    data["deployment_status"] = "approved"
    data["source"]["details"] = {
        "ray_observations": {
            "capture_id": CAPTURE_ID,
            "source_frame": "lidar_frame",
        },
    }
    data["artifacts"]["ray_observations"] = {
        "path": sidecar.name,
        "schema": "rm_map_ray_observations/v1",
        "frame_id": "map",
        "source_frame": "lidar_frame",
        "stamp_semantics": "source_integer_nanoseconds",
        "status": "complete",
        "capture_id": CAPTURE_ID,
        "bytes": sidecar.stat().st_size,
        "observation_frames": 1,
        "rays": 1,
        "sha256": _hash(sidecar),
        "authority": "offline_evidence_only",
    }
    _save_manifest(manifest, data)

    offline_calls = {"resolve": 0, "hash": 0, "parse": 0}
    original_resolve = map_bundle_module._resolve_artifact
    original_verify_hash = map_bundle_module._verify_hash
    original_parse = map_bundle_module._parse_ray_observations

    def track_resolve(root, relative_path, label, containment_root=None):
        if label == "ray observations":
            offline_calls["resolve"] += 1
        return original_resolve(root, relative_path, label, containment_root)

    def track_verify_hash(path, expected, label):
        if label == "ray observations":
            offline_calls["hash"] += 1
        return original_verify_hash(path, expected, label)

    def track_parse(path):
        offline_calls["parse"] += 1
        return original_parse(path)

    monkeypatch.setattr(map_bundle_module, "_resolve_artifact", track_resolve)
    monkeypatch.setattr(map_bundle_module, "_verify_hash", track_verify_hash)
    monkeypatch.setattr(
        map_bundle_module, "_parse_ray_observations", track_parse
    )

    runtime = resolve_map_bundle_for_runtime(manifest)
    assert runtime["deployment_status"] == "approved"
    assert "ray_observations" not in runtime
    explicitly_skipped = validate_map_bundle(
        manifest,
        require_approved=True,
        validate_offline_artifacts=False,
    )
    assert "ray_observations" not in explicitly_skipped
    assert offline_calls == {"resolve": 0, "hash": 0, "parse": 0}

    with pytest.raises(
        MapBundleError, match="invalid ray observations sidecar"
    ):
        validate_map_bundle(manifest, require_approved=True)
    assert offline_calls == {"resolve": 1, "hash": 1, "parse": 1}


def test_runtime_skip_does_not_require_ray_capture_provenance(
    tmp_path: Path,
) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["deployment_status"] = "approved"
    data["artifacts"]["ray_observations"] = {
        "path": "missing-offline-sidecar.rays.jsonl",
        "capture_id": "not-a-valid-capture-id",
    }
    assert "details" not in data["source"]
    _save_manifest(manifest, data)

    runtime = resolve_map_bundle_for_runtime(manifest)
    assert runtime["deployment_status"] == "approved"
    assert "ray_observations" not in runtime


def test_validated_ray_result_returns_bound_capture_id(tmp_path: Path) -> None:
    manifest = _write_ray_bundle(tmp_path)

    result = validate_map_bundle(manifest)

    assert result["ray_observations"]["capture_id"] == CAPTURE_ID
    assert result["ray_observations"]["source_frame"] == "lidar_frame"
    assert (
        result["ray_observations"]["stamp_semantics"]
        == "source_integer_nanoseconds"
    )


@pytest.mark.parametrize("location", ["artifact", "source", "sidecar"])
def test_rejects_cross_frame_ray_provenance(
    tmp_path: Path,
    location: str,
) -> None:
    manifest = _write_ray_bundle(tmp_path)
    if location == "sidecar":
        _rewrite_embedded_sidecar(
            manifest,
            header_update=lambda header: header.__setitem__(
                "source_frame", "other_lidar_frame"
            ),
        )
    else:
        data = _load_manifest(manifest)
        if location == "artifact":
            data["artifacts"]["ray_observations"][
                "source_frame"
            ] = "other_lidar_frame"
        else:
            data["source"]["details"]["ray_observations"][
                "source_frame"
            ] = "other_lidar_frame"
        _save_manifest(manifest, data)

    with pytest.raises(MapBundleError, match="source_frame mismatch"):
        validate_map_bundle(manifest)


@pytest.mark.parametrize("location", ["artifact", "sidecar"])
def test_rejects_missing_ray_stamp_semantics(
    tmp_path: Path,
    location: str,
) -> None:
    manifest = _write_ray_bundle(tmp_path)
    if location == "sidecar":
        _rewrite_embedded_sidecar(
            manifest,
            header_update=lambda header: header.pop("stamp_semantics"),
        )
    else:
        data = _load_manifest(manifest)
        del data["artifacts"]["ray_observations"]["stamp_semantics"]
        _save_manifest(manifest, data)

    with pytest.raises(MapBundleError, match="stamp_semantics"):
        validate_map_bundle(manifest)


def test_rejects_ray_source_nanosecond_mismatch(tmp_path: Path) -> None:
    manifest = _write_ray_bundle(tmp_path)
    _rewrite_embedded_sidecar(
        manifest,
        observation_update=lambda observation: observation.__setitem__(
            "source_stamp_ns", "1000000001"
        ),
    )

    with pytest.raises(MapBundleError, match="source_stamp_ns"):
        validate_map_bundle(manifest)


@pytest.mark.parametrize("location", ["artifact", "source", "sidecar"])
def test_rejects_cross_session_ray_capture_id(
    tmp_path: Path,
    location: str,
) -> None:
    manifest = _write_ray_bundle(tmp_path)
    if location == "sidecar":
        _rewrite_sidecar_capture_id(manifest, OTHER_CAPTURE_ID)
    else:
        data = _load_manifest(manifest)
        if location == "artifact":
            data["artifacts"]["ray_observations"][
                "capture_id"
            ] = OTHER_CAPTURE_ID
        else:
            data["source"]["details"]["ray_observations"][
                "capture_id"
            ] = OTHER_CAPTURE_ID
        _save_manifest(manifest, data)

    with pytest.raises(MapBundleError, match="capture_id mismatch"):
        validate_map_bundle(manifest)


@pytest.mark.parametrize("location", ["artifact", "source", "sidecar"])
def test_rejects_missing_ray_capture_id(
    tmp_path: Path,
    location: str,
) -> None:
    manifest = _write_ray_bundle(tmp_path)
    if location == "sidecar":
        _rewrite_sidecar_capture_id(manifest, None, remove=True)
    else:
        data = _load_manifest(manifest)
        if location == "artifact":
            del data["artifacts"]["ray_observations"]["capture_id"]
        else:
            del data["source"]["details"]["ray_observations"]["capture_id"]
        _save_manifest(manifest, data)

    with pytest.raises(MapBundleError, match="capture_id"):
        validate_map_bundle(manifest)


@pytest.mark.parametrize(
    ("location", "capture_id"),
    [
        ("artifact", "0" * 31),
        ("artifact", "A" * 32),
        ("source", "g" * 32),
        ("sidecar", "A" * 32),
    ],
)
def test_rejects_malformed_ray_capture_id(
    tmp_path: Path,
    location: str,
    capture_id: str,
) -> None:
    manifest = _write_ray_bundle(tmp_path)
    if location == "sidecar":
        _rewrite_sidecar_capture_id(manifest, capture_id)
    else:
        data = _load_manifest(manifest)
        if location == "artifact":
            data["artifacts"]["ray_observations"]["capture_id"] = capture_id
        else:
            data["source"]["details"]["ray_observations"][
                "capture_id"
            ] = capture_id
        _save_manifest(manifest, data)

    with pytest.raises(
        MapBundleError, match="32 lowercase hexadecimal characters"
    ):
        validate_map_bundle(manifest)


@pytest.mark.parametrize(
    ("artifact_name", "message"),
    [
        ("phase2e_test.pcd", "PCD sha256 mismatch"),
        ("phase2e_test.pgm", "occupancy image sha256 mismatch"),
    ],
)
def test_runtime_offline_skip_keeps_map_artifact_validation_strict(
    tmp_path: Path,
    artifact_name: str,
    message: str,
) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["deployment_status"] = "approved"
    _save_manifest(manifest, data)
    artifact = manifest.parent / artifact_name
    artifact.write_bytes(artifact.read_bytes() + b"tamper")

    with pytest.raises(MapBundleError, match=message):
        resolve_map_bundle_for_runtime(manifest)


def test_candidate_policy_accepts_candidate_and_rejects_test_only(tmp_path: Path) -> None:
    with pytest.raises(MapBundleError, match="not allowed by runtime policy"):
        resolve_map_bundle_for_runtime(
            FIXTURE / "phase2e_test.bundle.yaml",
            acceptance_policy="allow_candidate",
        )

    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["deployment_status"] = "candidate"
    _save_manifest(manifest, data)
    result = resolve_map_bundle_for_runtime(
        manifest,
        acceptance_policy="allow_candidate",
    )
    assert result["deployment_status"] == "candidate"
    assert result["acceptance_policy"] == "allow_candidate"


def test_allow_test_alias_conflicts_with_candidate_policy() -> None:
    with pytest.raises(MapBundleError, match="conflicts"):
        resolve_map_bundle_for_runtime(
            FIXTURE / "phase2e_test.bundle.yaml",
            allow_test_map=True,
            acceptance_policy="allow_candidate",
        )


def test_runtime_resolver_supports_occupancy_only_map() -> None:
    result = resolve_map_bundle_for_runtime(
        FIXTURE / "phase2j_occupancy_only.bundle.yaml",
        allow_test_map=True,
    )
    assert result["map_type"] == "occupancy_only"
    assert result["pcd_path"] is None
    assert result["occupancy_yaml_path"].endswith("phase2e_test.yaml")


def test_occupancy_only_bundle_rejects_pcd_artifact(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["schema_version"] = 2
    data["map_type"] = "occupancy_only"
    data["alignment"] = {"occupancy_origin_reviewed": False}
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="must not contain a PCD"):
        validate_map_bundle(manifest)


def test_approved_occupancy_only_requires_origin_review(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["schema_version"] = 2
    data["map_type"] = "occupancy_only"
    data["deployment_status"] = "approved"
    del data["artifacts"]["pcd"]
    data["alignment"] = {"occupancy_origin_reviewed": False}
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="confirm origin review"):
        validate_map_bundle(manifest, require_approved=True)


def test_approved_occupancy_only_accepts_reviewed_origin(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["schema_version"] = 2
    data["map_type"] = "occupancy_only"
    data["deployment_status"] = "approved"
    del data["artifacts"]["pcd"]
    data["alignment"] = {"occupancy_origin_reviewed": True}
    _save_manifest(manifest, data)
    result = validate_map_bundle(manifest, require_approved=True)
    assert result["pcd"] is None
    assert result["occupancy_origin_reviewed"] is True


def test_approved_bundle_review_is_enforced_under_permissive_policy(
    tmp_path: Path,
) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["deployment_status"] = "approved"
    data["alignment"]["pcd_and_occupancy_share_map_origin"] = False
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="confirm a shared map origin"):
        resolve_map_bundle_for_runtime(
            manifest,
            acceptance_policy="allow_test",
        )


def test_schema_two_requires_explicit_map_type(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    data = _load_manifest(manifest)
    data["schema_version"] = 2
    _save_manifest(manifest, data)
    with pytest.raises(MapBundleError, match="map_type"):
        validate_map_bundle(manifest)


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


def test_rejects_truncated_binary_pgm(tmp_path: Path) -> None:
    manifest = _copy_fixture(tmp_path)
    pgm = manifest.parent / "phase2e_test.pgm"
    data = pgm.read_bytes()
    if data.startswith(b"P5"):
        pgm.write_bytes(data[:-1])
    else:
        pgm.write_bytes(b"P5\n4 4\n255\n" + bytes(15))
    manifest_data = _load_manifest(manifest)
    manifest_data["artifacts"]["occupancy"]["image_sha256"] = _hash(pgm)
    _save_manifest(manifest, manifest_data)
    with pytest.raises(MapBundleError, match="payload size"):
        validate_map_bundle(manifest)
