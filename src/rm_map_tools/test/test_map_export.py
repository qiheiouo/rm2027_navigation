from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from rm_map_tools import immutable_output, map_export
from rm_map_tools.map_bundle import (
    MapBundleError,
    resolve_map_bundle_for_runtime,
    validate_map_bundle,
)
from rm_map_tools.map_export import (
    occupancy_values_to_pgm,
    write_candidate_map_bundle,
)
from rm_map_tools.ray_observations import RaySidecarLimitError, RaySidecarRecorder


_CAPTURE_ID = "0123456789abcdef0123456789abcdef"


def _bundle_arguments(tmp_path: Path, revision: str = "r1") -> dict[str, object]:
    return {
        "output_root": tmp_path / "maps",
        "map_id": "field_alpha",
        "revision": revision,
        "points": np.asarray(
            [[0.0, 0.0, 0.1], [1.0, 2.0, 0.3]], dtype=np.float32
        ),
        "occupancy_values": [0, 100, -1, 0],
        "occupancy_width": 2,
        "occupancy_height": 2,
        "occupancy_resolution": 0.05,
        "occupancy_origin": [-1.0, -2.0, 0.0],
        "source_method": "unit_test",
        "source_details": {
            "ray_observations": {
                "capture_id": _CAPTURE_ID,
                "source_frame": "lidar_frame",
            },
        },
        "created_utc": "2026-07-10T00:00:00+00:00",
    }


def _complete_ray_sidecar(tmp_path: Path, name: str = "evidence") -> Path:
    sidecar = tmp_path / f"{name}.jsonl"
    with RaySidecarRecorder(
        tmp_path / f".{name}.records.partial",
        source_frame="lidar_frame",
        min_sample_period_ns=0,
        metadata={"capture_id": _CAPTURE_ID},
    ) as recorder:
        assert recorder.record_observation(
            stamp_ns=1_000_000_000,
            source_frame="lidar_frame",
            origin=(0.0, 0.0, 1.0),
            endpoints=((1.0, 0.0, 1.0), (2.0, 0.0, 1.0)),
        )
        assert recorder.record_observation(
            stamp_ns=2_000_000_000,
            source_frame="lidar_frame",
            origin=(0.5, 0.0, 1.0),
            endpoints=((2.5, 0.0, 1.0),),
        )
        recorder.snapshot(sidecar)
    return sidecar


def _rewrite_sidecar_json(
    sidecar: Path,
    *,
    header_update=None,
    observation_update=None,
) -> None:
    lines = sidecar.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    if header_update is not None:
        header_update(header)
    lines[0] = json.dumps(header, separators=(",", ":"), sort_keys=True)
    if observation_update is not None:
        observation = json.loads(lines[1])
        observation_update(observation)
        lines[1] = json.dumps(
            observation, separators=(",", ":"), sort_keys=True
        )
    sidecar.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
    assert "ray_observations" not in result
    assert "ray_observations" not in yaml.safe_load(
        manifest.read_text(encoding="utf-8")
    )["artifacts"]
    assert sorted(path.name for path in manifest.parent.iterdir()) == [
        "field_alpha.bundle.yaml",
        "field_alpha.pcd",
        "field_alpha.pgm",
        "field_alpha.yaml",
    ]

    occupancy_yaml = yaml.safe_load(
        (manifest.parent / "field_alpha.yaml").read_text(encoding="utf-8")
    )
    unknown_probability = (255 - 205) / 255.0
    assert occupancy_yaml["free_thresh"] < unknown_probability
    assert unknown_probability < occupancy_yaml["occupied_thresh"]


def test_explicit_none_ray_sidecar_is_byte_compatible_with_default(
    tmp_path: Path,
) -> None:
    default_arguments = _bundle_arguments(tmp_path / "default")
    explicit_arguments = _bundle_arguments(tmp_path / "explicit")
    default_manifest = write_candidate_map_bundle(**default_arguments)
    explicit_manifest = write_candidate_map_bundle(
        **explicit_arguments,
        ray_observations_path=None,
    )

    default_files = {
        path.name: path.read_bytes() for path in default_manifest.parent.iterdir()
    }
    explicit_files = {
        path.name: path.read_bytes() for path in explicit_manifest.parent.iterdir()
    }
    assert explicit_files == default_files


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


@pytest.mark.parametrize("existing_kind", ["empty_directory", "file", "dangling"])
def test_candidate_export_rejects_every_existing_target_entry(
    tmp_path: Path, existing_kind: str
) -> None:
    arguments = _bundle_arguments(tmp_path)
    target = tmp_path / "maps" / "field_alpha" / "r1"
    target.parent.mkdir(parents=True)
    if existing_kind == "empty_directory":
        target.mkdir()
    elif existing_kind == "file":
        target.write_bytes(b"competitor")
    else:
        target.symlink_to(tmp_path / "missing-target", target_is_directory=True)

    with pytest.raises(MapBundleError, match="already exists"):
        write_candidate_map_bundle(**arguments)

    if existing_kind == "empty_directory":
        assert target.is_dir() and not tuple(target.iterdir())
    elif existing_kind == "file":
        assert target.read_bytes() == b"competitor"
    else:
        assert target.is_symlink()


@pytest.mark.parametrize(
    "competitor_kind", ["empty_directory", "file", "dangling"]
)
def test_candidate_export_publish_race_never_replaces_target(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    competitor_kind: str,
) -> None:
    target = tmp_path / "maps" / "field_alpha" / "r1"
    original_publish = immutable_output.publish_new_directory

    def publish_after_competitor(
        source: Path, destination: Path, label: str
    ) -> None:
        assert destination == target
        if competitor_kind == "empty_directory":
            destination.mkdir()
        elif competitor_kind == "file":
            destination.write_bytes(b"concurrent competitor")
        else:
            destination.symlink_to(
                tmp_path / "concurrent-missing", target_is_directory=True
            )
        original_publish(source, destination, label)

    monkeypatch.setattr(map_export, "publish_new_directory", publish_after_competitor)
    with pytest.raises(MapBundleError, match="appeared while writing"):
        write_candidate_map_bundle(**_bundle_arguments(tmp_path))

    if competitor_kind == "empty_directory":
        assert target.is_dir() and not tuple(target.iterdir())
    elif competitor_kind == "file":
        assert target.read_bytes() == b"concurrent competitor"
    else:
        assert target.is_symlink()
    assert not tuple(target.parent.glob(".r1-*"))


def test_candidate_export_fsyncs_all_files_and_directories_before_success(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source_sidecar = _complete_ray_sidecar(tmp_path)
    synchronized_files: list[str] = []
    synchronized_directories: list[Path] = []

    monkeypatch.setattr(
        map_export,
        "fsync_file",
        lambda path: synchronized_files.append(path.name),
    )
    monkeypatch.setattr(
        map_export,
        "fsync_directory",
        lambda path: synchronized_directories.append(path),
    )
    original_directory_sync = immutable_output.fsync_directory

    def record_parent_sync(path: Path) -> None:
        synchronized_directories.append(path)
        original_directory_sync(path)

    monkeypatch.setattr(immutable_output, "fsync_directory", record_parent_sync)
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=source_sidecar,
    )

    assert synchronized_files == [
        "field_alpha.pcd",
        "field_alpha.pgm",
        "field_alpha.yaml",
        "field_alpha.bundle.yaml",
        "field_alpha.rays.jsonl",
    ]
    assert len(synchronized_directories) == 2
    assert synchronized_directories[0].name.startswith(".r1-")
    assert synchronized_directories[1] == manifest.parent.parent


def test_candidate_export_file_sync_failure_does_not_publish(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    synchronized = 0

    def fail_manifest_sync(path: Path) -> None:
        nonlocal synchronized
        synchronized += 1
        if path.name.endswith(".bundle.yaml"):
            raise OSError("injected artifact fsync failure")

    monkeypatch.setattr(map_export, "fsync_file", fail_manifest_sync)
    with pytest.raises(OSError, match="injected artifact fsync failure"):
        write_candidate_map_bundle(**_bundle_arguments(tmp_path))

    target_parent = tmp_path / "maps" / "field_alpha"
    assert synchronized == 4
    assert not (target_parent / "r1").exists()
    assert not tuple(target_parent.glob(".r1-*"))


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


def test_candidate_export_embeds_complete_ray_sidecar(tmp_path: Path) -> None:
    source_sidecar = _complete_ray_sidecar(tmp_path)
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=source_sidecar,
    )

    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    artifact = data["artifacts"]["ray_observations"]
    embedded = manifest.parent / "field_alpha.rays.jsonl"
    assert embedded.read_bytes() == source_sidecar.read_bytes()
    assert artifact == {
        "path": "field_alpha.rays.jsonl",
        "schema": "rm_map_ray_observations/v1",
        "frame_id": "map",
        "source_frame": "lidar_frame",
        "stamp_semantics": "source_integer_nanoseconds",
        "status": "complete",
        "capture_id": _CAPTURE_ID,
        "bytes": embedded.stat().st_size,
        "observation_frames": 2,
        "rays": 3,
        "sha256": hashlib.sha256(embedded.read_bytes()).hexdigest(),
        "authority": "offline_evidence_only",
    }

    result = validate_map_bundle(manifest)
    assert result["ray_observations"] == {
        **artifact,
        "path": str(embedded.resolve()),
    }
    runtime = resolve_map_bundle_for_runtime(
        manifest, acceptance_policy="allow_candidate"
    )
    assert "ray_observations" not in runtime


def test_candidate_export_rejects_missing_ray_capture_id(tmp_path: Path) -> None:
    sidecar = tmp_path / "missing-capture.jsonl"
    with RaySidecarRecorder(
        tmp_path / ".missing-capture.records.partial",
        source_frame="lidar_frame",
        min_sample_period_ns=0,
    ) as recorder:
        assert recorder.record_observation(
            stamp_ns=1_000_000_000,
            source_frame="lidar_frame",
            origin=(0.0, 0.0, 0.0),
            endpoints=((1.0, 0.0, 0.0),),
        )
        recorder.snapshot(sidecar)

    with pytest.raises(MapBundleError, match="metadata.capture_id"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=sidecar,
        )


def test_candidate_export_rejects_cross_session_ray_sidecar(
    tmp_path: Path,
) -> None:
    arguments = _bundle_arguments(tmp_path)
    arguments["source_details"] = {
        "ray_observations": {
            "capture_id": "f" * 32,
            "source_frame": "lidar_frame",
        }
    }

    with pytest.raises(MapBundleError, match="capture_id must match"):
        write_candidate_map_bundle(
            **arguments,
            ray_observations_path=_complete_ray_sidecar(tmp_path),
        )


def test_candidate_export_rejects_missing_ray_stamp_semantics(
    tmp_path: Path,
) -> None:
    sidecar = _complete_ray_sidecar(tmp_path)
    _rewrite_sidecar_json(
        sidecar,
        header_update=lambda header: header.pop("stamp_semantics"),
    )

    with pytest.raises(MapBundleError, match="header stamp_semantics"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=sidecar,
        )


def test_candidate_export_rejects_source_nanosecond_mismatch(
    tmp_path: Path,
) -> None:
    sidecar = _complete_ray_sidecar(tmp_path)
    _rewrite_sidecar_json(
        sidecar,
        observation_update=lambda observation: observation.__setitem__(
            "source_stamp_ns", "1000000001"
        ),
    )

    with pytest.raises(MapBundleError, match="source_stamp_ns"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=sidecar,
        )


@pytest.mark.parametrize(
    "source_frame",
    ["map", "odom", "base_link", "base_footprint", "lio_imu_link"],
)
def test_candidate_export_rejects_non_sensor_source_frame(
    tmp_path: Path,
    source_frame: str,
) -> None:
    sidecar = _complete_ray_sidecar(tmp_path)
    _rewrite_sidecar_json(
        sidecar,
        header_update=lambda header: header.__setitem__(
            "source_frame", source_frame
        ),
    )

    with pytest.raises(MapBundleError, match="physical lidar frame"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=sidecar,
        )


def test_candidate_export_keeps_structurally_valid_incomplete_ray_sidecar(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "incomplete.jsonl"
    with RaySidecarRecorder(
        tmp_path / ".incomplete.records.partial",
        source_frame="lidar_frame",
        min_sample_period_ns=0,
        max_frames=1,
        metadata={"capture_id": _CAPTURE_ID},
    ) as recorder:
        recorder.record_observation(
            stamp_ns=1_000_000_000,
            source_frame="lidar_frame",
            origin=(0.0, 0.0, 0.0),
            endpoints=((1.0, 0.0, 0.0),),
        )
        with pytest.raises(RaySidecarLimitError):
            recorder.record_observation(
                stamp_ns=2_000_000_000,
                source_frame="lidar_frame",
                origin=(0.0, 0.0, 0.0),
                endpoints=((2.0, 0.0, 0.0),),
            )
        recorder.snapshot(sidecar)

    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=sidecar,
    )
    result = validate_map_bundle(manifest)
    assert result["ray_observations"]["status"] == "incomplete"
    assert yaml.safe_load(manifest.read_text(encoding="utf-8"))["artifacts"][
        "ray_observations"
    ]["status"] == "incomplete"


def test_ray_sidecar_manifest_path_cannot_escape_bundle(tmp_path: Path) -> None:
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=_complete_ray_sidecar(tmp_path),
    )
    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    data["artifacts"]["ray_observations"]["path"] = "../outside.jsonl"
    manifest.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    with pytest.raises(MapBundleError, match="escapes the map bundle"):
        validate_map_bundle(manifest)


def test_ray_sidecar_hash_tamper_is_rejected(tmp_path: Path) -> None:
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=_complete_ray_sidecar(tmp_path),
    )
    sidecar = manifest.parent / "field_alpha.rays.jsonl"
    sidecar.write_bytes(sidecar.read_bytes() + b"\n")

    with pytest.raises(MapBundleError, match="sha256 mismatch"):
        validate_map_bundle(manifest)


def test_ray_sidecar_streaming_payload_statistics_are_validated(
    tmp_path: Path,
) -> None:
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=_complete_ray_sidecar(tmp_path),
    )
    sidecar = manifest.parent / "field_alpha.rays.jsonl"
    lines = sidecar.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    header["statistics"]["accepted_frames"] += 1
    lines[0] = json.dumps(header, separators=(",", ":"), sort_keys=True)
    sidecar.write_text("\n".join(lines) + "\n", encoding="utf-8")

    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    artifact = data["artifacts"]["ray_observations"]
    artifact["sha256"] = hashlib.sha256(sidecar.read_bytes()).hexdigest()
    artifact["bytes"] = sidecar.stat().st_size
    manifest.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    with pytest.raises(MapBundleError, match="accepted_frames does not match"):
        validate_map_bundle(manifest)


def test_malformed_ray_sidecar_does_not_publish_candidate(tmp_path: Path) -> None:
    malformed = tmp_path / "malformed.jsonl"
    malformed.write_text(
        json.dumps(
            {
                "schema": "rm_map_ray_observations/v1",
                "frame_id": "map",
                "source_frame": "lidar_frame",
                "stamp_semantics": "source_integer_nanoseconds",
                "status": "complete",
                "statistics": {"accepted_frames": 1, "total_rays": 1},
                "metadata": {"capture_id": _CAPTURE_ID},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(MapBundleError, match="contains no observations"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=malformed,
        )
    target_parent = tmp_path / "maps" / "field_alpha"
    assert target_parent.is_dir()
    assert not list(target_parent.iterdir())


def test_ray_sidecar_change_during_copy_does_not_publish_candidate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _complete_ray_sidecar(tmp_path)
    original_copy = map_export.shutil.copyfile

    def mutate_then_copy(source_path, destination_path):
        lines = Path(source_path).read_text(encoding="utf-8").splitlines()
        observation = json.loads(lines[1])
        observation["endpoints"][0][0] += 0.25
        lines[1] = json.dumps(observation, separators=(",", ":"), sort_keys=True)
        Path(source_path).write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        return original_copy(source_path, destination_path)

    monkeypatch.setattr(map_export.shutil, "copyfile", mutate_then_copy)
    with pytest.raises(MapBundleError, match="changed while being copied"):
        write_candidate_map_bundle(
            **_bundle_arguments(tmp_path),
            ray_observations_path=source,
        )

    target_parent = tmp_path / "maps" / "field_alpha"
    assert target_parent.is_dir()
    assert not list(target_parent.iterdir())


@pytest.mark.parametrize(
    ("field", "increment", "message"),
    [
        ("bytes", 1, "byte count"),
        ("observation_frames", 1, "frame count"),
        ("rays", 1, "ray count"),
    ],
)
def test_ray_sidecar_manifest_statistics_must_match_payload(
    tmp_path: Path,
    field: str,
    increment: int,
    message: str,
) -> None:
    manifest = write_candidate_map_bundle(
        **_bundle_arguments(tmp_path),
        ray_observations_path=_complete_ray_sidecar(tmp_path),
    )
    data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    data["artifacts"]["ray_observations"][field] += increment
    manifest.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    with pytest.raises(MapBundleError, match=message):
        validate_map_bundle(manifest)
