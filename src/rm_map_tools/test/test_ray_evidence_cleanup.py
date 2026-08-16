from collections.abc import Callable
import hashlib
import json
from pathlib import Path
import stat

import numpy as np
import pytest
import yaml

import rm_map_tools.ray_evidence_cleanup as ray_evidence_cleanup
import rm_map_tools.immutable_output as immutable_output
from rm_map_tools.ray_evidence_cleanup import (
    RayObservation,
    clean_with_ray_evidence,
    load_ray_observations,
    main,
    traverse_voxels,
)
from rm_map_tools.map_export import (
    write_ascii_pcd,
    write_candidate_map_bundle,
)
from rm_map_tools.ray_observations import RaySidecarRecorder


def _write_cli_inputs(tmp_path: Path) -> tuple[Path, Path]:
    source = tmp_path / "source.pcd"
    observations = tmp_path / "rays.jsonl"
    write_ascii_pcd(
        source,
        np.asarray(
            [[1.1, 0.1, 0.1], [0.1, 2.1, 0.1]],
            dtype=np.float32,
        ),
    )
    observations.write_text(
        "\n".join(
            [
                json.dumps(
                    {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
                ),
                *[
                    json.dumps(
                        {
                            "type": "observation",
                            "stamp": float(index + 1),
                            "origin": [0.1, 0.1, 0.1],
                            "endpoints": [
                                [2.1, 0.1, 0.1],
                                [0.1, 2.1, 0.1],
                            ],
                        }
                    )
                    for index in range(4)
                ],
            ]
        ),
        encoding="utf-8",
    )
    return source, observations


def _write_bundle_cli_input(
    tmp_path: Path, *, include_ray_observations: bool = True
) -> Path:
    capture_id = "1" * 32
    ray_sidecar = None
    if include_ray_observations:
        ray_sidecar = tmp_path / "source.rays.jsonl"
        with RaySidecarRecorder(
            tmp_path / ".source.records.partial",
            source_frame="lidar_frame",
            min_sample_period_ns=0,
            metadata={"capture_id": capture_id},
        ) as recorder:
            for index in range(4):
                assert recorder.record_observation(
                    stamp_ns=(index + 1) * 1_000_000_000,
                    source_frame="lidar_frame",
                    origin=(0.1, 0.1, 0.1),
                    endpoints=(
                        (2.1, 0.1, 0.1),
                        (0.1, 2.1, 0.1),
                    ),
                )
            recorder.snapshot(ray_sidecar)

    return write_candidate_map_bundle(
        output_root=tmp_path / "maps",
        map_id="field_alpha",
        revision="r1",
        points=np.asarray(
            [[1.1, 0.1, 0.1], [0.1, 2.1, 0.1]],
            dtype=np.float32,
        ),
        occupancy_values=[0, 100, -1, 0],
        occupancy_width=2,
        occupancy_height=2,
        occupancy_resolution=0.05,
        occupancy_origin=[0.0, 0.0, 0.0],
        source_method="unit_test",
        source_details=(
            {
                "ray_observations": {
                    "capture_id": capture_id,
                    "source_frame": "lidar_frame",
                }
            }
            if include_ray_observations
            else None
        ),
        created_utc="2026-08-16T00:00:00+00:00",
        ray_observations_path=ray_sidecar,
    )


def test_dda_excludes_origin_and_endpoint_voxels() -> None:
    traversed = traverse_voxels((0.1, 0.1, 0.1), (3.1, 0.1, 0.1), 1.0)
    assert traversed == ((1, 0, 0), (2, 0, 0))


def test_sidecar_requires_map_frame_and_monotonic_time(tmp_path: Path) -> None:
    sidecar = tmp_path / "rays.jsonl"
    sidecar.write_text(
        "\n".join(
            [
                json.dumps(
                    {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
                ),
                json.dumps(
                    {
                        "type": "observation",
                        "stamp": 2.0,
                        "origin": [0.0, 0.0, 0.0],
                        "endpoints": [[1.0, 0.0, 0.0]],
                    }
                ),
                json.dumps(
                    {
                        "type": "observation",
                        "stamp": 1.0,
                        "origin": [0.0, 0.0, 0.0],
                        "endpoints": [[1.0, 0.0, 0.0]],
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        load_ray_observations(sidecar)


def test_ray_evidence_removes_transient_and_keeps_static() -> None:
    points = np.asarray(
        [
            [2.1, 1.1, 0.1],  # transient obstacle
            [2.2, 1.1, 0.1],
            [3.1, 0.1, 0.1],  # persistent wall
        ],
        dtype=np.float32,
    )
    observations = []
    for index in range(2):
        observations.append(
            RayObservation(
                1.0 + index,
                (0.1, 0.1, 0.1),
                ((2.1, 1.1, 0.1), (3.1, 0.1, 0.1)),
            )
        )
    for index in range(5):
        observations.append(
            RayObservation(
                3.0 + index,
                (0.1, 1.1, 0.1),
                ((4.1, 1.1, 0.1), (3.1, 0.1, 0.1)),
            )
        )

    result = clean_with_ray_evidence(
        points,
        observations,
        voxel_resolution=1.0,
        min_pass_observations=4,
        pass_to_hit_ratio=2.0,
    )

    assert result.kept_points.shape == (1, 3)
    assert np.allclose(result.kept_points[0], [3.1, 0.1, 0.1])
    assert result.report["removed_points"] == 2
    assert result.report["removed_voxels"] == 1


def test_cli_never_overwrites_source_and_requires_explicit_candidate(
    tmp_path: Path,
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()

    assert main(
        [
            "--input-pcd",
            str(source),
            "--observations",
            str(observations),
            "--report",
            str(report),
            "--voxel-resolution",
            "1.0",
            "--min-pass-observations",
            "4",
        ]
    ) == 0

    assert report.exists()
    assert stat.S_IMODE(report.stat().st_mode) == 0o644
    assert not candidate.exists()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash

    write_report = tmp_path / "write_report.json"
    assert main(
        [
            "--input-pcd",
            str(source),
            "--observations",
            str(observations),
            "--report",
            str(write_report),
            "--output-pcd",
            str(candidate),
            "--write-candidate",
            "--voxel-resolution",
            "1.0",
            "--min-pass-observations",
            "4",
        ]
    ) == 0

    assert candidate.exists()
    assert stat.S_IMODE(candidate.stat().st_mode) == 0o644
    assert stat.S_IMODE(write_report.stat().st_mode) == 0o644
    write_summary = json.loads(write_report.read_text(encoding="utf-8"))
    assert write_summary["candidate_written"]
    assert write_summary["kept_points"] == 1
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash


def test_cli_rejects_report_candidate_path_collision(tmp_path: Path) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    shared_output = tmp_path / "shared-output"
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()

    with pytest.raises(
        ValueError,
        match="report and candidate PCD paths must be distinct",
    ):
        main(
            [
                "--input-pcd",
                str(source),
                "--observations",
                str(observations),
                "--report",
                str(shared_output),
                "--output-pcd",
                str(tmp_path / "." / "shared-output"),
                "--write-candidate",
            ]
        )

    assert not shared_output.exists()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash


@pytest.mark.parametrize("source_role", ["input", "observations"])
@pytest.mark.parametrize("output_role", ["report", "candidate"])
def test_cli_rejects_read_only_output_path_collision(
    tmp_path: Path, source_role: str, output_role: str
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    read_only_path = source if source_role == "input" else observations
    source_bytes = read_only_path.read_bytes()
    report_path = (
        read_only_path if output_role == "report" else tmp_path / "report.json"
    )
    arguments = [
        "--input-pcd",
        str(source),
        "--observations",
        str(observations),
        "--report",
        str(report_path),
    ]
    if output_role == "candidate":
        arguments.extend(
            ["--output-pcd", str(read_only_path), "--write-candidate"]
        )

    with pytest.raises(ValueError, match="paths must be distinct"):
        main(arguments)

    assert read_only_path.read_bytes() == source_bytes
    assert not (tmp_path / "report.json").exists()


def test_cli_exclusive_publish_preserves_path_created_during_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"
    competing_bytes = b"independently-created-map-asset\n"
    original_writer = ray_evidence_cleanup.write_ascii_pcd

    def create_competing_candidate(path: Path, points: np.ndarray) -> None:
        original_writer(path, points)
        candidate.write_bytes(competing_bytes)

    monkeypatch.setattr(
        ray_evidence_cleanup,
        "write_ascii_pcd",
        create_competing_candidate,
    )

    with pytest.raises(
        ValueError,
        match="appeared while writing; refusing overwrite",
    ):
        main(
            [
                "--input-pcd",
                str(source),
                "--observations",
                str(observations),
                "--report",
                str(report),
                "--output-pcd",
                str(candidate),
                "--write-candidate",
            ]
        )

    assert candidate.read_bytes() == competing_bytes
    assert not report.exists()
    assert not tuple(tmp_path.glob(f".{candidate.name}.*.tmp"))


@pytest.mark.parametrize("output_role", ["report", "candidate"])
def test_cli_rejects_dangling_output_symlink(
    tmp_path: Path, output_role: str
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    dangling_target = tmp_path / f"missing-{output_role}"
    output_link = tmp_path / f"{output_role}-link"
    output_link.symlink_to(dangling_target)
    arguments = [
        "--input-pcd",
        str(source),
        "--observations",
        str(observations),
        "--report",
        str(
            output_link
            if output_role == "report"
            else tmp_path / "report.json"
        ),
    ]
    if output_role == "candidate":
        arguments.extend(
            ["--output-pcd", str(output_link), "--write-candidate"]
        )

    with pytest.raises(
        ValueError, match="is a symlink; refusing indirect output"
    ):
        main(arguments)

    assert output_link.is_symlink()
    assert not dangling_target.exists()
    assert not (tmp_path / "report.json").exists()


def test_publish_new_file_fsyncs_file_and_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "published.json"
    sync_targets: list[str] = []
    original_fsync = immutable_output.os.fsync

    def record_fsync(descriptor: int) -> None:
        mode = immutable_output.os.fstat(descriptor).st_mode
        sync_targets.append("directory" if stat.S_ISDIR(mode) else "file")
        original_fsync(descriptor)

    monkeypatch.setattr(immutable_output.os, "fsync", record_fsync)
    ray_evidence_cleanup._publish_new_file(
        destination,
        "report",
        lambda path: path.write_text("complete\n", encoding="utf-8"),
    )

    assert destination.read_text(encoding="utf-8") == "complete\n"
    assert stat.S_IMODE(destination.stat().st_mode) == 0o644
    assert sync_targets == ["file", "directory"]


def test_report_publish_race_keeps_complete_candidate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"
    competing_report = b"independently-created-report\n"
    original_publish = immutable_output.publish_new_file

    def create_competing_report(
        destination: Path,
        label: str,
        writer: Callable[[Path], object],
    ) -> None:
        if label == "report":
            destination.write_bytes(competing_report)
        original_publish(destination, label, writer)

    monkeypatch.setattr(
        immutable_output,
        "publish_new_file",
        create_competing_report,
    )

    with pytest.raises(
        ValueError,
        match="report appeared while writing; refusing overwrite",
    ):
        main(
            [
                "--input-pcd",
                str(source),
                "--observations",
                str(observations),
                "--report",
                str(report),
                "--output-pcd",
                str(candidate),
                "--write-candidate",
            ]
        )

    assert report.read_bytes() == competing_report
    assert candidate.read_bytes().startswith(b"# .PCD v0.7")
    assert stat.S_IMODE(candidate.stat().st_mode) == 0o644
    assert not tuple(tmp_path.glob(".*.tmp"))


def test_cli_rejects_incomplete_recorder_candidate_without_explicit_override(
    tmp_path: Path,
) -> None:
    source, observations = _write_cli_inputs(tmp_path)
    lines = observations.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    header["status"] = "incomplete"
    observations.write_text(
        "\n".join([json.dumps(header), *lines[1:]]) + "\n",
        encoding="utf-8",
    )
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"

    with pytest.raises(ValueError, match="status must be 'complete'"):
        main(
            [
                "--input-pcd",
                str(source),
                "--observations",
                str(observations),
                "--report",
                str(report),
                "--output-pcd",
                str(candidate),
                "--write-candidate",
            ]
        )
    assert not report.exists()
    assert not candidate.exists()

    assert main(
        [
            "--input-pcd",
            str(source),
            "--observations",
            str(observations),
            "--report",
            str(report),
            "--output-pcd",
            str(candidate),
            "--write-candidate",
            "--allow-incomplete-observations",
        ]
    ) == 0
    summary = json.loads(report.read_text(encoding="utf-8"))
    assert summary["observations_status"] == "incomplete"
    assert summary["allow_incomplete_observations"] is True


def test_cli_uses_validated_bundle_bound_inputs(tmp_path: Path) -> None:
    manifest = _write_bundle_cli_input(tmp_path)
    report = tmp_path / "report.json"
    manifest_hash = hashlib.sha256(manifest.read_bytes()).hexdigest()
    manifest_data = yaml.safe_load(manifest.read_text(encoding="utf-8"))

    assert main(
        [
            "--input-manifest",
            str(manifest),
            "--report",
            str(report),
            "--voxel-resolution",
            "1.0",
            "--min-pass-observations",
            "4",
        ]
    ) == 0

    summary = json.loads(report.read_text(encoding="utf-8"))
    assert summary["input_manifest"] == str(manifest.resolve())
    assert summary["input_manifest_sha256"] == manifest_hash
    assert summary["input_pcd"] == str(
        (manifest.parent / manifest_data["artifacts"]["pcd"]["path"]).resolve()
    )
    assert summary["observations"] == str(
        (
            manifest.parent
            / manifest_data["artifacts"]["ray_observations"]["path"]
        ).resolve()
    )
    assert summary["removed_points"] == 1


def test_cli_manifest_rejects_artifact_hash_mismatch_before_outputs(
    tmp_path: Path,
) -> None:
    manifest = _write_bundle_cli_input(tmp_path)
    manifest_data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    pcd = manifest.parent / manifest_data["artifacts"]["pcd"]["path"]
    pcd.write_bytes(pcd.read_bytes() + b"\n")
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"

    with pytest.raises(ValueError, match="PCD sha256 mismatch"):
        main(
            [
                "--input-manifest",
                str(manifest),
                "--report",
                str(report),
                "--output-pcd",
                str(candidate),
                "--write-candidate",
            ]
        )

    assert not report.exists()
    assert not candidate.exists()


@pytest.mark.parametrize(
    ("artifact_key", "changed_label"),
    [
        ("pcd", "input_pcd"),
        ("ray_observations", "observations"),
    ],
)
def test_cli_manifest_rechecks_inputs_after_processing_before_outputs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    artifact_key: str,
    changed_label: str,
) -> None:
    manifest = _write_bundle_cli_input(tmp_path)
    manifest_data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    artifact = (
        manifest.parent / manifest_data["artifacts"][artifact_key]["path"]
    )
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"
    original_cleanup = ray_evidence_cleanup.clean_with_ray_evidence

    def mutate_validated_input_after_processing(*args, **kwargs):
        result = original_cleanup(*args, **kwargs)
        artifact.write_bytes(artifact.read_bytes() + b"concurrent-change\n")
        return result

    monkeypatch.setattr(
        ray_evidence_cleanup,
        "clean_with_ray_evidence",
        mutate_validated_input_after_processing,
    )

    with pytest.raises(
        ValueError,
        match=f"changed after validation: {changed_label}",
    ):
        main(
            [
                "--input-manifest",
                str(manifest),
                "--report",
                str(report),
                "--output-pcd",
                str(candidate),
                "--write-candidate",
            ]
        )

    assert not report.exists()
    assert not candidate.exists()


def test_cli_manifest_requires_ray_observations_artifact(
    tmp_path: Path,
) -> None:
    manifest = _write_bundle_cli_input(
        tmp_path,
        include_ray_observations=False,
    )
    report = tmp_path / "report.json"

    with pytest.raises(ValueError, match="no ray_observations artifact"):
        main(
            [
                "--input-manifest",
                str(manifest),
                "--report",
                str(report),
            ]
        )

    assert not report.exists()


@pytest.mark.parametrize(
    "direct_arguments",
    [
        ["--input-pcd", "source.pcd"],
        ["--observations", "rays.jsonl"],
        [
            "--input-pcd",
            "source.pcd",
            "--observations",
            "rays.jsonl",
        ],
    ],
)
def test_cli_rejects_mixed_manifest_and_direct_inputs_before_outputs(
    tmp_path: Path,
    direct_arguments: list[str],
) -> None:
    manifest = _write_bundle_cli_input(tmp_path)
    report = tmp_path / "report.json"

    with pytest.raises(SystemExit, match="cannot be combined"):
        main(
            [
                "--input-manifest",
                str(manifest),
                *direct_arguments,
                "--report",
                str(report),
            ]
        )

    assert not report.exists()


@pytest.mark.parametrize(
    "input_arguments",
    [
        [],
        ["--input-pcd", "source.pcd"],
        ["--observations", "rays.jsonl"],
    ],
)
def test_cli_rejects_incomplete_input_mode_before_outputs(
    tmp_path: Path,
    input_arguments: list[str],
) -> None:
    report = tmp_path / "report.json"

    with pytest.raises(SystemExit, match="provide|provided together"):
        main([*input_arguments, "--report", str(report)])

    assert not report.exists()
