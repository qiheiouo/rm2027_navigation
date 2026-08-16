import hashlib
from itertools import repeat
import json
import math
import os
from pathlib import Path
import threading

import pytest

from rm_map_tools import immutable_output, ray_observations
from rm_map_tools.ray_observations import (
    RayObservationError,
    RayObservationReadLimits,
    RaySidecarHaltedError,
    RaySidecarLimitError,
    RaySidecarRecorder,
    load_ray_observations,
    prepare_map_ray_observation,
    stream_ray_observations,
)


def _recorder(tmp_path: Path, **kwargs) -> RaySidecarRecorder:
    options = {
        "min_sample_period_ns": 10,
        "max_frames": 4,
        "max_rays_per_frame": 4,
        "max_total_rays": 8,
        "max_bytes": 4096,
        "header_reserve_bytes": 2048,
        "metadata": {"source_topic": "/mapping/sensor_cloud"},
    }
    options.update(kwargs)
    return RaySidecarRecorder(tmp_path / "rays.records.partial", **options)


def _record(
    recorder: RaySidecarRecorder,
    stamp_ns: int,
    *,
    source_frame: str = "livox_frame",
    endpoints=((1.0, 0.0, 0.0),),
) -> bool:
    return recorder.record_observation(
        stamp_ns=stamp_ns,
        source_frame=source_frame,
        origin=(0.0, 0.0, 0.0),
        endpoints=endpoints,
    )


def test_recorder_spool_is_records_only_and_snapshot_is_v1_compatible(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 1_000_000_000)
    assert not _record(recorder, 1_000_000_005)
    assert _record(
        recorder,
        1_000_000_010,
        endpoints=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    )

    spool_first = json.loads(recorder.spool_path.read_text(encoding="utf-8").splitlines()[0])
    assert spool_first["type"] == "observation"
    assert "schema" not in spool_first

    output = tmp_path / "rays.jsonl"
    snapshot = recorder.snapshot(output)
    header, observations = load_ray_observations(output)

    assert snapshot.status == "complete"
    assert snapshot.observation_frames == 2
    assert snapshot.rays == 3
    assert snapshot.bytes == output.stat().st_size
    assert snapshot.sha256 == hashlib.sha256(output.read_bytes()).hexdigest()
    assert header["schema"] == "rm_map_ray_observations/v1"
    assert header["frame_id"] == "map"
    assert header["status"] == "complete"
    assert header["source_frame"] == "livox_frame"
    assert header["stamp_semantics"] == "source_integer_nanoseconds"
    assert header["statistics"]["skipped_min_period_frames"] == 1
    assert len(observations) == 2
    assert [observation.source_stamp_ns for observation in observations] == [
        1_000_000_000,
        1_000_000_010,
    ]
    recorder.close()


@pytest.mark.parametrize("replacement_kind", ("regular", "symlink"))
def test_snapshot_reads_frozen_prefix_from_held_original_inode(
    tmp_path: Path,
    replacement_kind: str,
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 1_000_000_000)
    original_records = recorder.spool_path.read_bytes()
    displaced = tmp_path / "displaced-original.records.partial"
    recorder.spool_path.rename(displaced)

    replacement_bytes = b"replacement path must never become snapshot input\n"
    if replacement_kind == "regular":
        recorder.spool_path.write_bytes(replacement_bytes)
    else:
        replacement = tmp_path / "replacement-target"
        replacement.write_bytes(replacement_bytes)
        recorder.spool_path.symlink_to(replacement)

    output = tmp_path / f"held-inode-{replacement_kind}.jsonl"
    snapshot = recorder.snapshot(output)

    assert output.read_bytes().endswith(original_records)
    assert replacement_bytes not in output.read_bytes()
    assert snapshot.observation_frames == 1
    assert len(load_ray_observations(output)[1]) == 1
    recorder.close()


def test_snapshot_rejects_closed_recorder_without_publishing(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 1_000_000_000)
    recorder.close()
    output = tmp_path / "closed.jsonl"

    with pytest.raises(RaySidecarHaltedError, match="cannot snapshot a closed"):
        recorder.snapshot(output)

    assert not output.exists()


def test_snapshot_freezes_prefix_while_concurrent_record_waits(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 1_000_000_000)
    output = tmp_path / "frozen.jsonl"
    publish_entered = threading.Event()
    allow_publish = threading.Event()
    record_finished = threading.Event()
    snapshot_results = []
    worker_errors = []
    real_publish = ray_observations.publish_new_file

    def paused_publish(*args, **kwargs):
        publish_entered.set()
        if not allow_publish.wait(timeout=2.0):
            raise RuntimeError("timed out waiting to continue snapshot publication")
        return real_publish(*args, **kwargs)

    def take_snapshot() -> None:
        try:
            snapshot_results.append(recorder.snapshot(output))
        except BaseException as exc:  # pragma: no cover - surfaced below
            worker_errors.append(exc)

    def append_observation() -> None:
        try:
            _record(recorder, 1_000_000_010)
        except BaseException as exc:  # pragma: no cover - surfaced below
            worker_errors.append(exc)
        finally:
            record_finished.set()

    monkeypatch.setattr(ray_observations, "publish_new_file", paused_publish)
    snapshot_thread = threading.Thread(target=take_snapshot)
    snapshot_thread.start()
    assert publish_entered.wait(timeout=2.0)
    record_thread = threading.Thread(target=append_observation)
    record_thread.start()
    assert not record_finished.wait(timeout=0.05)

    allow_publish.set()
    snapshot_thread.join(timeout=2.0)
    record_thread.join(timeout=2.0)

    assert not snapshot_thread.is_alive()
    assert not record_thread.is_alive()
    assert worker_errors == []
    assert snapshot_results[0].observation_frames == 1
    assert len(load_ray_observations(output)[1]) == 1
    assert recorder.stats.accepted_frames == 2
    recorder.close()


def test_streaming_reader_enforces_payload_counts_and_limits(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 1_000)
    assert _record(recorder, 1_010)
    output = tmp_path / "rays.jsonl"
    recorder.snapshot(output)

    with stream_ray_observations(output) as (header, observations):
        assert header["status"] == "complete"
        assert not isinstance(observations, list)
        assert [observation.stamp for observation in observations] == [1e-6, 1.01e-6]

    limits = RayObservationReadLimits(
        max_bytes=4096,
        max_header_bytes=2048,
        max_line_bytes=2048,
        max_frames=1,
        max_rays_per_frame=4,
        max_total_rays=4,
    )
    with pytest.raises(RaySidecarLimitError, match="max_frames=1"):
        load_ray_observations(output, limits=limits)
    recorder.close()


def test_streaming_reader_enforces_byte_limit_if_file_grows_after_stat(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "growing.jsonl"
    header = {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
    first = {
        "type": "observation",
        "stamp": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(first) + "\n",
        encoding="utf-8",
    )
    initial_size = sidecar.stat().st_size
    limits = RayObservationReadLimits(
        max_bytes=initial_size + 16,
        max_header_bytes=initial_size,
        max_line_bytes=initial_size,
        max_frames=4,
        max_rays_per_frame=4,
        max_total_rays=8,
    )

    with stream_ray_observations(sidecar, limits=limits) as (_, observations):
        second = {
            "type": "observation",
            "stamp": 2.0,
            "origin": [0.0, 0.0, 0.0],
            "endpoints": [[2.0, 0.0, 0.0]],
        }
        with sidecar.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(second) + "\n")
        with pytest.raises(RaySidecarLimitError, match="max_bytes"):
            list(observations)


def test_legacy_direct_sidecar_without_stamp_semantics_remains_readable(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "legacy.jsonl"
    header = {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
    observation = {
        "type": "observation",
        "stamp": 1.25,
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(observation) + "\n",
        encoding="utf-8",
    )

    loaded_header, observations = load_ray_observations(sidecar)

    assert "stamp_semantics" not in loaded_header
    assert observations[0].source_stamp_ns is None


def test_source_integer_nanoseconds_semantics_requires_matching_decimal_stamp(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "source-ns-mismatch.jsonl"
    header = {
        "schema": "rm_map_ray_observations/v1",
        "frame_id": "map",
        "stamp_semantics": "source_integer_nanoseconds",
    }
    observation = {
        "type": "observation",
        "stamp": 1.0,
        "source_stamp_ns": "1000000001",
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(observation) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(RayObservationError, match="does not match"):
        load_ray_observations(sidecar)


def test_source_integer_nanoseconds_semantics_requires_every_record_ns(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "source-ns-missing.jsonl"
    header = {
        "schema": "rm_map_ray_observations/v1",
        "frame_id": "map",
        "stamp_semantics": "source_integer_nanoseconds",
    }
    observation = {
        "type": "observation",
        "stamp": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(observation) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(RayObservationError, match="source_stamp_ns is required"):
        load_ray_observations(sidecar)


def test_source_stamp_ns_rejects_overlong_decimal_before_integer_conversion(
    tmp_path: Path,
) -> None:
    sidecar = tmp_path / "source-ns-overlong.jsonl"
    header = {
        "schema": "rm_map_ray_observations/v1",
        "frame_id": "map",
        "stamp_semantics": "source_integer_nanoseconds",
    }
    observation = {
        "type": "observation",
        "stamp": 1.0,
        "source_stamp_ns": "1" * 21,
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(observation) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(RayObservationError, match="at most 20 digits"):
        load_ray_observations(sidecar)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("stamp", True),
        ("origin", "123"),
        ("endpoints", ["100"]),
    ],
)
def test_reader_rejects_coercible_non_v1_observation_structures(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    sidecar = tmp_path / f"malformed-{field}.jsonl"
    header = {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
    observation = {
        "type": "observation",
        "stamp": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "endpoints": [[1.0, 0.0, 0.0]],
    }
    observation[field] = value
    sidecar.write_text(
        json.dumps(header) + "\n" + json.dumps(observation) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(RayObservationError, match="invalid timestamp or XYZ"):
        load_ray_observations(sidecar)


def test_non_increasing_time_halts_and_snapshots_as_incomplete(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 100)
    committed = recorder.spool_path.read_bytes()

    with pytest.raises(RayObservationError, match="strictly increasing"):
        _record(recorder, 100)
    assert recorder.status == "halted"
    assert recorder.spool_path.read_bytes() == committed
    with pytest.raises(RaySidecarHaltedError, match="halted"):
        _record(recorder, 110)

    output = tmp_path / "incomplete.jsonl"
    snapshot = recorder.snapshot(output)
    header, observations = load_ray_observations(output)
    assert snapshot.status == "incomplete"
    assert header["status"] == "incomplete"
    assert header["halt_reason"] == "non_increasing_source_time"
    assert len(observations) == 1
    recorder.close()


def test_source_frame_and_frame_limit_are_fatal_without_partial_line(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path, max_frames=1)
    assert _record(recorder, 100, source_frame="left_lidar")
    committed = recorder.spool_path.read_bytes()

    with pytest.raises(RayObservationError, match="source frame changed"):
        _record(recorder, 110, source_frame="right_lidar")
    assert recorder.stats.halt_reason == "source_frame_changed"
    assert recorder.spool_path.read_bytes() == committed
    recorder.close()

    second = RaySidecarRecorder(
        tmp_path / "second.records.partial",
        min_sample_period_ns=0,
        max_frames=1,
        max_rays_per_frame=1,
        max_total_rays=1,
        max_bytes=4096,
        header_reserve_bytes=2048,
    )
    assert _record(second, 100)
    committed = second.spool_path.read_bytes()
    with pytest.raises(RaySidecarLimitError, match="max_frames=1"):
        _record(second, 101)
    assert second.spool_path.read_bytes() == committed
    second.close()


def test_per_frame_total_ray_and_byte_limits_reject_whole_frame(
    tmp_path: Path,
) -> None:
    per_frame = _recorder(tmp_path, max_rays_per_frame=1)
    with pytest.raises(RaySidecarLimitError, match="max_rays_per_frame=1"):
        _record(
            per_frame,
            100,
            endpoints=((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        )
    assert per_frame.spool_path.read_bytes() == b""
    per_frame.close()

    total = RaySidecarRecorder(
        tmp_path / "total.records.partial",
        min_sample_period_ns=0,
        max_frames=3,
        max_rays_per_frame=2,
        max_total_rays=2,
        max_bytes=4096,
        header_reserve_bytes=2048,
    )
    assert _record(total, 100, endpoints=((1.0, 0.0, 0.0),))
    committed = total.spool_path.read_bytes()
    with pytest.raises(RaySidecarLimitError, match="max_total_rays=2"):
        _record(
            total,
            101,
            endpoints=((2.0, 0.0, 0.0), (3.0, 0.0, 0.0)),
        )
    assert total.spool_path.read_bytes() == committed
    total.close()

    byte_limited = RaySidecarRecorder(
        tmp_path / "bytes.records.partial",
        min_sample_period_ns=0,
        max_frames=2,
        max_rays_per_frame=2,
        max_total_rays=4,
        max_bytes=850,
        header_reserve_bytes=800,
    )
    with pytest.raises(RaySidecarLimitError, match="max_bytes=850"):
        _record(byte_limited, 100)
    assert byte_limited.spool_path.read_bytes() == b""
    byte_limited.close()


def test_reset_clears_halt_stats_records_and_source_frame(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 100, source_frame="left_lidar")
    with pytest.raises(RayObservationError):
        _record(recorder, 100, source_frame="left_lidar")

    recorder.reset()
    assert recorder.status == "recording"
    assert recorder.stats.accepted_frames == 0
    assert recorder.stats.source_frame is None
    assert recorder.spool_path.read_bytes() == b""
    assert _record(recorder, 200, source_frame="right_lidar")
    recorder.close()
    with pytest.raises(RaySidecarHaltedError, match="closed"):
        _record(recorder, 210, source_frame="right_lidar")


def test_public_halt_is_validated_idempotent_and_preserves_first_reason(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 100)
    recorder.halt("expected_source_frame_mismatch")
    recorder.halt("later_error")
    assert recorder.status == "halted"
    assert recorder.stats.halt_reason == "expected_source_frame_mismatch"
    with pytest.raises(ValueError, match="non-empty"):
        recorder.halt("  ")
    with pytest.raises(RaySidecarHaltedError, match="halted"):
        _record(recorder, 110)
    recorder.close()
    with pytest.raises(RaySidecarHaltedError, match="closed"):
        recorder.halt("after_close")


def test_byte_limit_accepts_exact_reserved_boundary_and_snapshot_stays_bounded(
    tmp_path: Path,
) -> None:
    probe = RaySidecarRecorder(
        tmp_path / "probe.records.partial",
        min_sample_period_ns=0,
        max_frames=2,
        max_rays_per_frame=1,
        max_total_rays=2,
        max_bytes=4096,
        header_reserve_bytes=800,
    )
    assert _record(probe, 100)
    record_bytes = probe.stats.record_bytes
    probe.close()

    exact = RaySidecarRecorder(
        tmp_path / "exact.records.partial",
        min_sample_period_ns=0,
        max_frames=2,
        max_rays_per_frame=1,
        max_total_rays=2,
        max_bytes=800 + record_bytes,
        header_reserve_bytes=800,
    )
    assert _record(exact, 100)
    output = tmp_path / "exact.jsonl"
    snapshot = exact.snapshot(output)
    assert snapshot.bytes <= 800 + record_bytes
    committed = exact.spool_path.read_bytes()
    with pytest.raises(RaySidecarLimitError, match="max_bytes"):
        _record(exact, 101)
    assert exact.spool_path.read_bytes() == committed
    exact.close()


def test_snapshot_header_reserve_failure_halts_without_final_output(
    tmp_path: Path,
) -> None:
    recorder = RaySidecarRecorder(
        tmp_path / "large-header.records.partial",
        min_sample_period_ns=0,
        max_frames=1,
        max_rays_per_frame=1,
        max_total_rays=1,
        max_bytes=4096,
        header_reserve_bytes=512,
        metadata={"large": "x" * 1000},
    )
    assert _record(recorder, 100)
    output = tmp_path / "must-not-exist.jsonl"
    with pytest.raises(RaySidecarLimitError, match="header_reserve_bytes=512"):
        recorder.snapshot(output)
    assert recorder.status == "halted"
    assert recorder.stats.halt_reason == "header_reserve_exceeded"
    assert not output.exists()
    recorder.close()


def test_snapshot_hash_failure_happens_before_immutable_publication(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 100)
    output = tmp_path / "must-not-publish.jsonl"
    real_sha256 = hashlib.sha256

    class FailingDigest:
        def __init__(self) -> None:
            self._digest = real_sha256()

        def update(self, payload: bytes) -> None:
            self._digest.update(payload)

        def hexdigest(self) -> str:
            raise OSError("injected pre-publication hash failure")

    monkeypatch.setattr(ray_observations.hashlib, "sha256", FailingDigest)
    with pytest.raises(
        RayObservationError, match="cannot publish ray sidecar snapshot"
    ):
        recorder.snapshot(output)

    assert recorder.stats.halt_reason == "snapshot_io_error"
    assert not output.exists()
    assert not tuple(tmp_path.glob(".*.tmp"))
    recorder.close()


def test_snapshot_publish_race_preserves_dangling_competitor(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    recorder = _recorder(tmp_path)
    assert _record(recorder, 100)
    output = tmp_path / "raced.jsonl"
    missing = tmp_path / "competitor-target"
    original_link = immutable_output.os.link

    def link_after_competitor(source, destination) -> None:
        Path(destination).symlink_to(missing)
        original_link(source, destination)

    monkeypatch.setattr(immutable_output.os, "link", link_after_competitor)
    with pytest.raises(ValueError, match="appeared while writing"):
        recorder.snapshot(output)

    assert output.is_symlink()
    assert os.readlink(output) == str(missing)
    assert not tuple(tmp_path.glob(".*.tmp"))
    recorder.close()


def _prepare(points, **kwargs):
    options = {
        "stamp_ns": 123_456_789,
        "translation": (0.0, 0.0, 0.0),
        "min_range": 0.1,
        "max_range": 10.0,
        "voxel_size": 0.1,
        "max_rays_per_frame": 10,
    }
    if "quaternion_xyzw" not in kwargs and "rotation_matrix" not in kwargs:
        options["quaternion_xyzw"] = (0.0, 0.0, 0.0, 1.0)
    options.update(kwargs)
    return prepare_map_ray_observation(points, **options)


def test_prepare_map_rays_uses_sensor_range_and_quaternion_transform() -> None:
    half_sqrt_two = math.sqrt(0.5)
    prepared = _prepare(
        (
            (float("nan"), 0.0, 0.0),
            (0.25, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.01, 0.0, 0.0),
        ),
        translation=(10.0, -2.0, 3.0),
        quaternion_xyzw=(0.0, 0.0, half_sqrt_two, half_sqrt_two),
        min_range=0.5,
        max_range=2.0,
    )

    assert prepared.stamp_ns == 123_456_789
    assert prepared.origin == (10.0, -2.0, 3.0)
    assert prepared.raw_point_count == 5
    assert prepared.finite_point_count == 4
    assert prepared.in_range_point_count == 2
    assert prepared.endpoints[0] == pytest.approx((10.0, -1.0, 3.0))
    assert prepared.endpoints[1] == pytest.approx((10.0, 0.0, 3.0))


def test_prepare_map_rays_deduplicates_nearest_and_orders_negative_voxels(
) -> None:
    points = (
        (1.8, 0.0, 0.0),
        (-0.9, 0.0, 0.0),
        (-1.01, 0.0, 0.0),
        (1.2, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
    )
    forward = _prepare(points, min_range=0.01, voxel_size=1.0)
    reverse = _prepare(reversed(points), min_range=0.01, voxel_size=1.0)

    expected = ((-1.01, 0.0, 0.0), (-0.1, 0.0, 0.0), (1.2, 0.0, 0.0))
    assert forward.endpoints == expected
    assert reverse.endpoints == expected


def test_prepare_map_rays_breaks_equal_range_ties_independent_of_input(
) -> None:
    points = ((1.0, 0.2, 0.0), (1.0, -0.2, 0.0))
    first = _prepare(points, translation=(0.0, 1.0, 0.0), voxel_size=2.0)
    second = _prepare(
        reversed(points),
        translation=(0.0, 1.0, 0.0),
        voxel_size=2.0,
    )

    assert first.endpoints == ((1.0, 0.8, 0.0),)
    assert second.endpoints == first.endpoints


def test_prepare_map_rays_accepts_a_valid_rotation_matrix() -> None:
    prepared = _prepare(
        ((1.0, 0.0, 0.0),),
        translation=(2.0, 3.0, 4.0),
        rotation_matrix=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    )
    assert prepared.endpoints == ((2.0, 4.0, 4.0),)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    (
        ({"quaternion_xyzw": None}, "exactly one"),
        (
            {
                "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
                "rotation_matrix": (
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
            },
            "exactly one",
        ),
    ),
)
def test_prepare_map_rays_requires_one_rotation_representation(
    kwargs, message
) -> None:
    with pytest.raises(ValueError, match=message):
        _prepare(((1.0, 0.0, 0.0),), **kwargs)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    (
        ({"quaternion_xyzw": (0.0, 0.0, 0.0)}, "four finite"),
        ({"quaternion_xyzw": (0.0, 0.0, 0.0, 0.0)}, "non-zero norm"),
        (
            {
                "rotation_matrix": (
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, -1.0),
                )
            },
            "determinant",
        ),
        (
            {
                "rotation_matrix": (
                    (1.0, 0.1, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                )
            },
            "orthonormal",
        ),
        ({"translation": (0.0, float("inf"), 0.0)}, "translation"),
    ),
)
def test_prepare_map_rays_rejects_invalid_rigid_transforms(
    kwargs, message
) -> None:
    with pytest.raises(RayObservationError, match=message):
        _prepare(((1.0, 0.0, 0.0),), **kwargs)


def test_prepare_map_rays_bounds_transform_iterable_validation() -> None:
    with pytest.raises(RayObservationError, match="translation"):
        _prepare(((1.0, 0.0, 0.0),), translation=repeat(0.0))


@pytest.mark.parametrize(
    ("kwargs", "message"),
    (
        ({"stamp_ns": 0}, "stamp_ns"),
        ({"stamp_ns": True}, "stamp_ns"),
        ({"stamp_ns": "1"}, "stamp_ns"),
        ({"min_range": -0.1}, "min_range"),
        ({"min_range": 1.0, "max_range": 1.0}, "min_range"),
        ({"max_range": float("inf")}, "finite"),
        ({"voxel_size": 0.0}, "voxel_size"),
        ({"max_rays_per_frame": 0}, "max_rays_per_frame"),
        ({"max_rays_per_frame": True}, "max_rays_per_frame"),
        ({"max_rays_per_frame": 1.0}, "max_rays_per_frame"),
    ),
)
def test_prepare_map_rays_rejects_invalid_geometry_options(
    kwargs, message
) -> None:
    with pytest.raises(ValueError, match=message):
        _prepare(((1.0, 0.0, 0.0),), **kwargs)


def test_prepare_map_rays_applies_cap_after_voxel_deduplication() -> None:
    accepted = _prepare(
        ((0.1, 0.0, 0.0), (0.2, 0.0, 0.0)),
        min_range=0.01,
        voxel_size=1.0,
        max_rays_per_frame=1,
    )
    assert accepted.endpoints == ((0.1, 0.0, 0.0),)

    with pytest.raises(RaySidecarLimitError, match="max_rays_per_frame=1"):
        _prepare(
            ((0.1, 0.0, 0.0), (1.1, 0.0, 0.0)),
            min_range=0.01,
            voxel_size=1.0,
            max_rays_per_frame=1,
        )
