import json
import threading
import uuid
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformException

import rm_map_tools.mapping_session_node as mapping_session_node
from rm_map_tools.mapping_session_node import (
    MappingSessionNode,
    _prepare_ray_session_root,
    _validate_ray_input_contract,
    _validate_ray_recorder_limits,
)
from rm_map_tools.pointcloud_sampler_node import PointCloudSamplerNode
from rm_map_tools.ray_observations import RaySidecarRecorder


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _CaptureLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def warning(self, message, **kwargs):
        del kwargs
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class _StaticTransformBuffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.calls.append((target_frame, source_frame, stamp, timeout))
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


class _ManualFuture:
    def __init__(self):
        self._callbacks = []
        self._exception = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        if self._exception is not None:
            raise self._exception
        return None

    def succeed(self):
        for callback in tuple(self._callbacks):
            callback(self)

    def fail(self, exception):
        self._exception = exception
        for callback in tuple(self._callbacks):
            callback(self)


class _ReadyResetClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = _ManualFuture()
        self.futures.append(future)
        return future


class _NotReadyResetClient(_ReadyResetClient):
    def service_is_ready(self):
        return False


class _RaisingResetClient(_ReadyResetClient):
    def call_async(self, request):
        self.requests.append(request)
        raise RuntimeError("cannot enqueue reset")


class _CaptureRayRecorder:
    def __init__(self):
        self.recorded = []
        self.halt_reasons = []
        self.reset_count = 0
        self.status = "recording"

    def record_observation(self, **kwargs):
        self.recorded.append(kwargs)
        return True

    def halt(self, reason):
        self.halt_reasons.append(reason)
        self.status = "halted"

    def reset(self):
        self.reset_count += 1
        self.status = "recording"


def _ray_session_without_ros_runtime(recorder=None):
    session = object.__new__(MappingSessionNode)
    session._lock = threading.Lock()
    session._ray_lock = threading.Lock()
    session._recording = True
    session._recording_publisher = _CapturePublisher()
    session._ray_enabled = True
    session._ray_contract_error = None
    session._ray_recorder = recorder or _CaptureRayRecorder()
    session._ray_source_frame = "mid360_left_frame"
    session._ray_min_range = 0.30
    session._ray_max_range = 12.0
    session._ray_voxel_size = 0.10
    session._ray_max_rays_per_frame = 10000
    session._ray_dropped_tf = 0
    session._ray_dropped_decode = 0
    session._ray_dropped_empty = 0
    session._ray_capture_id = "0" * 32
    session._ray_last_exported_complete_prefix = None
    session._ray_allow_degraded_save = False
    session._reset_pending = False
    session.get_parameter = lambda name: SimpleNamespace(
        value=session._ray_allow_degraded_save
    )
    session._map_frame = "map"
    session._tf_timeout = 0.0
    session._tf_buffer = _StaticTransformBuffer()
    return session


def _sampler_without_ros_runtime():
    sampler = object.__new__(PointCloudSamplerNode)
    sampler._lock = threading.Lock()
    sampler._latest = None
    sampler._received_sequence = 0
    sampler._published_sequence = 0
    sampler._enabled = False
    sampler._publisher = _CapturePublisher()
    return sampler


def test_sampler_pause_resume_drops_paused_and_stale_clouds():
    sampler = _sampler_without_ros_runtime()

    before_start = PointCloud2()
    sampler._cloud_callback(before_start)
    sampler._publish_latest()
    assert sampler._publisher.messages == []

    sampler._status_callback(Bool(data=True))
    sampler._publish_latest()
    assert sampler._publisher.messages == []

    active = PointCloud2()
    sampler._cloud_callback(active)
    sampler._publish_latest()
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    sampler._status_callback(Bool(data=False))
    received_while_paused = PointCloud2()
    sampler._cloud_callback(received_while_paused)
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    sampler._status_callback(Bool(data=True))
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    received_after_resume = PointCloud2()
    sampler._cloud_callback(received_after_resume)
    sampler._publish_latest()
    assert sampler._publisher.messages == [active, received_after_resume]


@pytest.mark.parametrize(
    "ray_topic,registered_topic,source_frame",
    [
        ("/lio/cloud_registered", "/lio/cloud_registered", "mid360_left_frame"),
        ("lio/cloud_registered", "/lio/cloud_registered", "mid360_left_frame"),
        ("/mapping/sensor_cloud", "/lio/cloud_registered", "map"),
        ("/mapping/sensor_cloud", "/lio/cloud_registered", "odom"),
        ("/mapping/sensor_cloud", "/lio/cloud_registered", "base_link"),
        ("/mapping/sensor_cloud", "/lio/cloud_registered", "base_footprint"),
        ("/mapping/sensor_cloud", "/lio/cloud_registered", "lio_imu_link"),
    ],
)
def test_ray_input_contract_rejects_registered_cloud_and_non_lidar_frames(
    ray_topic, registered_topic, source_frame
):
    with pytest.raises(ValueError):
        _validate_ray_input_contract(
            enabled=True,
            ray_cloud_topic=ray_topic,
            registered_cloud_topic=registered_topic,
            ray_source_frame=source_frame,
        )


def test_default_off_does_not_apply_ray_input_contract():
    _validate_ray_input_contract(
        enabled=False,
        ray_cloud_topic="/lio/cloud_registered",
        registered_cloud_topic="/lio/cloud_registered",
        ray_source_frame="map",
    )


@pytest.mark.parametrize(
    "field,consumer_limit",
    [
        ("max_frames", 10_000),
        ("max_rays_per_frame", 10_000),
        ("max_total_rays", 10_000_000),
        ("max_bytes", 512 * 1024 * 1024),
    ],
)
def test_ray_recorder_limits_cannot_exceed_default_consumer_limits(
    field, consumer_limit
):
    configured = {
        "max_frames": 10_000,
        "max_rays_per_frame": 10_000,
        "max_total_rays": 10_000_000,
        "max_bytes": 512 * 1024 * 1024,
    }
    configured[field] = consumer_limit + 1
    with pytest.raises(ValueError, match=field):
        _validate_ray_recorder_limits(enabled=True, **configured)


def test_default_off_does_not_apply_ray_consumer_limit_contract():
    _validate_ray_recorder_limits(
        enabled=False,
        max_frames=10_001,
        max_rays_per_frame=10_001,
        max_total_rays=10_000_001,
        max_bytes=512 * 1024 * 1024 + 1,
    )


def test_prepare_ray_session_root_refuses_child_symlink(tmp_path):
    output_root = tmp_path / "maps"
    output_root.mkdir()
    redirected = tmp_path / "redirected"
    redirected.mkdir()
    (output_root / ".ray_sidecar_sessions").symlink_to(
        redirected, target_is_directory=True
    )

    with pytest.raises(ValueError, match="must not be a symlink"):
        _prepare_ray_session_root(output_root)

    assert not tuple(redirected.iterdir())


def test_prepare_ray_session_root_creates_canonical_private_directory(tmp_path):
    output_root = tmp_path / "maps"

    session_root = _prepare_ray_session_root(output_root)

    assert session_root == (output_root / ".ray_sidecar_sessions").resolve()
    assert session_root.is_dir()


def test_mapping_services_publish_recording_state():
    session = object.__new__(MappingSessionNode)
    session._lock = threading.Lock()
    session._recording = True
    session._recording_publisher = _CapturePublisher()

    stop_response = session._stop(Trigger.Request(), Trigger.Response())
    assert stop_response.success
    assert stop_response.message == "mapping recording paused"
    assert session._recording is False
    assert session._recording_publisher.messages[-1].data is False

    start_response = session._start(Trigger.Request(), Trigger.Response())
    assert start_response.success
    assert start_response.message == "mapping recording enabled"
    assert session._recording is True
    assert session._recording_publisher.messages[-1].data is True


def test_mapping_reset_waits_for_transport_success_before_clearing_session(
    monkeypatch,
):
    session = object.__new__(MappingSessionNode)
    session._lock = threading.Lock()
    session._recording = True
    session._recording_publisher = _CapturePublisher()
    session._octomap_reset_client = _ReadyResetClient()
    session._voxels = {(1, 2, 3): (np.asarray([1.0, 2.0, 3.0]), 2)}
    session._latest_occupancy = object()
    session._last_sample_ns = 100
    session._accepted_clouds = 3
    session._dropped_tf = 2
    session._latest_tf_fallbacks = 1
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._reset(Trigger.Request(), Trigger.Response())

    assert response.success
    assert response.message == (
        "OctoMap reset requested; local mapping session remains paused and "
        "will clear after transport success"
    )
    assert session._recording is False
    assert [message.data for message in session._recording_publisher.messages] == [False]
    assert session._reset_pending is True
    assert session._voxels
    assert session._latest_occupancy is not None

    session._octomap_reset_client.futures[0].succeed()

    assert session._voxels == {}
    assert session._latest_occupancy is None
    assert session._last_sample_ns == -1
    assert session._accepted_clouds == 0
    assert session._dropped_tf == 0
    assert session._latest_tf_fallbacks == 0
    assert session._reset_pending is False
    assert len(session._octomap_reset_client.requests) == 1


def test_voxel_limit_auto_pause_publishes_false_without_accumulating(
    monkeypatch,
):
    session = object.__new__(MappingSessionNode)
    session._lock = threading.Lock()
    session._recording = True
    session._recording_publisher = _CapturePublisher()
    session._map_frame = "map"
    session._last_sample_ns = -1
    session._sample_period = 0.0
    session._tf_buffer = _StaticTransformBuffer()
    session._tf_timeout = 0.0
    session._allow_latest_tf_fallback = False
    session._dropped_tf = 0
    session._latest_tf_fallbacks = 0
    session._min_z = -1.0
    session._max_z = 3.0
    session._voxel_size = 1.0
    session._max_voxels = 1
    existing_point = np.asarray([0.1, 0.1, 0.1], dtype=np.float32)
    session._voxels = {(0, 0, 0): (existing_point, 1)}
    session._accepted_clouds = 1
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    monkeypatch.setattr(
        mapping_session_node.point_cloud2,
        "read_points_numpy",
        lambda *args, **kwargs: np.asarray([[2.1, 0.1, 0.1]], dtype=np.float32),
    )
    cloud = PointCloud2()
    cloud.header.frame_id = "mid360_left_frame"
    cloud.header.stamp.sec = 1

    session._cloud_callback(cloud)

    assert session._recording is False
    assert [message.data for message in session._recording_publisher.messages] == [False]
    assert tuple(session._voxels) == ((0, 0, 0),)
    assert session._accepted_clouds == 1
    assert session._last_sample_ns == -1
    assert logger.errors == [
        "mapping stopped before exceeding max_voxels=1; save or reset the session"
    ]


def test_ray_callback_uses_only_timestamped_map_sensor_transform(monkeypatch):
    session = _ray_session_without_ros_runtime()
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    monkeypatch.setattr(
        mapping_session_node.point_cloud2,
        "read_points_numpy",
        lambda *args, **kwargs: np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
    )
    cloud = PointCloud2()
    cloud.header.frame_id = "mid360_left_frame"
    cloud.header.stamp.sec = 4
    cloud.header.stamp.nanosec = 7

    session._ray_cloud_callback(cloud)

    assert len(session._tf_buffer.calls) == 1
    target, source, stamp, _ = session._tf_buffer.calls[0]
    assert (target, source) == ("map", "mid360_left_frame")
    assert stamp.nanoseconds == 4_000_000_007
    assert len(session._ray_recorder.recorded) == 1
    recorded = session._ray_recorder.recorded[0]
    assert recorded["stamp_ns"] == 4_000_000_007
    assert recorded["origin"] == (0.0, 0.0, 0.0)
    assert recorded["endpoints"] == ((1.0, 0.0, 0.0),)
    assert session._recording is True
    assert session._recording_publisher.messages == []


def test_ray_timestamped_tf_drop_is_nonfatal(monkeypatch):
    class _MissingTransformBuffer:
        def __init__(self):
            self.calls = 0

        def lookup_transform(self, *args, **kwargs):
            del args, kwargs
            self.calls += 1
            raise TransformException("missing timestamped transform")

    session = _ray_session_without_ros_runtime()
    session._tf_buffer = _MissingTransformBuffer()
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    cloud = PointCloud2()
    cloud.header.frame_id = "mid360_left_frame"
    cloud.header.stamp.sec = 1

    session._ray_cloud_callback(cloud)

    assert session._tf_buffer.calls == 1
    assert session._ray_dropped_tf == 1
    assert session._ray_recorder.recorded == []
    assert session._ray_recorder.halt_reasons == []
    assert session._recording is True
    assert session._recording_publisher.messages == []
    assert "timestamped map <- mid360_left_frame" in logger.warnings[0]


def test_ray_decode_drop_is_nonfatal(monkeypatch):
    session = _ray_session_without_ros_runtime()
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    def fail_decode(*args, **kwargs):
        del args, kwargs
        raise ValueError("bad fields")

    monkeypatch.setattr(
        mapping_session_node.point_cloud2,
        "read_points_numpy",
        fail_decode,
    )
    cloud = PointCloud2()
    cloud.header.frame_id = "mid360_left_frame"
    cloud.header.stamp.sec = 1

    session._ray_cloud_callback(cloud)

    assert session._ray_dropped_decode == 1
    assert session._ray_recorder.recorded == []
    assert session._ray_recorder.halt_reasons == []
    assert session._recording is True
    assert session._recording_publisher.messages == []


def test_ray_frame_contract_and_endpoint_limit_are_fatal(monkeypatch):
    session = _ray_session_without_ros_runtime()
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    wrong_frame = PointCloud2()
    wrong_frame.header.frame_id = "odom"
    wrong_frame.header.stamp.sec = 1

    session._ray_cloud_callback(wrong_frame)

    assert session._tf_buffer.calls == []
    assert session._ray_recorder.halt_reasons == ["source_frame_mismatch"]
    assert session._recording is False
    assert session._recording_publisher.messages[-1].data is False

    second = _ray_session_without_ros_runtime()
    second._ray_max_rays_per_frame = 1
    monkeypatch.setattr(
        mapping_session_node.point_cloud2,
        "read_points_numpy",
        lambda *args, **kwargs: np.asarray(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32
        ),
    )
    two_returns = PointCloud2()
    two_returns.header.frame_id = "mid360_left_frame"
    two_returns.header.stamp.sec = 2

    second._ray_cloud_callback(two_returns)

    assert second._ray_recorder.recorded == []
    assert second._ray_recorder.halt_reasons == ["ray_contract_error"]
    assert second._recording is False
    assert second._recording_publisher.messages[-1].data is False


def test_halted_ray_recorder_rejects_start_until_reset():
    recorder = _CaptureRayRecorder()
    recorder.status = "halted"
    session = _ray_session_without_ros_runtime(recorder)
    session._recording = False
    session._ray_contract_error = "limit reached"

    response = session._start(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert "reset" in response.message
    assert session._recording is False
    assert session._recording_publisher.messages[-1].data is False


def test_ray_reset_waits_for_octomap_service_readiness():
    recorder = _CaptureRayRecorder()
    session = _ray_session_without_ros_runtime(recorder)
    session._octomap_reset_client = _NotReadyResetClient()

    response = session._reset(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert recorder.reset_count == 0
    assert session._recording is True
    assert session._recording_publisher.messages == []


def test_ready_ray_reset_clears_both_sessions_and_stays_paused(monkeypatch):
    recorder = _CaptureRayRecorder()
    session = _ray_session_without_ros_runtime(recorder)
    session._octomap_reset_client = _ReadyResetClient()
    session._voxels = {(1, 2, 3): (np.asarray([1.0, 2.0, 3.0]), 2)}
    session._latest_occupancy = object()
    session._last_sample_ns = 100
    session._accepted_clouds = 3
    session._dropped_tf = 2
    session._latest_tf_fallbacks = 1
    session._ray_contract_error = "old failure"
    session._ray_dropped_tf = 2
    session._ray_dropped_decode = 1
    session._ray_dropped_empty = 3
    capture_id = session._ray_capture_id
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._reset(Trigger.Request(), Trigger.Response())

    assert response.success is True
    assert recorder.reset_count == 0
    assert session._voxels
    assert session._reset_pending is True

    session._octomap_reset_client.futures[0].succeed()

    assert recorder.reset_count == 1
    assert session._ray_contract_error is None
    assert session._ray_dropped_tf == 0
    assert session._ray_dropped_decode == 0
    assert session._ray_dropped_empty == 0
    assert session._voxels == {}
    assert session._recording is False
    assert session._reset_pending is False
    assert session._ray_capture_id == capture_id
    assert len(session._octomap_reset_client.requests) == 1


def test_reset_transport_failure_retains_all_local_data_paused(monkeypatch):
    recorder = _CaptureRayRecorder()
    session = _ray_session_without_ros_runtime(recorder)
    session._octomap_reset_client = _ReadyResetClient()
    session._voxels = {(1, 2, 3): (np.asarray([1.0, 2.0, 3.0]), 2)}
    session._latest_occupancy = object()
    session._last_sample_ns = 100
    session._accepted_clouds = 3
    session._dropped_tf = 2
    session._latest_tf_fallbacks = 1
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._reset(Trigger.Request(), Trigger.Response())
    session._octomap_reset_client.futures[0].fail(RuntimeError("transport"))

    assert response.success is True
    assert session._reset_pending is False
    assert session._recording is False
    assert session._voxels
    assert session._latest_occupancy is not None
    assert recorder.reset_count == 0
    assert "retained local PCD" in logger.errors[-1]


def test_reset_enqueue_failure_retains_all_local_data_paused(monkeypatch):
    session = _ray_session_without_ros_runtime()
    session._octomap_reset_client = _RaisingResetClient()
    session._voxels = {(1, 2, 3): (np.asarray([1.0, 2.0, 3.0]), 2)}
    session._latest_occupancy = object()
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._reset(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert "retained" in response.message
    assert session._reset_pending is False
    assert session._recording is False
    assert session._voxels
    assert session._latest_occupancy is not None
    assert session._ray_recorder.reset_count == 0


def test_reset_pending_rejects_start_save_and_duplicate_reset():
    session = _ray_session_without_ros_runtime()
    session._recording = False
    session._reset_pending = True
    session._octomap_reset_client = _ReadyResetClient()

    start = session._start(Trigger.Request(), Trigger.Response())
    save = session._save(Trigger.Request(), Trigger.Response())
    reset = session._reset(Trigger.Request(), Trigger.Response())

    assert start.success is False
    assert "pending" in start.message
    assert save.success is False
    assert "pending" in save.message
    assert reset.success is False
    assert "already pending" in reset.message
    assert session._octomap_reset_client.requests == []


def _save_ready_session(tmp_path: Path, recorder: RaySidecarRecorder):
    session = _ray_session_without_ros_runtime(recorder)
    session._recording = False
    session._voxels = {
        (0, 0, 0): (np.asarray([0.1, 0.1, 0.1], dtype=np.float32), 2)
    }
    occupancy = OccupancyGrid()
    occupancy.header.frame_id = "map"
    occupancy.info.width = 1
    occupancy.info.height = 1
    occupancy.info.resolution = 0.1
    occupancy.info.origin.orientation.w = 1.0
    occupancy.data = [0]
    session._latest_occupancy = occupancy
    session._min_observations = 2
    session._accepted_clouds = 1
    session._dropped_tf = 0
    session._latest_tf_fallbacks = 0
    session._revision = "test_revision"
    session._output_root = str(tmp_path / "bundles")
    session._map_id = "test_map"
    session._cloud_topic = "/lio/cloud_registered"
    session._occupancy_topic = "/mapping/projected_map"
    session._voxel_size = 0.05
    session._sample_period = 0.20
    session._ray_cloud_topic = "/mapping/sensor_cloud"
    session._ray_capture_id = recorder.spool_path.name.removesuffix(
        ".records.partial"
    )
    return session


def _real_ray_recorder(tmp_path: Path, name: str) -> RaySidecarRecorder:
    capture_id = uuid.uuid5(uuid.NAMESPACE_URL, f"rm-map-tools-test:{name}").hex
    return RaySidecarRecorder(
        tmp_path
        / "bundles"
        / ".ray_sidecar_sessions"
        / f"{capture_id}.records.partial",
        source_frame="mid360_left_frame",
        min_sample_period_ns=0,
        max_frames=4,
        max_rays_per_frame=4,
        max_total_rays=8,
        max_bytes=4096,
        header_reserve_bytes=2048,
        metadata={"capture_id": capture_id},
    )


def test_enabled_ray_save_requires_pause(tmp_path):
    recorder = _real_ray_recorder(tmp_path, "pause")
    session = _save_ready_session(tmp_path, recorder)
    session._recording = True

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert "pause mapping" in response.message
    recorder.close()


def test_map_export_failure_keeps_ray_spool_and_removes_owned_snapshot(
    tmp_path, monkeypatch
):
    recorder = _real_ray_recorder(tmp_path, "retry")
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    committed_spool = recorder.spool_path.read_bytes()
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    captured = {}

    def fail_export(**kwargs):
        captured["snapshot"] = Path(kwargs["ray_observations_path"])
        assert captured["snapshot"].is_file()
        raise RuntimeError("synthetic bundle failure")

    monkeypatch.setattr(mapping_session_node, "write_candidate_map_bundle", fail_export)

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert response.message == "synthetic bundle failure"
    assert recorder.spool_path.read_bytes() == committed_spool
    assert recorder.status == "recording"
    assert not captured["snapshot"].exists()
    session._close_ray_recorder()
    assert recorder.spool_path.is_file()


@pytest.mark.parametrize(
    "halted,expected_status",
    [(False, "complete"), (True, "incomplete")],
)
def test_enabled_ray_save_attaches_atomic_bundle_artifact(
    tmp_path, monkeypatch, halted, expected_status
):
    recorder = _real_ray_recorder(tmp_path, f"bundle-{expected_status}")
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    if halted:
        recorder.halt("synthetic_contract_failure")
    committed_spool = recorder.spool_path.read_bytes()
    session = _save_ready_session(tmp_path, recorder)
    session._ray_allow_degraded_save = halted
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is True
    manifest_path = Path(response.message)
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    artifact = manifest["artifacts"]["ray_observations"]
    assert artifact["status"] == expected_status
    sidecar_path = manifest_path.parent / artifact["path"]
    assert sidecar_path.is_file()
    header = json.loads(sidecar_path.read_text(encoding="utf-8").splitlines()[0])
    details = manifest["source"]["details"]["ray_observations"]
    assert header["metadata"]["capture_id"] == session._ray_capture_id
    assert details["capture_id"] == session._ray_capture_id
    assert details["degraded_save_override"] is halted
    assert recorder.spool_path.read_bytes() == committed_spool
    assert list(recorder.spool_path.parent.glob("*.snapshot.jsonl")) == []
    if not halted:
        stats = recorder.stats
        assert session._ray_last_exported_complete_prefix == (
            stats.accepted_frames,
            stats.record_bytes,
        )
    else:
        assert session._ray_last_exported_complete_prefix is None
    if halted:
        session._close_ray_recorder()
        assert recorder.spool_path.is_file()
    else:
        recorder.close()


def test_incomplete_ray_sidecar_is_fail_closed_without_override(
    tmp_path, monkeypatch
):
    recorder = _real_ray_recorder(tmp_path, "incomplete-refused")
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    recorder.halt("synthetic_contract_failure")
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    called = []
    monkeypatch.setattr(
        mapping_session_node,
        "write_candidate_map_bundle",
        lambda **kwargs: called.append(kwargs),
    )

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert "incomplete" in response.message
    assert "ray_allow_degraded_save=false" in response.message
    assert called == []
    assert recorder.spool_path.is_file()
    assert list(recorder.spool_path.parent.glob("*.snapshot.jsonl")) == []
    recorder.close()


@pytest.mark.parametrize("record_after_save", [False, True])
def test_normal_shutdown_removes_only_fully_exported_current_spool(
    tmp_path, monkeypatch, record_after_save
):
    recorder = _real_ray_recorder(tmp_path, f"cleanup-{record_after_save}")
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    response = session._save(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert session._ray_last_exported_complete_prefix is not None

    if record_after_save:
        recorder.record_observation(
            stamp_ns=2_000_000_000,
            source_frame="mid360_left_frame",
            origin=(0.0, 0.0, 0.0),
            endpoints=((2.0, 0.0, 0.0),),
        )

    session._close_ray_recorder()

    assert recorder.spool_path.exists() is record_after_save


def test_normal_shutdown_removes_empty_owned_spool(tmp_path, monkeypatch):
    recorder = _real_ray_recorder(tmp_path, "empty-shutdown")
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)

    session._close_ray_recorder()

    assert not recorder.spool_path.exists()


@pytest.mark.parametrize("replacement_kind", ("regular", "symlink"))
def test_normal_shutdown_never_unlinks_replaced_spool_path(
    tmp_path,
    monkeypatch,
    replacement_kind,
):
    recorder = _real_ray_recorder(tmp_path, f"replaced-{replacement_kind}")
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    displaced = recorder.spool_path.with_suffix(".original")
    recorder.spool_path.rename(displaced)

    replacement_bytes = b"not owned by the recorder"
    if replacement_kind == "regular":
        recorder.spool_path.write_bytes(replacement_bytes)
    else:
        replacement_target = tmp_path / "replacement-spool-target"
        replacement_target.write_bytes(replacement_bytes)
        recorder.spool_path.symlink_to(replacement_target)

    session._close_ray_recorder()

    assert displaced.is_file()
    if replacement_kind == "regular":
        assert recorder.spool_path.read_bytes() == replacement_bytes
    else:
        assert recorder.spool_path.is_symlink()
    assert any("refusing to remove replaced" in item for item in logger.warnings)


@pytest.mark.parametrize("failure_kind", ("competitor", "post_publish_error"))
def test_snapshot_failure_never_unlinks_unowned_or_ambiguous_destination(
    tmp_path,
    monkeypatch,
    failure_kind,
):
    recorder = _real_ray_recorder(tmp_path, f"snapshot-{failure_kind}")
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    attempted_paths = []
    preserved_bytes = b"entry whose ownership is not established"

    def fail_snapshot(destination):
        path = Path(destination)
        attempted_paths.append(path)
        if failure_kind == "competitor":
            missing = tmp_path / "missing-competitor-target"
            path.symlink_to(missing)
            raise ValueError("competitor appeared while publishing")
        path.write_bytes(preserved_bytes)
        raise OSError("parent fsync failed after publication")

    monkeypatch.setattr(recorder, "snapshot", fail_snapshot)

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert len(attempted_paths) == 1
    attempted = attempted_paths[0]
    if failure_kind == "competitor":
        assert attempted.is_symlink()
    else:
        assert attempted.read_bytes() == preserved_bytes
    session._close_ray_recorder()


def test_zero_ray_observations_fail_closed_then_dynamic_override_saves_base(
    tmp_path, monkeypatch
):
    recorder = _real_ray_recorder(tmp_path, "empty")
    session = _save_ready_session(tmp_path, recorder)
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    captured = {}

    def capture_export(**kwargs):
        captured.update(kwargs)
        return tmp_path / "test_map.bundle.yaml"

    monkeypatch.setattr(
        mapping_session_node, "write_candidate_map_bundle", capture_export
    )

    refused = session._save(Trigger.Request(), Trigger.Response())

    assert refused.success is False
    assert "no accepted observations" in refused.message
    assert captured == {}
    assert recorder.spool_path.is_file()

    session._ray_allow_degraded_save = True
    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is True
    assert "ray_observations_path" not in captured
    details = captured["source_details"]["ray_observations"]
    assert details["export_outcome"] == "no_observations"
    assert details["degraded_save_override"] is True
    assert details["capture_id"] == session._ray_capture_id
    assert recorder.status == "recording"
    assert logger.warnings
    session._close_ray_recorder()
    assert not recorder.spool_path.exists()


@pytest.mark.parametrize("allow_degraded_save", [False, True])
def test_ray_snapshot_failure_requires_explicit_degraded_override(
    tmp_path, monkeypatch, allow_degraded_save
):
    recorder = RaySidecarRecorder(
        tmp_path / "snapshot-failure.records.partial",
        source_frame="mid360_left_frame",
        min_sample_period_ns=0,
        max_frames=2,
        max_rays_per_frame=2,
        max_total_rays=4,
        max_bytes=4096,
        header_reserve_bytes=512,
        metadata={"too_large": "x" * 1000},
    )
    recorder.record_observation(
        stamp_ns=1_000_000_000,
        source_frame="mid360_left_frame",
        origin=(0.0, 0.0, 0.0),
        endpoints=((1.0, 0.0, 0.0),),
    )
    session = _save_ready_session(tmp_path, recorder)
    session._ray_allow_degraded_save = allow_degraded_save
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    captured = {}

    def capture_export(**kwargs):
        captured.update(kwargs)
        return tmp_path / "test_map.bundle.yaml"

    monkeypatch.setattr(
        mapping_session_node, "write_candidate_map_bundle", capture_export
    )

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is allow_degraded_save
    if allow_degraded_save:
        assert "ray_observations_path" not in captured
        details = captured["source_details"]["ray_observations"]
        assert details["export_outcome"] == "snapshot_failed"
        assert details["status"] == "halted"
        assert details["halt_reason"] == "header_reserve_exceeded"
        assert details["degraded_save_override"] is True
    else:
        assert "snapshot failed" in response.message
        assert captured == {}
    assert recorder.spool_path.is_file()
    assert session._ray_last_exported_complete_prefix is None
    session._close_ray_recorder()
    assert recorder.spool_path.is_file()


@pytest.mark.parametrize("allow_degraded_save", [False, True])
def test_missing_ray_recorder_requires_explicit_degraded_override(
    tmp_path, monkeypatch, allow_degraded_save
):
    owned_recorder = _real_ray_recorder(tmp_path, "missing")
    session = _save_ready_session(tmp_path, owned_recorder)
    session._ray_recorder = None
    session._ray_allow_degraded_save = allow_degraded_save
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    captured = {}

    def capture_export(**kwargs):
        captured.update(kwargs)
        return tmp_path / "test_map.bundle.yaml"

    monkeypatch.setattr(
        mapping_session_node, "write_candidate_map_bundle", capture_export
    )

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is allow_degraded_save
    if allow_degraded_save:
        details = captured["source_details"]["ray_observations"]
        assert details["export_outcome"] == "snapshot_failed"
        assert details["degraded_save_override"] is True
    else:
        assert "unavailable" in response.message
        assert captured == {}
    assert owned_recorder.spool_path.is_file()
    owned_recorder.close()


def test_default_off_keeps_existing_save_contract(tmp_path, monkeypatch):
    recorder = _real_ray_recorder(tmp_path, "unused")
    session = _save_ready_session(tmp_path, recorder)
    session._ray_enabled = False
    session._recording = True
    logger = _CaptureLogger()
    monkeypatch.setattr(MappingSessionNode, "get_logger", lambda self: logger)
    captured = {}

    def capture_export(**kwargs):
        captured.update(kwargs)
        return tmp_path / "test_map.bundle.yaml"

    monkeypatch.setattr(
        mapping_session_node, "write_candidate_map_bundle", capture_export
    )

    response = session._save(Trigger.Request(), Trigger.Response())

    assert response.success is True
    assert "ray_observations_path" not in captured
    assert "ray_observations" not in captured["source_details"]
    assert session._recording is True
    recorder.close()
