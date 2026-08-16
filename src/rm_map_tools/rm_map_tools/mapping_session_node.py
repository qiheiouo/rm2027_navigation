from __future__ import annotations

import math
import stat
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .immutable_output import fsync_directory
from .map_export import write_candidate_map_bundle
from .ray_observations import (
    DEFAULT_MAX_BYTES,
    DEFAULT_MAX_FRAMES,
    DEFAULT_MAX_RAYS_PER_FRAME,
    DEFAULT_MAX_TOTAL_RAYS,
    RayObservationError,
    RayObservationReadLimits,
    RaySidecarRecorder,
    prepare_map_ray_observation,
)


_NON_LIDAR_RAY_SOURCE_FRAMES = frozenset(
    {"map", "odom", "base_link", "base_footprint", "lio_imu_link"}
)


def _validate_ray_input_contract(
    *,
    enabled: bool,
    ray_cloud_topic: str,
    registered_cloud_topic: str,
    ray_source_frame: str,
) -> None:
    if not enabled:
        return
    normalized_ray_topic = ray_cloud_topic.strip().lstrip("/")
    normalized_registered_topic = registered_cloud_topic.strip().lstrip("/")
    if not ray_source_frame:
        raise ValueError(
            "ray_source_frame must name the physical lidar frame when "
            "record_ray_observations is enabled"
        )
    if not normalized_ray_topic or normalized_ray_topic == normalized_registered_topic:
        raise ValueError(
            "ray_cloud_topic must be a non-empty sensor-frame topic distinct "
            "from registered_cloud_topic"
        )
    if ray_source_frame in _NON_LIDAR_RAY_SOURCE_FRAMES:
        raise ValueError(
            "ray_source_frame must be a physical lidar origin, not a world, "
            "body or IMU frame"
        )


def _validate_ray_recorder_limits(
    *,
    enabled: bool,
    max_frames: int,
    max_rays_per_frame: int,
    max_total_rays: int,
    max_bytes: int,
) -> None:
    if not enabled:
        return
    consumer = RayObservationReadLimits()
    configured = {
        "ray_max_frames": (max_frames, consumer.max_frames),
        "ray_max_rays_per_frame": (
            max_rays_per_frame,
            consumer.max_rays_per_frame,
        ),
        "ray_max_total_rays": (max_total_rays, consumer.max_total_rays),
        "ray_max_bytes": (max_bytes, consumer.max_bytes),
    }
    exceeded = [
        f"{name}={value} exceeds consumer limit {limit}"
        for name, (value, limit) in configured.items()
        if value > limit
    ]
    if exceeded:
        raise ValueError("; ".join(exceeded))


def _prepare_ray_session_root(output_root: str | Path) -> Path:
    """Create the private recorder directory without following a child symlink."""
    canonical_output_root = Path(output_root).expanduser().resolve()
    session_root = canonical_output_root / ".ray_sidecar_sessions"
    if session_root.is_symlink():
        raise ValueError(
            "ray sidecar session directory must not be a symlink: "
            f"{session_root}"
        )
    session_root.mkdir(parents=True, exist_ok=True)
    resolved_session_root = session_root.resolve()
    if resolved_session_root != session_root:
        raise ValueError(
            "ray sidecar session directory escaped output_root: "
            f"{resolved_session_root}"
        )
    return resolved_session_root


def _regular_path_matches_identity(
    path: Path,
    identity: tuple[int, int],
) -> bool:
    """Check an exact regular-file path entry without following symlinks."""
    try:
        path_stat = path.lstat()
    except FileNotFoundError:
        return False
    return stat.S_ISREG(path_stat.st_mode) and (
        path_stat.st_dev,
        path_stat.st_ino,
    ) == identity


def _rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("transform quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class MappingSessionNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_session")
        self._cloud_topic = self.declare_parameter(
            "registered_cloud_topic", "/lio/cloud_registered"
        ).value
        self._occupancy_topic = self.declare_parameter(
            "occupancy_topic", "/mapping/projected_map"
        ).value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._output_root = self.declare_parameter(
            "output_root", "/tmp/rm27_maps"
        ).value
        self._map_id = self.declare_parameter("map_id", "rm_field").value
        self._revision = self.declare_parameter("revision", "auto").value
        self._voxel_size = float(self.declare_parameter("voxel_size", 0.05).value)
        self._sample_period = float(
            self.declare_parameter("sample_period_sec", 0.20).value
        )
        self._min_z = float(self.declare_parameter("min_z", -1.0).value)
        self._max_z = float(self.declare_parameter("max_z", 3.0).value)
        self._tf_timeout = float(
            self.declare_parameter("transform_timeout_sec", 0.10).value
        )
        self._allow_latest_tf_fallback = bool(
            self.declare_parameter("allow_latest_transform_fallback", False).value
        )
        self._max_voxels = int(
            self.declare_parameter("max_voxels", 2000000).value
        )
        self._min_observations = int(
            self.declare_parameter("min_observations", 2).value
        )
        self._recording = bool(self.declare_parameter("autostart", True).value)
        self._ray_enabled = bool(
            self.declare_parameter("record_ray_observations", False).value
        )
        self._ray_cloud_topic = str(
            self.declare_parameter(
                "ray_cloud_topic", "/mapping/sensor_cloud"
            ).value
        )
        self._ray_source_frame = str(
            self.declare_parameter("ray_source_frame", "").value
        ).strip().lstrip("/")
        self._ray_sample_period = float(
            self.declare_parameter("ray_sample_period_sec", 0.20).value
        )
        self._ray_min_range = float(
            self.declare_parameter("ray_min_range", 0.30).value
        )
        self._ray_max_range = float(
            self.declare_parameter("ray_max_range", 12.0).value
        )
        self._ray_voxel_size = float(
            self.declare_parameter("ray_voxel_size", 0.10).value
        )
        self._ray_max_frames = int(
            self.declare_parameter("ray_max_frames", DEFAULT_MAX_FRAMES).value
        )
        self._ray_max_rays_per_frame = int(
            self.declare_parameter(
                "ray_max_rays_per_frame", DEFAULT_MAX_RAYS_PER_FRAME
            ).value
        )
        self._ray_max_total_rays = int(
            self.declare_parameter(
                "ray_max_total_rays", DEFAULT_MAX_TOTAL_RAYS
            ).value
        )
        self._ray_max_bytes = int(
            self.declare_parameter("ray_max_bytes", DEFAULT_MAX_BYTES).value
        )
        self.declare_parameter("ray_allow_degraded_save", False)

        if self._map_frame != "map":
            raise ValueError("mapping_session map_frame must remain canonical 'map'")
        if not Path(self._output_root).expanduser().is_absolute():
            raise ValueError("mapping_session output_root must be an absolute path")
        if self._voxel_size <= 0.0 or self._sample_period < 0.0:
            raise ValueError("voxel_size must be positive and sample_period_sec non-negative")
        if (
            self._min_z >= self._max_z
            or self._max_voxels <= 0
            or self._min_observations <= 0
        ):
            raise ValueError("height limits, max_voxels or min_observations are invalid")
        _validate_ray_input_contract(
            enabled=self._ray_enabled,
            ray_cloud_topic=self._ray_cloud_topic,
            registered_cloud_topic=str(self._cloud_topic),
            ray_source_frame=self._ray_source_frame,
        )
        if self._ray_enabled and (
            not math.isfinite(self._ray_sample_period)
            or self._ray_sample_period < 0.0
            or not math.isfinite(self._ray_min_range)
            or not math.isfinite(self._ray_max_range)
            or not math.isfinite(self._ray_voxel_size)
            or self._ray_min_range < 0.0
            or self._ray_max_range <= self._ray_min_range
            or self._ray_voxel_size <= 0.0
            or self._ray_max_frames <= 0
            or self._ray_max_rays_per_frame <= 0
            or self._ray_max_total_rays < self._ray_max_rays_per_frame
            or self._ray_max_bytes <= 0
        ):
            raise ValueError("ray observation ranges, sampling or limits are invalid")
        _validate_ray_recorder_limits(
            enabled=self._ray_enabled,
            max_frames=self._ray_max_frames,
            max_rays_per_frame=self._ray_max_rays_per_frame,
            max_total_rays=self._ray_max_total_rays,
            max_bytes=self._ray_max_bytes,
        )

        self._lock = threading.Lock()
        self._voxels: dict[tuple[int, int, int], tuple[np.ndarray, int]] = {}
        self._latest_occupancy: OccupancyGrid | None = None
        self._last_sample_ns = -1
        self._accepted_clouds = 0
        self._dropped_tf = 0
        self._latest_tf_fallbacks = 0
        self._ray_recorder: RaySidecarRecorder | None = None
        self._ray_subscription = None
        self._ray_lock = threading.Lock()
        self._ray_contract_error: str | None = None
        self._ray_dropped_tf = 0
        self._ray_dropped_decode = 0
        self._ray_dropped_empty = 0
        self._ray_capture_id: str | None = None
        self._ray_last_exported_complete_prefix: tuple[int, int] | None = None
        self._reset_pending = False

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        occupancy_qos = QoSProfile(depth=1)
        occupancy_qos.reliability = ReliabilityPolicy.RELIABLE
        occupancy_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._recording_publisher = self.create_publisher(
            Bool, "/mapping/recording", occupancy_qos
        )
        self.create_subscription(
            OccupancyGrid,
            self._occupancy_topic,
            self._occupancy_callback,
            occupancy_qos,
        )
        if self._ray_enabled:
            self._ray_capture_id = uuid.uuid4().hex
            resolved_session_root = _prepare_ray_session_root(self._output_root)
            spool_path = resolved_session_root / (
                f"{self._ray_capture_id}.records.partial"
            )
            self._ray_recorder = RaySidecarRecorder(
                spool_path,
                source_frame=self._ray_source_frame,
                min_sample_period_ns=int(round(self._ray_sample_period * 1.0e9)),
                max_frames=self._ray_max_frames,
                max_rays_per_frame=self._ray_max_rays_per_frame,
                max_total_rays=self._ray_max_total_rays,
                max_bytes=self._ray_max_bytes,
                metadata={
                    "source_topic": self._ray_cloud_topic,
                    "map_id": self._map_id,
                    "revision": self._revision,
                    "capture_id": self._ray_capture_id,
                    "endpoint_policy": "physical_returns_only_no_max_range_clipping",
                },
            )
            self._ray_subscription = self.create_subscription(
                PointCloud2,
                self._ray_cloud_topic,
                self._ray_cloud_callback,
                qos_profile_sensor_data,
            )

        self.create_service(Trigger, "/mapping/start", self._start)
        self.create_service(Trigger, "/mapping/stop", self._stop)
        self.create_service(Trigger, "/mapping/reset", self._reset)
        self.create_service(Trigger, "/mapping/save", self._save)
        self._octomap_reset_client = self.create_client(Empty, "/octomap_server/reset")
        self.create_timer(5.0, self._report_progress)
        self._set_recording(self._recording)
        self.get_logger().info(
            "mapping session ready: cloud=%s occupancy=%s output_root=%s "
            "autostart=%s ray_observations=%s"
            % (
                self._cloud_topic,
                self._occupancy_topic,
                self._output_root,
                self._recording,
                self._ray_enabled,
            )
        )

    def _occupancy_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self._map_frame:
            self.get_logger().warning(
                "ignore occupancy grid in frame '%s'; expected '%s'"
                % (msg.header.frame_id, self._map_frame)
            )
            return
        with self._lock:
            self._latest_occupancy = msg

    def _halt_ray_recording(self, reason: str, message: str) -> None:
        if not getattr(self, "_ray_enabled", False):
            return
        recorder = self._ray_recorder
        with self._ray_lock:
            self._ray_contract_error = message
            if recorder is not None:
                try:
                    recorder.halt(reason)
                except RayObservationError:
                    pass
        self._set_recording(False)
        self.get_logger().error(
            f"ray observation recording halted ({reason}): {message}; reset required"
        )

    def _ray_cloud_callback(self, msg: PointCloud2) -> None:
        if (
            not self._ray_enabled
            or not self._recording
            or self._ray_contract_error is not None
        ):
            return

        source_frame = msg.header.frame_id.strip().lstrip("/")
        if source_frame != self._ray_source_frame:
            self._halt_ray_recording(
                "source_frame_mismatch",
                "ray cloud frame %r does not match configured physical lidar frame %r"
                % (source_frame, self._ray_source_frame),
            )
            return

        stamp = Time.from_msg(msg.header.stamp)
        stamp_ns = stamp.nanoseconds
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._ray_source_frame,
                stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self._ray_dropped_tf += 1
            self.get_logger().warning(
                "drop ray cloud without timestamped %s <- %s transform: %s"
                % (self._map_frame, self._ray_source_frame, exc),
                throttle_duration_sec=2.0,
            )
            return

        try:
            points = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
            points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        except (
            AssertionError,
            KeyError,
            OverflowError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            self._ray_dropped_decode += 1
            self.get_logger().warning(
                f"drop undecodable ray PointCloud2: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        q = transform.transform.rotation
        t = transform.transform.translation
        try:
            prepared = prepare_map_ray_observation(
                points,
                stamp_ns=stamp_ns,
                translation=(t.x, t.y, t.z),
                quaternion_xyzw=(q.x, q.y, q.z, q.w),
                min_range=self._ray_min_range,
                max_range=self._ray_max_range,
                voxel_size=self._ray_voxel_size,
                max_rays_per_frame=self._ray_max_rays_per_frame,
            )
            if not prepared.endpoints:
                self._ray_dropped_empty += 1
                return
            recorder = self._ray_recorder
            if recorder is None:
                raise RayObservationError(
                    "enabled ray recorder is unexpectedly unavailable"
                )
            with self._ray_lock:
                if not self._recording or self._ray_contract_error is not None:
                    return
                recorder.record_observation(
                    stamp_ns=prepared.stamp_ns,
                    source_frame=self._ray_source_frame,
                    origin=prepared.origin,
                    endpoints=prepared.endpoints,
                )
        except (RayObservationError, ValueError) as exc:
            self._halt_ray_recording("ray_contract_error", str(exc))

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if not self._recording or not msg.header.frame_id:
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if (
            self._last_sample_ns >= 0
            and stamp_ns >= self._last_sample_ns
            and stamp_ns - self._last_sample_ns < int(self._sample_period * 1.0e9)
        ):
            return

        used_latest_transform = False
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            if self._allow_latest_tf_fallback:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._map_frame,
                        msg.header.frame_id,
                        Time(),
                        timeout=Duration(seconds=self._tf_timeout),
                    )
                    used_latest_transform = True
                except TransformException as latest_exc:
                    self._dropped_tf += 1
                    self.get_logger().warning(
                        "drop registered cloud: timestamped %s <- %s failed (%s); "
                        "latest transform also failed (%s)"
                        % (
                            self._map_frame,
                            msg.header.frame_id,
                            exc,
                            latest_exc,
                        ),
                        throttle_duration_sec=2.0,
                    )
                    return
            else:
                self._dropped_tf += 1
                self.get_logger().warning(
                    "drop registered cloud without timestamped %s <- %s transform: %s"
                    % (self._map_frame, msg.header.frame_id, exc),
                    throttle_duration_sec=2.0,
                )
                return

        try:
            points = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (AssertionError, KeyError, ValueError) as exc:
            self.get_logger().error(f"cannot decode registered PointCloud2: {exc}")
            return
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if points.size == 0:
            return

        q = transform.transform.rotation
        t = transform.transform.translation
        rotation = _rotation_matrix(q.x, q.y, q.z, q.w)
        points = points @ rotation.T + np.asarray([t.x, t.y, t.z])
        points = points[(points[:, 2] >= self._min_z) & (points[:, 2] <= self._max_z)]
        if points.size == 0:
            return

        keys = np.floor(points / self._voxel_size).astype(np.int64)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        unique_keys = keys[unique_indices]
        unique_points = points[unique_indices].astype(np.float32)

        voxel_limit_reached = False
        with self._lock:
            normalized_keys = [
                (int(key[0]), int(key[1]), int(key[2])) for key in unique_keys
            ]
            additional_voxels = sum(
                1 for key in normalized_keys if key not in self._voxels
            )
            projected_size = len(self._voxels) + additional_voxels
            if projected_size > self._max_voxels:
                voxel_limit_reached = True
            else:
                for key, point in zip(normalized_keys, unique_points):
                    previous = self._voxels.get(key)
                    if previous is None:
                        self._voxels[key] = (point, 1)
                        continue
                    previous_point, count = previous
                    next_count = min(count + 1, 65535)
                    averaged = previous_point + (point - previous_point) / float(next_count)
                    self._voxels[key] = (averaged.astype(np.float32), next_count)
                self._accepted_clouds += 1
                if used_latest_transform:
                    self._latest_tf_fallbacks += 1
                self._last_sample_ns = stamp_ns

        if voxel_limit_reached:
            # Publish outside the accumulator lock. A recording-state subscriber
            # may immediately stop another mapping input, so do not hold the
            # session data lock while notifying the ROS graph.
            self._set_recording(False)
            self.get_logger().error(
                "mapping stopped before exceeding max_voxels=%d; save or reset the session"
                % self._max_voxels
            )

    def _start(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            reset_pending = getattr(self, "_reset_pending", False)
        if reset_pending:
            response.success = False
            response.message = "OctoMap reset is pending; wait for completion"
            return response
        if getattr(self, "_ray_enabled", False):
            recorder = self._ray_recorder
            with self._ray_lock:
                ray_error = self._ray_contract_error
                recorder_status = recorder.status if recorder is not None else "missing"
            if ray_error is not None or recorder_status != "recording":
                self._set_recording(False)
                response.success = False
                response.message = (
                    "ray observation recorder is halted or unavailable; reset the "
                    "mapping session before starting"
                )
                return response
        self._set_recording(True)
        response.success = True
        response.message = "mapping recording enabled"
        return response

    def _stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._set_recording(False)
        response.success = True
        response.message = "mapping recording paused"
        return response

    def _reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if getattr(self, "_reset_pending", False):
                response.success = False
                response.message = "OctoMap reset is already pending"
                return response
        if getattr(self, "_ray_enabled", False) and self._ray_recorder is None:
            response.success = False
            response.message = (
                "ray observation recorder is unavailable; reset was not requested"
            )
            return response
        if not self._octomap_reset_client.service_is_ready():
            response.success = False
            response.message = "octomap reset service is not ready; session was not changed"
            return response
        self._set_recording(False)
        with self._lock:
            self._reset_pending = True
        try:
            future = self._octomap_reset_client.call_async(Empty.Request())
            future.add_done_callback(self._finish_reset)
        except Exception as exc:
            with self._lock:
                self._reset_pending = False
            response.success = False
            response.message = (
                f"cannot request OctoMap reset; local session was retained: {exc}"
            )
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = (
            "OctoMap reset requested; local mapping session remains paused and "
            "will clear after transport success"
        )
        return response

    def _finish_reset(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            with self._lock:
                self._reset_pending = False
            self.get_logger().error(
                "OctoMap reset transport failed; retained local PCD, occupancy "
                f"and ray evidence while paused: {exc}"
            )
            return

        local_session_cleared = False
        if getattr(self, "_ray_enabled", False):
            recorder = self._ray_recorder
            if recorder is None:
                with self._lock:
                    self._reset_pending = False
                self.get_logger().error(
                    "OctoMap reset succeeded but ray recorder is unavailable; "
                    "retained local PCD and occupancy while paused"
                )
                return
            try:
                with self._ray_lock:
                    recorder.reset()
                    self._ray_contract_error = None
                    self._ray_dropped_tf = 0
                    self._ray_dropped_decode = 0
                    self._ray_dropped_empty = 0
                    self._ray_last_exported_complete_prefix = None
                    with self._lock:
                        self._clear_local_session_locked()
                        local_session_cleared = True
            except RayObservationError as exc:
                with self._lock:
                    self._reset_pending = False
                self.get_logger().error(
                    "OctoMap reset succeeded but local ray reset failed; "
                    f"retained PCD and occupancy while paused: {exc}"
                )
                return

        if not local_session_cleared:
            with self._lock:
                self._clear_local_session_locked()
        self.get_logger().info(
            "OctoMap and local mapping session reset completed; recording remains paused"
        )

    def _clear_local_session_locked(self) -> None:
        self._voxels.clear()
        self._latest_occupancy = None
        self._last_sample_ns = -1
        self._accepted_clouds = 0
        self._dropped_tf = 0
        self._latest_tf_fallbacks = 0
        self._reset_pending = False

    def _set_recording(self, recording: bool) -> None:
        with self._lock:
            self._recording = recording
        msg = Bool()
        msg.data = recording
        self._recording_publisher.publish(msg)

    def _degraded_ray_save_allowed(self) -> bool:
        return bool(self.get_parameter("ray_allow_degraded_save").value)

    def _save(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if getattr(self, "_ray_enabled", False):
            # Any failed or degraded save must retain the records spool. A new
            # successful complete attachment below may re-arm normal cleanup.
            self._ray_last_exported_complete_prefix = None
        with self._lock:
            if getattr(self, "_reset_pending", False):
                response.success = False
                response.message = "OctoMap reset is pending; save is unavailable"
                return response
            if getattr(self, "_ray_enabled", False) and self._recording:
                response.success = False
                response.message = (
                    "pause mapping before saving a ray-observation candidate bundle"
                )
                return response
            if not self._voxels:
                response.success = False
                response.message = "no registered pointcloud samples have been accumulated"
                return response
            if self._latest_occupancy is None:
                response.success = False
                response.message = "no projected occupancy grid has been received"
                return response
            stable_points = [
                point
                for point, count in self._voxels.values()
                if count >= self._min_observations
            ]
            if not stable_points:
                response.success = False
                response.message = (
                    "no PCD voxels satisfy min_observations=%d"
                    % self._min_observations
                )
                return response
            points = np.asarray(stable_points, dtype=np.float32)
            occupancy = self._latest_occupancy
            accepted_clouds = self._accepted_clouds
            dropped_tf = self._dropped_tf
            latest_tf_fallbacks = self._latest_tf_fallbacks

        revision = self._revision
        if revision.strip().lower() == "auto":
            revision = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        q = occupancy.info.origin.orientation
        quaternion_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if quaternion_norm <= 1.0e-12:
            response.success = False
            response.message = "occupancy origin has an invalid zero quaternion"
            return response
        qx, qy, qz, qw = (
            q.x / quaternion_norm,
            q.y / quaternion_norm,
            q.z / quaternion_norm,
            q.w / quaternion_norm,
        )
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch_sine = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(pitch_sine)
        if abs(roll) > 1.0e-3 or abs(pitch) > 1.0e-3:
            response.success = False
            response.message = "occupancy origin must be planar (roll/pitch near zero)"
            return response
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        origin = occupancy.info.origin.position

        source_details = {
            "registered_cloud_topic": self._cloud_topic,
            "occupancy_topic": self._occupancy_topic,
            "voxel_size": self._voxel_size,
            "sample_period_sec": self._sample_period,
            "min_observations": self._min_observations,
            "accepted_clouds": accepted_clouds,
            "dropped_tf_clouds": dropped_tf,
            "latest_transform_fallback_clouds": latest_tf_fallbacks,
        }
        ray_snapshot_path: Path | None = None
        ray_snapshot_status: str | None = None
        ray_snapshot_prefix: tuple[int, int] | None = None
        if getattr(self, "_ray_enabled", False):
            allow_degraded_save = self._degraded_ray_save_allowed()
            recorder = self._ray_recorder
            ray_failure_message: str | None = None
            if recorder is None:
                ray_details = {
                    "enabled": True,
                    "capture_id": self._ray_capture_id,
                    "degraded_save_override": allow_degraded_save,
                    "export_outcome": "snapshot_failed",
                    "error": "recorder unavailable",
                }
                ray_failure_message = (
                    "ray observations are enabled but the recorder is unavailable"
                )
            else:
                with self._ray_lock:
                    stats = recorder.stats
                    ray_details = {
                        "enabled": True,
                        "capture_id": self._ray_capture_id,
                        "degraded_save_override": allow_degraded_save,
                        "source_topic": self._ray_cloud_topic,
                        "source_frame": self._ray_source_frame,
                        "status": stats.status,
                        "halt_reason": stats.halt_reason,
                        "contract_error": self._ray_contract_error,
                        "received_frames": stats.received_frames,
                        "accepted_frames": stats.accepted_frames,
                        "total_rays": stats.total_rays,
                        "record_bytes": stats.record_bytes,
                        "dropped_tf_frames": self._ray_dropped_tf,
                        "dropped_decode_frames": self._ray_dropped_decode,
                        "dropped_empty_frames": self._ray_dropped_empty,
                    }
                    if stats.accepted_frames > 0:
                        requested_snapshot = recorder.spool_path.parent / (
                            f"{uuid.uuid4().hex}.snapshot.jsonl"
                        )
                        try:
                            snapshot = recorder.snapshot(requested_snapshot)
                        except Exception as exc:
                            try:
                                recorder.halt("snapshot_failure")
                            except RayObservationError:
                                pass
                            failed_stats = recorder.stats
                            ray_details["status"] = failed_stats.status
                            ray_details["halt_reason"] = failed_stats.halt_reason
                            ray_details["export_outcome"] = "snapshot_failed"
                            ray_details["error"] = str(exc)
                            ray_failure_message = f"ray sidecar snapshot failed: {exc}"
                        else:
                            ray_snapshot_path = snapshot.path
                            ray_snapshot_status = snapshot.status
                            ray_snapshot_prefix = (
                                stats.accepted_frames,
                                stats.record_bytes,
                            )
                            ray_details["export_outcome"] = (
                                "attached"
                                if snapshot.status == "complete"
                                else "attached_incomplete_override"
                            )
                            ray_details["sidecar_status"] = snapshot.status
                            ray_details["sidecar_sha256"] = snapshot.sha256
                            ray_details["sidecar_bytes"] = snapshot.bytes
                            if snapshot.status != "complete":
                                ray_failure_message = (
                                    "ray sidecar is incomplete; reset and capture "
                                    "complete evidence before saving"
                                )
                    else:
                        ray_details["export_outcome"] = "no_observations"
                        ray_failure_message = (
                            "ray recorder has no accepted observations"
                        )
            if ray_failure_message is not None:
                ray_details["degraded_reason"] = ray_failure_message
                if not allow_degraded_save:
                    if ray_snapshot_path is not None:
                        try:
                            ray_snapshot_path.unlink(missing_ok=True)
                        except OSError as cleanup_exc:
                            self.get_logger().warning(
                                "cannot remove owned rejected ray snapshot %s: %s"
                                % (ray_snapshot_path, cleanup_exc)
                            )
                        ray_snapshot_path = None
                    response.success = False
                    response.message = (
                        f"{ray_failure_message}; /mapping/save refused because "
                        "ray_allow_degraded_save=false"
                    )
                    self.get_logger().error(response.message)
                    return response
                self.get_logger().warning(
                    f"{ray_failure_message}; explicit ray_allow_degraded_save=true "
                    "permits degraded candidate export"
                )
            source_details["ray_observations"] = ray_details

        try:
            export_arguments = {
                "output_root": self._output_root,
                "map_id": self._map_id,
                "revision": revision,
                "points": points,
                "occupancy_values": occupancy.data,
                "occupancy_width": occupancy.info.width,
                "occupancy_height": occupancy.info.height,
                "occupancy_resolution": occupancy.info.resolution,
                "occupancy_origin": [origin.x, origin.y, yaw],
                "source_method": "registered_pointcloud_plus_octomap_projection",
                "source_details": source_details,
            }
            if ray_snapshot_path is not None:
                export_arguments["ray_observations_path"] = ray_snapshot_path
            manifest = write_candidate_map_bundle(
                **export_arguments,
            )
        except Exception as exc:
            self.get_logger().error(f"map bundle export failed: {exc}")
            response.success = False
            response.message = str(exc)
            return response
        finally:
            if ray_snapshot_path is not None:
                try:
                    ray_snapshot_path.unlink(missing_ok=True)
                except OSError as exc:
                    self.get_logger().warning(
                        f"cannot remove owned temporary ray snapshot "
                        f"{ray_snapshot_path}: {exc}"
                    )

        if (
            getattr(self, "_ray_enabled", False)
            and ray_snapshot_status == "complete"
            and ray_snapshot_prefix is not None
            and self._ray_recorder is not None
        ):
            with self._ray_lock:
                current_stats = self._ray_recorder.stats
                if (
                    current_stats.status == "recording"
                    and (
                        current_stats.accepted_frames,
                        current_stats.record_bytes,
                    )
                    == ray_snapshot_prefix
                ):
                    self._ray_last_exported_complete_prefix = ray_snapshot_prefix

        response.success = True
        response.message = str(manifest)
        self.get_logger().info(f"candidate map bundle saved: {manifest}")
        return response

    def _report_progress(self) -> None:
        with self._lock:
            voxel_count = len(self._voxels)
            stable_voxel_count = sum(
                1
                for _, count in self._voxels.values()
                if count >= self._min_observations
            )
            cloud_count = self._accepted_clouds
            has_occupancy = self._latest_occupancy is not None
        ray_progress = ""
        if getattr(self, "_ray_enabled", False):
            recorder = self._ray_recorder
            with self._ray_lock:
                if recorder is None:
                    ray_progress = " ray_status=missing"
                else:
                    stats = recorder.stats
                    ray_progress = (
                        " ray_status=%s ray_frames=%d ray_count=%d ray_bytes=%d "
                        "ray_tf_drops=%d ray_decode_drops=%d ray_empty_drops=%d"
                        % (
                            stats.status,
                            stats.accepted_frames,
                            stats.total_rays,
                            stats.record_bytes,
                            self._ray_dropped_tf,
                            self._ray_dropped_decode,
                            self._ray_dropped_empty,
                        )
                    )
        self.get_logger().info(
            "mapping progress: recording=%s clouds=%d voxels=%d stable_voxels=%d "
            "occupancy=%s tf_drops=%d latest_tf_fallbacks=%d%s"
            % (
                self._recording,
                cloud_count,
                voxel_count,
                stable_voxel_count,
                has_occupancy,
                self._dropped_tf,
                self._latest_tf_fallbacks,
                ray_progress,
            )
        )

    def _close_ray_recorder(self) -> None:
        recorder = getattr(self, "_ray_recorder", None)
        if recorder is None:
            return
        try:
            with self._ray_lock:
                stats = recorder.stats
                prefix = (stats.accepted_frames, stats.record_bytes)
                capture_id = getattr(self, "_ray_capture_id", None)
                spool_path = recorder.spool_path
                spool_identity = recorder.spool_identity
                expected_parent = (
                    Path(self._output_root).expanduser().resolve()
                    / ".ray_sidecar_sessions"
                )
                eligible_to_unlink = (
                    stats.status == "recording"
                    and (
                        prefix == (0, 0)
                        or prefix
                        == getattr(
                            self, "_ray_last_exported_complete_prefix", None
                        )
                    )
                    and capture_id is not None
                    and spool_path.parent == expected_parent
                    and spool_path.name == f"{capture_id}.records.partial"
                )
                try:
                    recorder.close()
                except RayObservationError as exc:
                    self.get_logger().error(
                        f"cannot durably close ray records-only spool: {exc}"
                    )
                    return
                if eligible_to_unlink:
                    try:
                        if not _regular_path_matches_identity(
                            spool_path,
                            spool_identity,
                        ):
                            self.get_logger().warning(
                                "refusing to remove replaced or missing owned ray "
                                f"spool path {spool_path}"
                            )
                            return
                        spool_path.unlink()
                        fsync_directory(spool_path.parent)
                    except OSError as exc:
                        self.get_logger().warning(
                            f"cannot remove fully exported owned ray spool "
                            f"{spool_path}: {exc}"
                        )
        except RayObservationError as exc:
            self.get_logger().error(f"cannot inspect ray records-only spool: {exc}")

    def destroy_node(self):
        self._close_ray_recorder()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
