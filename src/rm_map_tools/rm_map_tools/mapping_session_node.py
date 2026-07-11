from __future__ import annotations

import math
import threading
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

from .map_export import write_candidate_map_bundle


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
        self._max_voxels = int(
            self.declare_parameter("max_voxels", 2000000).value
        )
        self._min_observations = int(
            self.declare_parameter("min_observations", 2).value
        )
        self._recording = bool(self.declare_parameter("autostart", True).value)

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

        self._lock = threading.Lock()
        self._voxels: dict[tuple[int, int, int], tuple[np.ndarray, int]] = {}
        self._latest_occupancy: OccupancyGrid | None = None
        self._last_sample_ns = -1
        self._accepted_clouds = 0
        self._dropped_tf = 0

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

        self.create_service(Trigger, "/mapping/start", self._start)
        self.create_service(Trigger, "/mapping/stop", self._stop)
        self.create_service(Trigger, "/mapping/reset", self._reset)
        self.create_service(Trigger, "/mapping/save", self._save)
        self._octomap_reset_client = self.create_client(Empty, "/octomap_server/reset")
        self.create_timer(5.0, self._report_progress)
        self._publish_recording_state()
        self.get_logger().info(
            "mapping session ready: cloud=%s occupancy=%s output_root=%s autostart=%s"
            % (self._cloud_topic, self._occupancy_topic, self._output_root, self._recording)
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

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
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

        with self._lock:
            normalized_keys = [
                (int(key[0]), int(key[1]), int(key[2])) for key in unique_keys
            ]
            additional_voxels = sum(
                1 for key in normalized_keys if key not in self._voxels
            )
            projected_size = len(self._voxels) + additional_voxels
            if projected_size > self._max_voxels:
                self._recording = False
                self.get_logger().error(
                    "mapping stopped before exceeding max_voxels=%d; save or reset the session"
                    % self._max_voxels
                )
                return
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
            self._last_sample_ns = stamp_ns

    def _start(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._recording = True
        self._publish_recording_state()
        response.success = True
        response.message = "mapping recording enabled"
        return response

    def _stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._recording = False
        self._publish_recording_state()
        response.success = True
        response.message = "mapping recording paused"
        return response

    def _reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._octomap_reset_client.service_is_ready():
            response.success = False
            response.message = "octomap reset service is not ready; session was not changed"
            return response
        self._recording = False
        self._publish_recording_state()
        with self._lock:
            self._voxels.clear()
            self._latest_occupancy = None
            self._last_sample_ns = -1
            self._accepted_clouds = 0
            self._dropped_tf = 0
        self._octomap_reset_client.call_async(Empty.Request())
        response.success = True
        response.message = "mapping session and OctoMap reset requested; recording is paused"
        return response

    def _publish_recording_state(self) -> None:
        msg = Bool()
        msg.data = self._recording
        self._recording_publisher.publish(msg)

    def _save(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
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

        try:
            manifest = write_candidate_map_bundle(
                output_root=self._output_root,
                map_id=self._map_id,
                revision=revision,
                points=points,
                occupancy_values=occupancy.data,
                occupancy_width=occupancy.info.width,
                occupancy_height=occupancy.info.height,
                occupancy_resolution=occupancy.info.resolution,
                occupancy_origin=[origin.x, origin.y, yaw],
                source_method="registered_pointcloud_plus_octomap_projection",
                source_details={
                    "registered_cloud_topic": self._cloud_topic,
                    "occupancy_topic": self._occupancy_topic,
                    "voxel_size": self._voxel_size,
                    "sample_period_sec": self._sample_period,
                    "min_observations": self._min_observations,
                    "accepted_clouds": accepted_clouds,
                    "dropped_tf_clouds": dropped_tf,
                },
            )
        except Exception as exc:
            self.get_logger().error(f"map bundle export failed: {exc}")
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = str(manifest)
        self.get_logger().info(f"candidate map bundle saved: {manifest}")
        return response

    def _report_progress(self) -> None:
        with self._lock:
            voxel_count = len(self._voxels)
            cloud_count = self._accepted_clouds
            has_occupancy = self._latest_occupancy is not None
        self.get_logger().info(
            "mapping progress: recording=%s clouds=%d voxels=%d occupancy=%s tf_drops=%d"
            % (
                self._recording,
                cloud_count,
                voxel_count,
                has_occupancy,
                self._dropped_tf,
            )
        )


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
