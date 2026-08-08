"""Shadow-only ROS node for localization integrity observations."""

from __future__ import annotations

from collections import deque
import copy
import json
import math
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .core import (
    IntegrityMetrics,
    IntegrityState,
    IntegrityThresholds,
    OccupancyDistanceField,
    Pose2D,
    Transform3D,
    correction_from_global_and_odom,
    evaluate_integrity,
    metrics_record,
    pose_delta,
    quaternion_to_yaw,
    scan_map_metrics,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _pose_2d(pose: Pose) -> Pose2D:
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=quaternion_to_yaw(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        ),
    )


def _pose_transform(pose: Pose) -> Transform3D:
    return Transform3D(
        translation=(pose.position.x, pose.position.y, pose.position.z),
        rotation_xyzw=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _transform_message(message) -> Transform3D:
    return Transform3D(
        translation=(
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        ),
        rotation_xyzw=(
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        ),
    )


class LocalizationIntegrityNode(Node):
    """Observe localization evidence without taking authority over it."""

    def __init__(self) -> None:
        super().__init__("localization_integrity")
        self._mode = str(self.declare_parameter("mode", "shadow").value).lower()
        if self._mode != "shadow":
            raise ValueError("the first integrity implementation supports shadow mode only")

        self._profile = str(self.declare_parameter("profile", "generic").value)
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self._base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self._map_topic = str(self.declare_parameter("map_topic", "/map").value)
        self._scan_topic = str(
            self.declare_parameter("scan_topic", "/localization/scan").value
        )
        self._global_pose_topic = str(
            self.declare_parameter(
                "global_pose_topic", "/localization/global_pose"
            ).value
        )
        self._odom_topic = str(
            self.declare_parameter("odom_topic", "/odometry/lio").value
        )
        self._diagnostics_topic = str(
            self.declare_parameter("diagnostics_topic", "/diagnostics").value
        )
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 2.0).value
        )
        self._tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.05).value
        )
        self._scan_cache_size = int(self.declare_parameter("scan_cache_size", 30).value)
        self._odom_cache_size = int(self.declare_parameter("odom_cache_size", 500).value)
        self._occupied_threshold = int(
            self.declare_parameter("occupied_threshold", 65).value
        )
        self._agreement_distance_m = float(
            self.declare_parameter("agreement_distance_m", 0.20).value
        )
        self._residual_cap_m = float(
            self.declare_parameter("residual_cap_m", 2.0).value
        )
        self._max_scan_points = int(
            self.declare_parameter("max_scan_points", 720).value
        )
        self._metrics_output_path = str(
            self.declare_parameter("metrics_output_path", "").value
        ).strip()
        self._thresholds = IntegrityThresholds(
            max_global_pose_age_sec=float(
                self.declare_parameter("limits.max_global_pose_age_sec", 0.5).value
            ),
            max_scan_age_sec=float(
                self.declare_parameter("limits.max_scan_age_sec", 0.5).value
            ),
            max_pose_scan_dt_sec=float(
                self.declare_parameter("limits.max_pose_scan_dt_sec", 0.15).value
            ),
            max_pose_odom_dt_sec=float(
                self.declare_parameter("limits.max_pose_odom_dt_sec", 0.05).value
            ),
            max_tf_scan_dt_sec=float(
                self.declare_parameter("limits.max_tf_scan_dt_sec", 0.05).value
            ),
            min_valid_scan_points=int(
                self.declare_parameter("limits.min_valid_scan_points", 30).value
            ),
            min_scan_map_agreement_warn=float(
                self.declare_parameter("limits.min_scan_map_agreement_warn", 0.45).value
            ),
            min_scan_map_agreement_reject=float(
                self.declare_parameter("limits.min_scan_map_agreement_reject", 0.20).value
            ),
            max_scan_map_mean_residual_warn_m=float(
                self.declare_parameter(
                    "limits.max_scan_map_mean_residual_warn_m", 0.30
                ).value
            ),
            max_global_translation_jump_warn_m=float(
                self.declare_parameter(
                    "limits.max_global_translation_jump_warn_m", 0.30
                ).value
            ),
            max_global_translation_jump_reject_m=float(
                self.declare_parameter(
                    "limits.max_global_translation_jump_reject_m", 0.75
                ).value
            ),
            max_global_yaw_jump_warn_rad=float(
                self.declare_parameter("limits.max_global_yaw_jump_warn_rad", 0.60).value
            ),
            max_global_yaw_jump_reject_rad=float(
                self.declare_parameter(
                    "limits.max_global_yaw_jump_reject_rad", 1.20
                ).value
            ),
            max_correction_translation_jump_warn_m=float(
                self.declare_parameter(
                    "limits.max_correction_translation_jump_warn_m", 0.30
                ).value
            ),
            max_correction_translation_jump_reject_m=float(
                self.declare_parameter(
                    "limits.max_correction_translation_jump_reject_m", 0.75
                ).value
            ),
            max_correction_yaw_jump_warn_rad=float(
                self.declare_parameter(
                    "limits.max_correction_yaw_jump_warn_rad", 0.60
                ).value
            ),
            max_correction_yaw_jump_reject_rad=float(
                self.declare_parameter(
                    "limits.max_correction_yaw_jump_reject_rad", 1.20
                ).value
            ),
            max_correction_translation_rate_warn_mps=float(
                self.declare_parameter(
                    "limits.max_correction_translation_rate_warn_mps", 1.50
                ).value
            ),
            max_correction_yaw_rate_warn_rps=float(
                self.declare_parameter(
                    "limits.max_correction_yaw_rate_warn_rps", 3.00
                ).value
            ),
        )
        self._validate_parameters()

        self._distance_field = None
        self._scan_cache = deque(maxlen=self._scan_cache_size)
        self._odom_cache = deque(maxlen=self._odom_cache_size)
        self._previous_global = None
        self._previous_global_stamp = None
        self._previous_correction = None
        self._previous_correction_stamp = None
        self._last_odom = None
        self._last_odom_stamp = None
        self._last_odom_step = (None, None)
        self._odom_time_reset = False
        self._latest_metrics = IntegrityMetrics(profile=self._profile)
        self._latest_pose_stamp = None
        self._latest_scan_stamp = None
        self._previous_scan_stamp = None
        self._latest_scan_rate_hz = None
        self._latest_scan_nan_ratio = None
        self._latest_scan_infinite_ratio = None
        self._latest_scan_out_of_range_ratio = None
        self._scan_time_reset = False
        self._metrics_file = None
        self._last_record_signature = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 100)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._global_pose_topic,
            self._on_global_pose,
            10,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            lambda: self._publish(record_event=False),
        )
        self.get_logger().info(
            "Localization integrity SHADOW ready. It publishes diagnostics only; "
            "it owns no TF, localization pose, navigation goal, or chassis command."
        )

    def _validate_parameters(self) -> None:
        if self._publish_rate_hz <= 0.0 or self._tf_timeout_sec < 0.0:
            raise ValueError("publish rate must be positive and TF timeout non-negative")
        if self._scan_cache_size <= 0 or self._odom_cache_size <= 0:
            raise ValueError("cache sizes must be positive")
        if self._max_scan_points <= 0 or self._agreement_distance_m < 0.0:
            raise ValueError("scan metric parameters are invalid")
        if self._residual_cap_m <= 0.0:
            raise ValueError("residual_cap_m must be positive")
        if not 0 <= self._occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [0, 100]")
        if self._thresholds.min_valid_scan_points <= 0:
            raise ValueError("min_valid_scan_points must be positive")
        if (
            not 0.0
            <= self._thresholds.min_scan_map_agreement_reject
            <= self._thresholds.min_scan_map_agreement_warn
            <= 1.0
        ):
            raise ValueError("scan-map agreement thresholds must satisfy 0 <= reject <= warn <= 1")
        ordered_limits = (
            (
                self._thresholds.max_global_translation_jump_warn_m,
                self._thresholds.max_global_translation_jump_reject_m,
            ),
            (
                self._thresholds.max_global_yaw_jump_warn_rad,
                self._thresholds.max_global_yaw_jump_reject_rad,
            ),
            (
                self._thresholds.max_correction_translation_jump_warn_m,
                self._thresholds.max_correction_translation_jump_reject_m,
            ),
            (
                self._thresholds.max_correction_yaw_jump_warn_rad,
                self._thresholds.max_correction_yaw_jump_reject_rad,
            ),
        )
        if any(warn < 0.0 or reject < warn for warn, reject in ordered_limits):
            raise ValueError("jump thresholds must satisfy 0 <= warn <= reject")
        nonnegative_limits = (
            self._thresholds.max_global_pose_age_sec,
            self._thresholds.max_scan_age_sec,
            self._thresholds.max_pose_scan_dt_sec,
            self._thresholds.max_pose_odom_dt_sec,
            self._thresholds.max_tf_scan_dt_sec,
            self._thresholds.max_scan_map_mean_residual_warn_m,
            self._thresholds.max_correction_translation_rate_warn_mps,
            self._thresholds.max_correction_yaw_rate_warn_rps,
        )
        if any(value < 0.0 for value in nonnegative_limits):
            raise ValueError("age, residual, and rate limits must not be negative")

    def _on_map(self, message: OccupancyGrid) -> None:
        if message.header.frame_id and message.header.frame_id != self._map_frame:
            self.get_logger().error(
                f"Ignoring map frame {message.header.frame_id!r}; expected {self._map_frame!r}"
            )
            self._distance_field = None
            return
        try:
            origin = Pose2D(
                x=message.info.origin.position.x,
                y=message.info.origin.position.y,
                yaw=quaternion_to_yaw(
                    (
                        message.info.origin.orientation.x,
                        message.info.origin.orientation.y,
                        message.info.origin.orientation.z,
                        message.info.origin.orientation.w,
                    )
                ),
            )
            self._distance_field = OccupancyDistanceField(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin=origin,
                data=message.data,
                occupied_threshold=self._occupied_threshold,
            )
            self.get_logger().info(
                f"Built scan-map distance field {message.info.width}x{message.info.height}"
            )
        except ValueError as error:
            self._distance_field = None
            self.get_logger().error(f"Cannot build map distance field: {error}")

    def _on_scan(self, message: LaserScan) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        if stamp > 0.0:
            if self._previous_scan_stamp is not None and stamp <= self._previous_scan_stamp:
                self._scan_cache.clear()
                self._scan_time_reset = True
            elif self._previous_scan_stamp is not None:
                self._latest_scan_rate_hz = 1.0 / (stamp - self._previous_scan_stamp)
            self._previous_scan_stamp = stamp
            point_count = len(message.ranges)
            if point_count > 0:
                nan_count = sum(1 for value in message.ranges if math.isnan(value))
                infinite_count = sum(
                    1 for value in message.ranges if math.isinf(value)
                )
                out_of_range_count = sum(
                    1
                    for value in message.ranges
                    if math.isfinite(value)
                    and (value < message.range_min or value > message.range_max)
                )
                self._latest_scan_nan_ratio = nan_count / point_count
                self._latest_scan_infinite_ratio = infinite_count / point_count
                self._latest_scan_out_of_range_ratio = (
                    out_of_range_count / point_count
                )
            self._scan_cache.append((stamp, message))
            self._latest_scan_stamp = stamp

    def _on_odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self._odom_frame
            or message.child_frame_id != self._base_frame
        ):
            return
        stamp = _stamp_seconds(message.header.stamp)
        if stamp <= 0.0:
            return
        try:
            pose = _pose_2d(message.pose.pose)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid odometry pose: {error}")
            return
        if self._last_odom_stamp is not None and stamp <= self._last_odom_stamp:
            self._odom_cache.clear()
            self._odom_time_reset = True
            self._last_odom_step = (None, None)
        elif self._last_odom is not None:
            self._last_odom_step = pose_delta(pose, self._last_odom)
        self._last_odom = pose
        self._last_odom_stamp = stamp
        self._odom_cache.append((stamp, pose))

    def _on_global_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self._map_frame:
            return
        stamp = _stamp_seconds(message.header.stamp)
        if stamp <= 0.0:
            return
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        metrics = IntegrityMetrics(
            stamp_sec=stamp,
            profile=self._profile,
            map_ready=self._distance_field is not None,
            global_pose_ready=True,
            global_pose_age_sec=now_sec - stamp,
            scan_time_reset=self._scan_time_reset,
            odom_time_reset=self._odom_time_reset,
            odom_translation_step_m=self._last_odom_step[0],
            odom_yaw_step_rad=self._last_odom_step[1],
            scan_rate_hz=self._latest_scan_rate_hz,
            scan_nan_ratio=self._latest_scan_nan_ratio,
            scan_infinite_ratio=self._latest_scan_infinite_ratio,
            scan_out_of_range_ratio=self._latest_scan_out_of_range_ratio,
        )
        self._scan_time_reset = False
        self._odom_time_reset = False
        try:
            current_global = _pose_2d(message.pose.pose)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid global pose: {error}")
            return
        time_reset = (
            self._previous_global_stamp is not None
            and stamp <= self._previous_global_stamp
        )
        metrics.global_pose_time_reset = bool(time_reset)
        if time_reset:
            self._previous_global = None
            self._previous_global_stamp = None
            self._previous_correction = None
            self._previous_correction_stamp = None
        if self._previous_global is not None:
            (
                metrics.global_translation_jump_m,
                metrics.global_yaw_jump_rad,
            ) = pose_delta(current_global, self._previous_global)

        odom_sample = self._nearest(self._odom_cache, stamp)
        if odom_sample is not None:
            odom_stamp, odom_pose = odom_sample
            metrics.odom_ready = True
            metrics.pose_odom_dt_sec = abs(stamp - odom_stamp)
            correction = correction_from_global_and_odom(current_global, odom_pose)
            metrics.correction_translation_m = math.hypot(correction.x, correction.y)
            metrics.correction_yaw_rad = abs(correction.yaw)
            if self._previous_correction is not None:
                translation, yaw = pose_delta(correction, self._previous_correction)
                metrics.correction_translation_jump_m = translation
                metrics.correction_yaw_jump_rad = yaw
                dt = stamp - self._previous_correction_stamp
                if dt > 1.0e-9:
                    metrics.correction_translation_rate_mps = translation / dt
                    metrics.correction_yaw_rate_rps = yaw / dt
            self._previous_correction = correction
            self._previous_correction_stamp = stamp

        scan_sample = self._nearest(self._scan_cache, stamp)
        if scan_sample is not None:
            scan_stamp, scan = scan_sample
            metrics.scan_ready = True
            metrics.scan_age_sec = now_sec - scan_stamp
            metrics.pose_scan_dt_sec = abs(stamp - scan_stamp)
            metrics.valid_scan_points = sum(
                1
                for value in scan.ranges
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max
            )
            self._latest_scan_stamp = scan_stamp
            base_to_scan, tf_dt = self._lookup_base_to_scan(scan)
            if base_to_scan is not None:
                metrics.tf_ready = True
                metrics.tf_scan_dt_sec = tf_dt
                if (
                    self._distance_field is not None
                    and metrics.pose_scan_dt_sec
                    <= self._thresholds.max_pose_scan_dt_sec
                ):
                    try:
                        agreement = scan_map_metrics(
                            ranges=scan.ranges,
                            angle_min=scan.angle_min,
                            angle_increment=scan.angle_increment,
                            range_min=scan.range_min,
                            range_max=scan.range_max,
                            map_to_base=_pose_transform(message.pose.pose),
                            base_to_scan=base_to_scan,
                            distance_field=self._distance_field,
                            agreement_distance_m=self._agreement_distance_m,
                            residual_cap_m=self._residual_cap_m,
                            max_scan_points=self._max_scan_points,
                        )
                    except ValueError as error:
                        self.get_logger().warning(
                            f"Cannot compute scan-map evidence: {error}"
                        )
                        agreement = None
                    if agreement is not None:
                        metrics.valid_scan_points = agreement.valid_point_count
                        metrics.scan_points_in_map = agreement.points_in_map
                        metrics.scan_map_agreement = agreement.agreement
                        metrics.scan_map_mean_residual_m = agreement.mean_residual_m
                        metrics.scan_map_p95_residual_m = agreement.p95_residual_m
                        metrics.scan_map_ready = agreement.agreement is not None

        self._previous_global = current_global
        self._previous_global_stamp = stamp
        self._latest_pose_stamp = stamp
        self._latest_metrics = metrics
        self._publish(record_event=True)

    def _lookup_base_to_scan(self, scan: LaserScan):
        if scan.header.frame_id == self._base_frame:
            return Transform3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), 0.0
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Timestamped TF {self._base_frame} <- {scan.header.frame_id} failed: {error}"
            )
            return None, None
        transform_stamp = _stamp_seconds(transform.header.stamp)
        scan_stamp = _stamp_seconds(scan.header.stamp)
        tf_dt = 0.0 if transform_stamp <= 0.0 else abs(transform_stamp - scan_stamp)
        return _transform_message(transform), tf_dt

    @staticmethod
    def _nearest(cache, stamp: float):
        if not cache:
            return None
        return min(cache, key=lambda sample: abs(sample[0] - stamp))

    def _publish(self, record_event: bool) -> None:
        metrics = copy.copy(self._latest_metrics)
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        metrics.map_ready = self._distance_field is not None
        if self._latest_pose_stamp is not None:
            metrics.global_pose_ready = True
            metrics.global_pose_age_sec = now_sec - self._latest_pose_stamp
        if self._latest_scan_stamp is not None:
            metrics.scan_ready = True
            metrics.scan_age_sec = now_sec - self._latest_scan_stamp
        result = evaluate_integrity(metrics, self._thresholds)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "navigation_integrity/localization"
        status.hardware_id = self._profile
        status.level = {
            IntegrityState.GOOD: DiagnosticStatus.OK,
            IntegrityState.SUSPECT: DiagnosticStatus.WARN,
            IntegrityState.REJECT: DiagnosticStatus.ERROR,
            IntegrityState.UNKNOWN: DiagnosticStatus.STALE,
        }[result.state]
        reason_text = ",".join(result.reasons) if result.reasons else "NONE"
        status.message = f"{result.state.value}: {reason_text} (shadow only)"
        record = metrics_record(metrics, result)
        status.values = [
            KeyValue(key="mode", value=self._mode),
            KeyValue(key="state", value=result.state.value),
            KeyValue(key="reason_codes", value=reason_text),
        ]
        for key, value in record["metrics"].items():
            status.values.append(KeyValue(key=key, value=self._format_value(value)))
        array.status.append(status)
        self._diagnostics_pub.publish(array)
        signature = (result.state.value, result.reasons)
        if record_event or signature != self._last_record_signature:
            self._write_record(record)
            self._last_record_signature = signature

    @staticmethod
    def _format_value(value) -> str:
        if value is None:
            return "unavailable"
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, float):
            return f"{value:.9g}"
        return str(value)

    def _write_record(self, record: dict[str, object]) -> None:
        if not self._metrics_output_path:
            return
        try:
            if self._metrics_file is None:
                path = Path(self._metrics_output_path).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                self._metrics_file = path.open("a", encoding="utf-8", newline="\n")
            self._metrics_file.write(json.dumps(record, sort_keys=True) + "\n")
            self._metrics_file.flush()
        except OSError as error:
            self.get_logger().error(
                f"Disabling integrity JSONL output after write failure: {error}"
            )
            if self._metrics_file is not None:
                self._metrics_file.close()
                self._metrics_file = None
            self._metrics_output_path = ""

    def destroy_node(self):
        if self._metrics_file is not None:
            self._metrics_file.close()
            self._metrics_file = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationIntegrityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
