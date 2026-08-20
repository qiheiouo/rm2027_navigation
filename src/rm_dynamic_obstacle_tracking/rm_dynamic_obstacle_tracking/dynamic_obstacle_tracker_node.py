"""ROS wrapper for the shadow-only dynamic obstacle tracker."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rm_competition_interfaces.msg import (
    DynamicObstaclePrediction,
    DynamicObstaclePredictionArray,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    MultiObjectTracker,
    OccupancyMap,
    Point2D,
    TrackSnapshot,
    TrackState,
    cluster_points,
    dynamic_candidates,
    planar_rotation_matrix,
)


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DynamicObstacleTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_tracker_shadow")
        self._map_topic = self.declare_parameter("map_topic", "/map").value
        self._scan_topic = self.declare_parameter("scan_topic", "/local_scan").value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._markers_topic = self.declare_parameter(
            "markers_topic", "/perception/dynamic_obstacles_shadow/markers"
        ).value
        self._diagnostics_topic = self.declare_parameter(
            "diagnostics_topic", "/perception/dynamic_obstacles_shadow/diagnostics"
        ).value
        self._predictions_topic = self.declare_parameter(
            "predictions_topic", "/perception/dynamic_obstacles_shadow/predictions"
        ).value
        self._static_distance_threshold = float(
            self.declare_parameter("static_distance_threshold", 0.25).value
        )
        self._require_known_free = bool(
            self.declare_parameter("require_known_free", True).value
        )
        self._cluster_tolerance = float(
            self.declare_parameter("cluster_tolerance", 0.20).value
        )
        self._cluster_min_points = int(
            self.declare_parameter("cluster_min_points", 3).value
        )
        self._cluster_max_extent = float(
            self.declare_parameter("cluster_max_extent", 1.5).value
        )
        self._tf_timeout = float(self.declare_parameter("tf_timeout_sec", 0.08).value)
        self._max_scan_age = float(
            self.declare_parameter("max_scan_age_sec", 0.4).value
        )
        self._max_future_scan = float(
            self.declare_parameter("max_future_scan_sec", 0.05).value
        )
        self._show_tentative = bool(
            self.declare_parameter("show_tentative_tracks", True).value
        )
        self._marker_lifetime = float(
            self.declare_parameter("marker_lifetime_sec", 0.4).value
        )
        self._occupied_threshold = int(
            self.declare_parameter("occupied_threshold", 65).value
        )
        self._validate_parameters()

        self._prediction_dt = float(
            self.declare_parameter("prediction.dt", 0.1).value
        )
        self._prediction_steps = int(
            self.declare_parameter("prediction.steps", 15).value
        )
        self._prediction_max_tracks = int(
            self.declare_parameter("prediction.max_tracks", 64).value
        )
        if not 1 <= self._prediction_steps <= 1000:
            raise ValueError("prediction.steps must be in [1, 1000]")
        if not 1 <= self._prediction_max_tracks <= 256:
            raise ValueError("prediction.max_tracks must be in [1, 256]")
        self._tracker = MultiObjectTracker(
            association_gate=float(
                self.declare_parameter("tracker.association_gate", 0.60).value
            ),
            process_noise=float(
                self.declare_parameter("tracker.process_noise", 3.0).value
            ),
            measurement_noise=float(
                self.declare_parameter("tracker.measurement_noise", 0.08).value
            ),
            initial_variance=float(
                self.declare_parameter("tracker.initial_variance", 1.0).value
            ),
            min_hits_to_confirm=int(
                self.declare_parameter("tracker.min_hits_to_confirm", 3).value
            ),
            tentative_max_misses=int(
                self.declare_parameter("tracker.tentative_max_misses", 1).value
            ),
            max_coast_time_sec=float(
                self.declare_parameter("tracker.max_coast_time_sec", 0.6).value
            ),
            prediction_steps=self._prediction_steps,
            prediction_dt=self._prediction_dt,
            velocity_decay_tau=float(
                self.declare_parameter("prediction.velocity_decay_tau", 1.5).value
            ),
            max_prediction_speed=float(
                self.declare_parameter("prediction.max_speed", 3.0).value
            ),
        )
        self._map: OccupancyMap | None = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._markers_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )
        self._predictions_pub = self.create_publisher(
            DynamicObstaclePredictionArray, self._predictions_topic, 10
        )
        self.get_logger().info(
            "SHADOW ONLY dynamic tracker ready: %s + %s -> %s; "
            "it does not modify costmaps, plans, goals, TF, or cmd_vel"
            % (self._map_topic, self._scan_topic, self._predictions_topic)
        )

    def _validate_parameters(self) -> None:
        if (
            not self._map_topic
            or not self._scan_topic
            or not self._map_frame
            or not self._predictions_topic
            or self._static_distance_threshold < 0.0
        ):
            raise ValueError("map frame and static subtraction threshold are invalid")
        if (
            self._cluster_tolerance <= 0.0
            or self._cluster_min_points <= 0
            or self._cluster_max_extent <= 0.0
        ):
            raise ValueError("cluster parameters must be positive")
        if (
            self._tf_timeout < 0.0
            or self._max_scan_age < 0.0
            or self._max_future_scan < 0.0
            or self._marker_lifetime < 0.0
        ):
            raise ValueError("time limits must not be negative")
        if not 0 <= self._occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [0, 100]")

    def _on_map(self, message: OccupancyGrid) -> None:
        if message.header.frame_id != self._map_frame:
            self.get_logger().warning(
                "reject map frame %r; expected %r"
                % (message.header.frame_id, self._map_frame)
            )
            return
        orientation = message.info.origin.orientation
        yaw = _quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        try:
            self._map = OccupancyMap(
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                yaw,
                message.data,
                self._occupied_threshold,
            )
        except ValueError as error:
            self.get_logger().error(f"reject invalid occupancy map: {error}")

    def _on_scan(self, message: LaserScan) -> None:
        started = time.perf_counter()
        if self._map is None:
            self._publish_diagnostics(message, "waiting_for_map", 0, 0, 0, started)
            return
        if not message.header.frame_id:
            self._publish_diagnostics(message, "empty_scan_frame", 0, 0, 0, started)
            return
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0:
            self._publish_diagnostics(message, "invalid_scan_stamp", 0, 0, 0, started)
            return
        scan_age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        if self._max_future_scan > 0.0 and scan_age < -self._max_future_scan:
            self._publish_diagnostics(message, "future_scan", 0, 0, 0, started)
            return
        if self._max_scan_age > 0.0 and scan_age > self._max_scan_age:
            self._publish_diagnostics(message, "stale_scan", 0, 0, 0, started)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                message.header.frame_id,
                stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as error:
            self.get_logger().warning(f"drop scan without timestamped TF: {error}")
            self._publish_diagnostics(message, "tf_unavailable", 0, 0, 0, started)
            return

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        try:
            rotation_xx, rotation_xy, rotation_yx, rotation_yy = (
                planar_rotation_matrix(
                    (rotation.x, rotation.y, rotation.z, rotation.w)
                )
            )
        except ValueError as error:
            self.get_logger().warning(f"drop scan with invalid TF rotation: {error}")
            self._publish_diagnostics(
                message, "invalid_tf_rotation", 0, 0, 0, started
            )
            return
        endpoints: list[Point2D] = []
        angle = message.angle_min
        for scan_range in message.ranges:
            if (
                math.isfinite(scan_range)
                and message.range_min <= scan_range <= message.range_max
            ):
                scan_x = scan_range * math.cos(angle)
                scan_y = scan_range * math.sin(angle)
                endpoints.append(
                    Point2D(
                        translation.x
                        + rotation_xx * scan_x
                        + rotation_xy * scan_y,
                        translation.y
                        + rotation_yx * scan_x
                        + rotation_yy * scan_y,
                    )
                )
            angle += message.angle_increment

        candidates = dynamic_candidates(
            endpoints,
            self._map,
            self._static_distance_threshold,
            self._require_known_free,
        )
        detections = cluster_points(
            candidates,
            self._cluster_tolerance,
            self._cluster_min_points,
            self._cluster_max_extent,
        )
        update = self._tracker.update(detections, stamp.nanoseconds / 1.0e9)
        visible_tracks = [
            track
            for track in update.tracks
            if self._show_tentative or track.state != TrackState.TENTATIVE
        ]
        self._publish_predictions(message, list(update.tracks))
        self._publish_markers(message, visible_tracks)
        self._publish_diagnostics(
            message,
            "ok_time_reset" if update.time_reset else "ok",
            len(endpoints),
            len(candidates),
            len(detections),
            started,
            list(update.tracks),
            update.created,
            update.deleted,
        )

    def _publish_predictions(
        self, message: LaserScan, tracks: list[TrackSnapshot]
    ) -> None:
        output = DynamicObstaclePredictionArray()
        output.header.frame_id = self._map_frame
        output.header.stamp = message.header.stamp
        output.schema = DynamicObstaclePredictionArray.SCHEMA
        output.authority = DynamicObstaclePredictionArray.AUTHORITY_SHADOW_ONLY
        output.processing_stamp = self.get_clock().now().to_msg()
        output.prediction_dt = self._prediction_dt
        output.prediction_steps = self._prediction_steps
        output.total_track_count = len(tracks)
        output.complete = len(tracks) <= self._prediction_max_tracks
        state_values = {
            TrackState.TENTATIVE: DynamicObstaclePrediction.STATE_TENTATIVE,
            TrackState.CONFIRMED: DynamicObstaclePrediction.STATE_CONFIRMED,
            TrackState.COASTING: DynamicObstaclePrediction.STATE_COASTING,
        }
        ordered_tracks = sorted(
            tracks,
            key=lambda track: (
                {
                    TrackState.CONFIRMED: 0,
                    TrackState.COASTING: 1,
                    TrackState.TENTATIVE: 2,
                }[track.state],
                track.track_id,
            ),
        )
        for track in ordered_tracks[: self._prediction_max_tracks]:
            prediction = DynamicObstaclePrediction()
            prediction.track_id = track.track_id
            prediction.state = state_values[track.state]
            prediction.position.x = track.position.x
            prediction.position.y = track.position.y
            prediction.velocity.x = track.velocity.x
            prediction.velocity.y = track.velocity.y
            prediction.size.x = track.size_x
            prediction.size.y = track.size_y
            prediction.last_observation_stamp = Time(
                nanoseconds=round(track.last_observation_timestamp * 1.0e9)
            ).to_msg()
            prediction.observation_count = track.observations
            prediction.miss_count = track.misses
            prediction.prediction = [
                Point(x=point.x, y=point.y, z=0.0) for point in track.prediction
            ]
            output.tracks.append(prediction)
        self._predictions_pub.publish(output)

    def _publish_markers(
        self, message: LaserScan, tracks: list[TrackSnapshot]
    ) -> None:
        markers = MarkerArray()
        reset = Marker()
        reset.header.frame_id = self._map_frame
        reset.header.stamp = message.header.stamp
        reset.action = Marker.DELETEALL
        markers.markers.append(reset)
        for track in tracks:
            markers.markers.extend(self._markers_for_track(message, track))
        self._markers_pub.publish(markers)

    def _markers_for_track(
        self, message: LaserScan, track: TrackSnapshot
    ) -> list[Marker]:
        color = {
            TrackState.TENTATIVE: (1.0, 0.75, 0.1),
            TrackState.CONFIRMED: (0.1, 0.9, 0.25),
            TrackState.COASTING: (1.0, 0.35, 0.1),
        }.get(track.state, (0.5, 0.5, 0.5))

        box = self._new_marker(message, "track_box", track.track_id, Marker.CUBE)
        box.pose.position.x = track.position.x
        box.pose.position.y = track.position.y
        box.pose.position.z = 0.35
        box.pose.orientation.w = 1.0
        box.scale.x = max(track.size_x, 0.12)
        box.scale.y = max(track.size_y, 0.12)
        box.scale.z = 0.70
        box.color.r, box.color.g, box.color.b = color
        box.color.a = 0.45

        velocity = self._new_marker(
            message, "track_velocity", track.track_id, Marker.ARROW
        )
        velocity.points = [
            Point(x=track.position.x, y=track.position.y, z=0.75),
            Point(
                x=track.position.x + track.velocity.x,
                y=track.position.y + track.velocity.y,
                z=0.75,
            ),
        ]
        velocity.scale.x = 0.035
        velocity.scale.y = 0.08
        velocity.scale.z = 0.08
        velocity.color.r, velocity.color.g, velocity.color.b = color
        velocity.color.a = 0.95

        prediction = self._new_marker(
            message, "track_prediction", track.track_id, Marker.LINE_STRIP
        )
        prediction.points = [
            Point(x=track.position.x, y=track.position.y, z=0.70),
            *[Point(x=point.x, y=point.y, z=0.70) for point in track.prediction],
        ]
        prediction.scale.x = 0.035
        prediction.color.r, prediction.color.g, prediction.color.b = color
        prediction.color.a = 0.8

        label = self._new_marker(
            message,
            "track_label",
            track.track_id,
            Marker.TEXT_VIEW_FACING,
        )
        label.pose.position.x = track.position.x
        label.pose.position.y = track.position.y
        label.pose.position.z = 1.1
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        speed = math.hypot(track.velocity.x, track.velocity.y)
        label.text = (
            f"#{track.track_id} {track.state.value} v={speed:.2f} "
            f"hits={track.observations} miss={track.misses}"
        )
        return [box, velocity, prediction, label]

    def _new_marker(
        self, message: LaserScan, namespace: str, marker_id: int, marker_type: int
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = message.header.stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=self._marker_lifetime).to_msg()
        return marker

    def _publish_diagnostics(
        self,
        message: LaserScan,
        state: str,
        endpoint_count: int,
        candidate_count: int,
        detection_count: int,
        started: float,
        tracks: list[TrackSnapshot] | None = None,
        created: int = 0,
        deleted: int = 0,
    ) -> None:
        tracks = tracks or []
        status = DiagnosticStatus()
        status.name = "dynamic_obstacle_tracking_shadow"
        status.hardware_id = "software_shadow"
        status.level = (
            DiagnosticStatus.OK if state.startswith("ok") else DiagnosticStatus.WARN
        )
        status.message = state
        values = {
            "authority": "shadow_only",
            "input_scan": self._scan_topic,
            "input_map": self._map_topic,
            "scan_frame": message.header.frame_id,
            "endpoints": endpoint_count,
            "dynamic_candidates": candidate_count,
            "detections": detection_count,
            "tracks": len(tracks),
            "confirmed": sum(track.state == TrackState.CONFIRMED for track in tracks),
            "coasting": sum(track.state == TrackState.COASTING for track in tracks),
            "created": created,
            "deleted": deleted,
            "latency_ms": f"{(time.perf_counter() - started) * 1000.0:.3f}",
        }
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        array.status.extend(self._track_diagnostics(tracks))
        self._diagnostics_pub.publish(array)

    @staticmethod
    def _track_diagnostics(
        tracks: list[TrackSnapshot],
    ) -> list[DiagnosticStatus]:
        results: list[DiagnosticStatus] = []
        for track in tracks:
            status = DiagnosticStatus()
            status.name = f"dynamic_obstacle_tracking_shadow/track/{track.track_id}"
            status.hardware_id = "software_shadow"
            status.level = DiagnosticStatus.OK
            status.message = track.state.value
            fields = {
                "track_id": track.track_id,
                "state": track.state.value,
                "timestamp": f"{track.timestamp:.9f}",
                "last_observation_timestamp": (
                    f"{track.last_observation_timestamp:.9f}"
                ),
                "age_sec": f"{track.age_sec:.3f}",
                "position_x": f"{track.position.x:.6f}",
                "position_y": f"{track.position.y:.6f}",
                "velocity_x": f"{track.velocity.x:.6f}",
                "velocity_y": f"{track.velocity.y:.6f}",
                "size_x": f"{track.size_x:.3f}",
                "size_y": f"{track.size_y:.3f}",
                "observations": track.observations,
                "misses": track.misses,
                "prediction_points": len(track.prediction),
            }
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in fields.items()
            ]
            results.append(status)
        return results


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
