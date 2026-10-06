"""ROS wrapper for the shadow-only dynamic obstacle tracker."""

from __future__ import annotations

import math
import time
import hashlib
from dataclasses import asdict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
    cluster_detections_with_members,
    dynamic_candidates,
    filter_detections_near_static,
    planar_rotation_matrix,
)
from .surface_member_evidence import SurfaceMemberEvidence, encoded_range


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
        self._detection_static_distance_threshold = float(
            self.declare_parameter(
                "detection_static_distance_threshold", 0.0
            ).value
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

        evidence_directory = self.declare_parameter("surface_evidence_directory", "").value
        if type(evidence_directory) is not str:
            raise ValueError("surface_evidence_directory must be a string")

        self._prediction_dt = float(
            self.declare_parameter("prediction.dt", 0.1).value
        )
        self._prediction_steps = int(
            self.declare_parameter("prediction.steps", 15).value
        )
        self._prediction_max_tracks = int(
            self.declare_parameter("prediction.max_tracks", 64).value
        )
        self._anchor_mode = self.declare_parameter(
            "prediction.anchor_mode", "filtered"
        ).value
        schemas = {
            "filtered": DynamicObstaclePredictionArray.SCHEMA,
            "last_observation_cv": DynamicObstaclePredictionArray.SCHEMA_OBSERVATION_ANCHOR,
        }
        if self._anchor_mode not in schemas:
            raise ValueError("unsupported prediction.anchor_mode")
        self._prediction_schema = schemas[self._anchor_mode]
        self._observation_stamp_ns: dict[int, int] = {}
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
            min_displacement_to_confirm=float(
                self.declare_parameter(
                    "tracker.min_displacement_to_confirm", 0.0
                ).value
            ),
            use_global_assignment=bool(
                self.declare_parameter("tracker.use_global_assignment", False).value
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
            public_anchor_mode=self._anchor_mode,
        )
        self._surface_member_evidence = (
            SurfaceMemberEvidence(evidence_directory) if evidence_directory else None
        )
        self._surface_map_identity = None
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
            or self._detection_static_distance_threshold < 0.0
        ):
            raise ValueError("cluster and detection filter parameters are invalid")
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
            if self._surface_member_evidence is not None:
                self._surface_map_identity = {
                    "frame": message.header.frame_id,
                    "stamp_ns": Time.from_msg(message.header.stamp).nanoseconds,
                    "size": [message.info.width, message.info.height],
                    "resolution": message.info.resolution,
                    "origin_xy": [message.info.origin.position.x, message.info.origin.position.y],
                    "origin_z": message.info.origin.position.z,
                    "origin_quaternion": [orientation.x, orientation.y, orientation.z, orientation.w],
                    "signed_int8_data_sha256": hashlib.sha256(bytes(v & 255 for v in message.data)).hexdigest(),
                }
                evidence = self._surface_member_evidence
                if evidence.complete and not evidence.write({
                        "status": "map", "identity": self._surface_map_identity,
                        "data": list(message.data),
                        "capture_stamp_ns": self.get_clock().now().nanoseconds}):
                    self.get_logger().error("surface map evidence incomplete: " + evidence.failed_reason)
        except ValueError as error:
            self.get_logger().error(f"reject invalid occupancy map: {error}")

    def _on_scan(self, message: LaserScan) -> None:
        started = time.perf_counter()
        evidence = self._surface_member_evidence
        capture = evidence is not None and evidence.complete
        callback_stamp_ns = self.get_clock().now().nanoseconds if capture else None
        if capture and len(message.ranges) > evidence.MAX_SCAN_BEAMS:
            evidence.fail("complete scan beam budget exceeded")
            capture = False
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
        source_indices = [] if capture else None
        angle = message.angle_min
        for beam_index, scan_range in enumerate(message.ranges):
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
                if source_indices is not None:
                    source_indices.append(beam_index)
            angle += message.angle_increment

        candidates = dynamic_candidates(
            endpoints,
            self._map,
            self._static_distance_threshold,
            self._require_known_free,
        )
        members = None
        if capture:
            members = cluster_detections_with_members(
                candidates, self._cluster_tolerance, self._cluster_min_points,
                self._cluster_max_extent,
            )
            detections = [d for d, _ in members]
        else:
            detections = cluster_points(
                candidates, self._cluster_tolerance, self._cluster_min_points,
                self._cluster_max_extent,
            )
        detections = filter_detections_near_static(
            detections,
            self._map,
            self._detection_static_distance_threshold,
        )
        if capture:
            retained = {id(d) for d in detections}
            members = [(d, indices) for d, indices in members if id(d) in retained]
        update = self._tracker.update(detections, stamp.nanoseconds / 1.0e9,
            source_stamp_ns=stamp.nanoseconds if self._anchor_mode == "last_observation_cv" else None,
            capture_assignments=capture)
        visible_tracks = [
            track
            for track in update.tracks
            if self._show_tentative or track.state != TrackState.TENTATIVE
        ]
        public = self._publish_predictions(message, list(update.tracks))
        if capture:
            beam_by_point = {id(p): i for p, i in zip(endpoints, source_indices)}
            record = {
                "status": "accepted", "source_stamp_ns": stamp.nanoseconds,
                "callback_stamp_ns": callback_stamp_ns,
                "capture_stamp_ns": self.get_clock().now().nanoseconds,
                "scan_frame": message.header.frame_id, "map_identity": self._surface_map_identity,
                "extraction_parameters": {"occupied_threshold": self._occupied_threshold,
                    "static_distance_threshold": self._static_distance_threshold,
                    "require_known_free": self._require_known_free,
                    "cluster_tolerance": self._cluster_tolerance,
                    "cluster_min_points": self._cluster_min_points,
                    "cluster_max_extent": self._cluster_max_extent,
                    "detection_static_distance_threshold": self._detection_static_distance_threshold},
                "tracker_parameters": {name: getattr(self._tracker, name) for name in (
                    "association_gate", "process_noise", "measurement_noise", "initial_variance",
                    "min_hits_to_confirm", "min_displacement_to_confirm", "use_global_assignment",
                    "tentative_max_misses", "max_coast_time_sec", "prediction_steps", "prediction_dt",
                    "velocity_decay_tau", "max_prediction_speed", "max_update_dt", "public_anchor_mode")},
                "prediction_max_tracks": self._prediction_max_tracks,
                "scan": {"angle_min": message.angle_min, "angle_increment": message.angle_increment,
                         "range_min": message.range_min, "range_max": message.range_max,
                         "time_increment": message.time_increment, "scan_time": message.scan_time,
                         "ranges": [encoded_range(v) for v in message.ranges]},
                "source_tf": {"requested_stamp_ns": stamp.nanoseconds,
                              "returned_stamp_ns": Time.from_msg(transform.header.stamp).nanoseconds,
                              "frame": transform.header.frame_id, "child_frame": transform.child_frame_id,
                              "translation": [translation.x, translation.y, translation.z],
                              "quaternion": [rotation.x, rotation.y, rotation.z, rotation.w],
                              "used_xy_rotation": [rotation_xx, rotation_xy, rotation_yx, rotation_yy]},
                "projected_endpoints": [[i, p.x, p.y] for i, p in zip(source_indices, endpoints)],
                "candidate_source_indices": [beam_by_point[id(p)] for p in candidates],
                "detections": [{"detection_index": i, "value": asdict(d),
                                "candidate_indices": list(indices),
                                "source_indices": [beam_by_point[id(candidates[j])] for j in indices],
                                "track_id": update.detection_track_ids[i]}
                               for i, (d, indices) in enumerate(members)],
                "tracker_update": asdict(update),
                "public_prediction": {"schema": public.schema, "authority": public.authority,
                    "source_stamp_ns": Time.from_msg(public.header.stamp).nanoseconds,
                    "processing_stamp_ns": Time.from_msg(public.processing_stamp).nanoseconds,
                    "frame": public.header.frame_id, "complete": public.complete,
                    "total_track_count": public.total_track_count,
                    "prediction_dt": public.prediction_dt, "prediction_steps": public.prediction_steps,
                    "tracks": [{"id": t.track_id, "state": t.state,
                                "xy": [t.position.x, t.position.y], "vxy": [t.velocity.x, t.velocity.y],
                                "size_xy": [t.size.x, t.size.y],
                                "position_z": t.position.z, "velocity_z": t.velocity.z, "size_z": t.size.z,
                                "last_observation_stamp_ns": Time.from_msg(t.last_observation_stamp).nanoseconds,
                                "observation_count": t.observation_count, "miss_count": t.miss_count,
                                "prediction_xyz": [[p.x, p.y, p.z] for p in t.prediction]} for t in public.tracks]},
            }
            if not evidence.write(record):
                self.get_logger().error("surface member evidence incomplete: " + evidence.failed_reason)
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
    ) -> DynamicObstaclePredictionArray:
        output = DynamicObstaclePredictionArray()
        output.header.frame_id = self._map_frame
        output.header.stamp = message.header.stamp
        output.schema = self._prediction_schema
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
        if self._anchor_mode == "last_observation_cv":
            # Preserve integer ROS observation epochs: float seconds cannot
            # round-trip nanoseconds at wall-clock epoch magnitudes.
            source_ns = Time.from_msg(message.header.stamp).nanoseconds
            live_ids = {track.track_id for track in tracks}
            self._observation_stamp_ns = {
                key: value for key, value in self._observation_stamp_ns.items()
                if key in live_ids
            }
            for track in tracks:
                if track.misses == 0:
                    self._observation_stamp_ns[track.track_id] = source_ns
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
                nanoseconds=(self._observation_stamp_ns[track.track_id]
                    if self._anchor_mode == "last_observation_cv" else
                    round(track.last_observation_timestamp * 1.0e9))
            ).to_msg()
            prediction.observation_count = track.observations
            prediction.miss_count = track.misses
            prediction.prediction = [
                Point(x=point.x, y=point.y, z=0.0) for point in track.prediction
            ]
            output.tracks.append(prediction)
        self._predictions_pub.publish(output)
        return output

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
        evidence = self._surface_member_evidence
        if evidence is not None and evidence.complete and not state.startswith("ok"):
            if not evidence.write({"status": "rejected", "reason": state,
                    "source_stamp_ns": Time.from_msg(message.header.stamp).nanoseconds,
                    "scan_frame": message.header.frame_id,
                    "capture_stamp_ns": self.get_clock().now().nanoseconds}):
                self.get_logger().error("surface member evidence incomplete: " + evidence.failed_reason)
        status = DiagnosticStatus()
        status.name = "dynamic_obstacle_tracking_shadow"
        status.hardware_id = "software_shadow"
        status.level = (
            DiagnosticStatus.OK if state.startswith("ok") else DiagnosticStatus.WARN
        )
        status.message = state
        values = {
            "authority": "shadow_only",
            "prediction_schema": self._prediction_schema,
            "anchor_mode": self._anchor_mode,
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
        if evidence is not None:
            values["surface_evidence_complete"] = evidence.complete
            values["surface_evidence_records"] = evidence.records
            values["surface_evidence_failure"] = evidence.failed_reason
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

    def destroy_node(self):
        evidence = self._surface_member_evidence
        if evidence is not None:
            evidence.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleTrackerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
