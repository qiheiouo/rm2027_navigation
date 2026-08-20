"""ROS wrapper for revision-bound shadow dynamic-clearance evaluation."""

from __future__ import annotations

from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rm_competition_interfaces.msg import (
    AnnotatedPath,
    DynamicClearanceReport,
    DynamicObstaclePredictionArray,
)

from rm_dynamic_clearance.core import (
    ClearanceConfig,
    ClearanceContractError,
    Decision,
    IntentSegment,
    Point2D,
    PredictedTrack,
    PredictionFrame,
    evaluate_dynamic_clearance,
    validate_clearance_config,
)
from rm_path_annotations.core import (
    ANNOTATED_PATH_SCHEMA,
    PathPose,
    RegionContractError,
    compute_path_revision,
)
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1.0e9


class DynamicClearanceNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_clearance_shadow")
        path_topic = str(self.declare_parameter("path_topic", "/plan").value)
        annotated_topic = str(
            self.declare_parameter(
                "annotated_path_topic", "/navigation/annotated_path"
            ).value
        )
        prediction_topic = str(
            self.declare_parameter(
                "prediction_topic",
                "/perception/dynamic_obstacles_shadow/predictions",
            ).value
        )
        report_topic = str(
            self.declare_parameter(
                "report_topic", "/navigation/dynamic_clearance_shadow"
            ).value
        )
        topics = (path_topic, annotated_topic, prediction_topic, report_topic)
        if any(not topic or not topic.startswith("/") for topic in topics):
            raise ClearanceContractError("all topics must be absolute and non-empty")
        if len(set(topics)) != len(topics):
            raise ClearanceContractError("dynamic-clearance topics must be distinct")
        self._base_frame = str(
            self.declare_parameter("base_frame", "base_link").value
        )
        self._tf_timeout = float(
            self.declare_parameter("tf_timeout_sec", 0.05).value
        )
        if (
            not self._base_frame
            or self._base_frame in ("map", "odom")
            or self._tf_timeout < 0.0
        ):
            raise ClearanceContractError("base_frame/tf_timeout_sec is invalid")
        self._config = ClearanceConfig(
            nominal_speed=float(
                self.declare_parameter("nominal_speed_mps", 1.0).value
            ),
            sample_resolution=float(
                self.declare_parameter("sample_resolution_m", 0.10).value
            ),
            lookahead_distance=float(
                self.declare_parameter("lookahead_distance_m", 6.0).value
            ),
            robot_radius=float(
                self.declare_parameter("robot_radius_m", 0.35).value
            ),
            minimum_obstacle_radius=float(
                self.declare_parameter("minimum_obstacle_radius_m", 0.15).value
            ),
            safety_margin=float(
                self.declare_parameter("safety_margin_m", 0.15).value
            ),
            coasting_extra_margin=float(
                self.declare_parameter("coasting_extra_margin_m", 0.15).value
            ),
            max_prediction_age=float(
                self.declare_parameter("max_prediction_age_sec", 0.40).value
            ),
            max_future_prediction=float(
                self.declare_parameter("max_future_prediction_sec", 0.05).value
            ),
            max_path_projection_distance=float(
                self.declare_parameter(
                    "max_path_projection_distance_m", 1.0
                ).value
            ),
            projection_ambiguity_distance=float(
                self.declare_parameter(
                    "projection_ambiguity_distance_m", 0.03
                ).value
            ),
            projection_ambiguity_progress=float(
                self.declare_parameter(
                    "projection_ambiguity_progress_m", 0.50
                ).value
            ),
            include_coasting=bool(
                self.declare_parameter("include_coasting", True).value
            ),
        )
        validate_clearance_config(self._config)
        self._path: Path | None = None
        self._path_points: tuple[Point2D, ...] = ()
        self._path_revision = ""
        self._annotation: AnnotatedPath | None = None
        self._prediction: DynamicObstaclePredictionArray | None = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._report_publisher = self.create_publisher(
            DynamicClearanceReport, report_topic, 10
        )
        self._path_subscription = self.create_subscription(
            Path, path_topic, self._on_path, 10
        )
        self._annotation_subscription = self.create_subscription(
            AnnotatedPath, annotated_topic, self._on_annotation, 10
        )
        self._prediction_subscription = self.create_subscription(
            DynamicObstaclePredictionArray,
            prediction_topic,
            self._on_prediction,
            10,
        )
        self.get_logger().info(
            "SHADOW ONLY dynamic-clearance evaluator ready. It publishes only "
            f"{report_topic}; it owns no Path, TF, action, costmap, or cmd_vel."
        )

    def _on_path(self, message: Path) -> None:
        try:
            if not message.header.frame_id:
                raise ClearanceContractError("path frame is empty")
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            poses: list[PathPose] = []
            points: list[Point2D] = []
            for index, pose in enumerate(message.poses):
                if pose.header.frame_id and pose.header.frame_id != message.header.frame_id:
                    raise ClearanceContractError(
                        f"path pose {index} has a conflicting frame"
                    )
                poses.append(
                    PathPose(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z,
                        qx=pose.pose.orientation.x,
                        qy=pose.pose.orientation.y,
                        qz=pose.pose.orientation.z,
                        qw=pose.pose.orientation.w,
                    )
                )
                points.append(Point2D(pose.pose.position.x, pose.pose.position.y))
            revision = compute_path_revision(message.header.frame_id, stamp_ns, poses)
            self._path = message
            self._path_points = tuple(points)
            self._path_revision = revision
            self._try_evaluate()
        except (ClearanceContractError, RegionContractError) as error:
            self._path = None
            self._path_points = ()
            self._path_revision = ""
            self.get_logger().warning(f"Rejecting clearance path: {error}")

    def _on_annotation(self, message: AnnotatedPath) -> None:
        self._annotation = message
        self._try_evaluate()

    def _on_prediction(self, message: DynamicObstaclePredictionArray) -> None:
        self._prediction = message
        self._try_evaluate()

    def _try_evaluate(self) -> None:
        if self._path is None or self._annotation is None or self._prediction is None:
            return
        try:
            annotation = self._annotation
            prediction_message = self._prediction
            if annotation.schema != ANNOTATED_PATH_SCHEMA:
                raise ClearanceContractError("unsupported annotated-path schema")
            if annotation.header.frame_id != self._path.header.frame_id:
                raise ClearanceContractError("annotation frame does not match path")
            if self._base_frame == self._path.header.frame_id:
                raise ClearanceContractError("base frame must differ from path frame")
            if annotation.header.stamp != self._path.header.stamp:
                raise ClearanceContractError("annotation stamp does not match path")
            if annotation.path_pose_count != len(self._path_points):
                raise ClearanceContractError("annotation pose count does not match path")
            segments = tuple(
                IntentSegment(
                    start_distance=segment.start_distance,
                    end_distance=segment.end_distance,
                    region_ids=tuple(segment.region_ids),
                    blocked=segment.blocked,
                    max_linear_speed=(
                        segment.max_linear_speed
                        if segment.has_max_linear_speed
                        else None
                    ),
                    admission_policy=segment.admission_policy,
                    traversal_policy=segment.traversal_policy,
                )
                for segment in annotation.segments
            )
            prediction_stamp = Time.from_msg(prediction_message.header.stamp)
            if prediction_stamp.nanoseconds <= 0:
                raise ClearanceContractError("prediction source stamp must be positive")
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._path.header.frame_id,
                    self._base_frame,
                    prediction_stamp,
                    timeout=Duration(seconds=self._tf_timeout),
                )
            except TransformException as error:
                self.get_logger().warning(
                    f"Clearance robot pose unavailable at prediction stamp: {error}"
                )
                self._publish_unknown("robot_pose_unavailable")
                return
            prediction = self._prediction_from_message(prediction_message)
            report = evaluate_dynamic_clearance(
                path_frame=self._path.header.frame_id,
                path_points=self._path_points,
                robot_position=Point2D(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                ),
                path_revision=self._path_revision,
                annotated_path_revision=annotation.path_revision,
                annotated_path_length=annotation.path_length,
                region_set_sha256=annotation.region_set_sha256,
                segments=segments,
                prediction=prediction,
                now=self.get_clock().now().nanoseconds / 1.0e9,
                config=self._config,
            )
            self._publish_report(report)
        except ClearanceContractError as error:
            self.get_logger().warning(f"Clearance contract rejected: {error}")
            self._publish_unknown("contract_error")

    @staticmethod
    def _prediction_from_message(
        message: DynamicObstaclePredictionArray,
    ) -> PredictionFrame:
        tracks = []
        for track in message.tracks:
            tracks.append(
                PredictedTrack(
                    track_id=track.track_id,
                    state=track.state,
                    position=Point2D(track.position.x, track.position.y),
                    velocity=Point2D(track.velocity.x, track.velocity.y),
                    size_x=track.size.x,
                    size_y=track.size.y,
                    last_observation_stamp=_stamp_seconds(
                        track.last_observation_stamp
                    ),
                    observation_count=track.observation_count,
                    miss_count=track.miss_count,
                    prediction=tuple(
                        Point2D(point.x, point.y) for point in track.prediction
                    ),
                )
            )
        return PredictionFrame(
            frame_id=message.header.frame_id,
            source_stamp=_stamp_seconds(message.header.stamp),
            processing_stamp=_stamp_seconds(message.processing_stamp),
            prediction_dt=message.prediction_dt,
            prediction_steps=message.prediction_steps,
            complete=message.complete,
            total_track_count=message.total_track_count,
            tracks=tuple(tracks),
            schema=message.schema,
            authority=message.authority,
        )

    def _publish_report(self, report) -> None:
        output = DynamicClearanceReport()
        output.header.frame_id = self._path.header.frame_id
        output.header.stamp = self.get_clock().now().to_msg()
        output.schema = DynamicClearanceReport.SCHEMA
        output.authority = DynamicClearanceReport.AUTHORITY_SHADOW_ONLY
        output.decision = int(report.decision)
        output.reason = report.reason
        output.path_revision = report.path_revision
        output.region_set_sha256 = report.region_set_sha256
        source_ns = round(report.prediction_stamp * 1.0e9)
        output.prediction_stamp.sec = source_ns // 1_000_000_000
        output.prediction_stamp.nanosec = source_ns % 1_000_000_000
        output.region_ids = list(report.region_ids)
        output.traversal_policy = report.traversal_policy
        output.route_progress = report.route_progress
        output.start_distance = report.start_distance
        output.end_distance = report.end_distance
        output.entry_eta = report.entry_eta
        output.prediction_horizon = report.prediction_horizon
        output.sample_count = report.sample_count
        output.has_minimum_clearance = report.minimum_clearance is not None
        output.minimum_clearance = report.minimum_clearance or 0.0
        output.blocking_track_ids = list(report.blocking_track_ids)
        self._report_publisher.publish(output)

    def _publish_unknown(self, reason: str) -> None:
        output = DynamicClearanceReport()
        output.header.frame_id = self._path.header.frame_id
        output.header.stamp = self.get_clock().now().to_msg()
        output.schema = DynamicClearanceReport.SCHEMA
        output.authority = DynamicClearanceReport.AUTHORITY_SHADOW_ONLY
        output.decision = int(Decision.UNKNOWN)
        output.reason = reason
        output.path_revision = self._path_revision
        if self._annotation is not None:
            output.region_set_sha256 = self._annotation.region_set_sha256
        if self._prediction is not None:
            output.prediction_stamp = self._prediction.header.stamp
        self._report_publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DynamicClearanceNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
