from __future__ import annotations

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rm_competition_interfaces.msg import (
    AnnotatedPath,
    PathIntentSegment as PathIntentSegmentMessage,
)

from rm_path_annotations.core import (
    ANNOTATED_PATH_SCHEMA,
    PathIntentSegment,
    PathPose,
    RegionContractError,
    annotate_path,
    compute_path_revision,
    load_region_set,
    validate_map_binding,
)


class SemanticPathAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_path_annotator")
        regions_file = self.declare_parameter("regions_file", "").value
        expected_map_id = self.declare_parameter("expected_map_id", "").value
        expected_map_revision = self.declare_parameter(
            "expected_map_revision", ""
        ).value
        expected_manifest_sha256 = self.declare_parameter(
            "expected_manifest_sha256", ""
        ).value
        input_path_topic = self.declare_parameter("input_path_topic", "/plan").value
        output_topic = self.declare_parameter(
            "output_topic", "/navigation/annotated_path"
        ).value
        if not all(
            isinstance(value, str) and value
            for value in (
                regions_file,
                expected_map_id,
                expected_map_revision,
                expected_manifest_sha256,
                input_path_topic,
                output_topic,
            )
        ):
            raise RegionContractError("all semantic path annotator parameters are required")

        self._region_set = load_region_set(regions_file)
        validate_map_binding(
            self._region_set,
            expected_map_id=expected_map_id,
            expected_map_revision=expected_map_revision,
            expected_manifest_sha256=expected_manifest_sha256,
        )
        self._publisher = self.create_publisher(AnnotatedPath, output_topic, 10)
        self._subscription = self.create_subscription(
            Path, input_path_topic, self._handle_path, 10
        )
        self.get_logger().info(
            "Semantic path annotator ready: "
            f"regions={self._region_set.region_set_id}@{self._region_set.revision} "
            f"map={expected_map_id}@{expected_map_revision} "
            f"input={input_path_topic} output={output_topic}. It publishes no Path, "
            "TF, action, or velocity command."
        )

    def _handle_path(self, message: Path) -> None:
        try:
            if message.header.frame_id != self._region_set.map_binding.frame_id:
                raise RegionContractError(
                    "path frame does not match semantic-region map frame"
                )
            for index, pose in enumerate(message.poses):
                if pose.header.frame_id and pose.header.frame_id != message.header.frame_id:
                    raise RegionContractError(
                        f"path pose {index} has a conflicting frame_id"
                    )
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            poses = tuple(
                PathPose(
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    z=pose.pose.position.z,
                    qx=pose.pose.orientation.x,
                    qy=pose.pose.orientation.y,
                    qz=pose.pose.orientation.z,
                    qw=pose.pose.orientation.w,
                )
                for pose in message.poses
            )
            revision = compute_path_revision(message.header.frame_id, stamp_ns, poses)
            result = annotate_path(poses, self._region_set)
            output = AnnotatedPath()
            output.header = message.header
            output.schema = ANNOTATED_PATH_SCHEMA
            output.path_revision = revision
            output.path_pose_count = len(poses)
            output.path_length = result.path_length
            output.region_set_id = self._region_set.region_set_id
            output.region_set_revision = self._region_set.revision
            output.region_set_sha256 = self._region_set.contract_sha256
            output.map_id = self._region_set.map_binding.map_id
            output.map_revision = self._region_set.map_binding.map_revision
            output.map_manifest_sha256 = self._region_set.map_binding.manifest_sha256
            output.segments = [self._to_message(segment) for segment in result.segments]
            self._publisher.publish(output)
        except RegionContractError as error:
            self.get_logger().warning(f"Rejecting path annotation: {error}")

    @staticmethod
    def _to_message(segment: PathIntentSegment) -> PathIntentSegmentMessage:
        message = PathIntentSegmentMessage()
        message.start_distance = segment.start_distance
        message.end_distance = segment.end_distance
        message.region_ids = list(segment.region_ids)
        message.region_types = [int(value) for value in segment.region_types]
        message.blocked = segment.blocked
        message.has_max_linear_speed = segment.max_linear_speed is not None
        message.max_linear_speed = (
            segment.max_linear_speed if segment.max_linear_speed is not None else 0.0
        )
        message.has_required_heading = segment.required_heading is not None
        message.required_heading = (
            segment.required_heading if segment.required_heading is not None else 0.0
        )
        message.heading_tolerance = (
            segment.heading_tolerance if segment.heading_tolerance is not None else 0.0
        )
        message.no_spin = segment.no_spin
        message.admission_policy = int(segment.admission_policy)
        message.traversal_policy = int(segment.traversal_policy)
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticPathAnnotatorNode()
        rclpy.spin(node)
    except (RegionContractError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"semantic_path_annotator fatal: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
