from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from rm_dog_hole_entry_gate.core import DogHoleEntryPauseGate, GateState
from rm_path_annotations.core import (
    PathPose,
    RegionContractError,
    load_region_set,
    validate_map_binding,
)


class DogHoleEntryPauseGateNode(Node):
    """Gate final chassis velocity while a map-bound dog-hole pause executes."""

    def __init__(self) -> None:
        super().__init__("dog_hole_entry_pause_gate")
        regions_file = self._required_string("regions_file")
        expected_map_id = self._required_string("expected_map_id")
        expected_map_revision = self._required_string("expected_map_revision")
        expected_manifest_sha256 = self._required_string(
            "expected_manifest_sha256"
        )
        path_topic = self.declare_parameter("path_topic", "/plan").value
        pose_topic = self.declare_parameter(
            "pose_topic", "/localization/global_pose"
        ).value
        map_frame = self.declare_parameter("map_frame", "map").value
        base_frame = self.declare_parameter("base_frame", "base_link").value
        use_tf_pose = self.declare_parameter("use_tf_pose", True).value
        input_cmd_vel_topic = self.declare_parameter(
            "input_cmd_vel_topic", "/cmd_vel"
        ).value
        output_cmd_vel_topic = self.declare_parameter(
            "output_cmd_vel_topic", "/cmd_vel_dog_hole_gated"
        ).value
        hold_sec = self._finite_nonnegative("hold_sec", 5.0)
        brake_settle_sec = self._finite_nonnegative("brake_settle_sec", 0.5)
        rearm_clear_sec = self._finite_nonnegative("rearm_clear_sec", 1.0)
        pose_timeout_sec = self._finite_positive("pose_timeout_sec", 2.0)
        zero_publish_hz = self._finite_positive("zero_publish_hz", 20.0)
        if input_cmd_vel_topic == output_cmd_vel_topic:
            raise RegionContractError(
                "input_cmd_vel_topic and output_cmd_vel_topic must differ"
            )

        region_set = load_region_set(regions_file)
        validate_map_binding(
            region_set,
            expected_map_id=expected_map_id,
            expected_map_revision=expected_map_revision,
            expected_manifest_sha256=expected_manifest_sha256,
        )
        self._gate = DogHoleEntryPauseGate(
            region_set,
            brake_settle_sec=brake_settle_sec,
            hold_sec=hold_sec,
            rearm_clear_sec=rearm_clear_sec,
        )
        self._pose_timeout_sec = pose_timeout_sec
        self._last_pose_monotonic: float | None = None
        self._last_state = self._gate.state
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._tf_buffer = Buffer() if use_tf_pose else None
        self._tf_listener = (
            TransformListener(self._tf_buffer, self)
            if self._tf_buffer is not None
            else None
        )
        self._cmd_publisher = self.create_publisher(Twist, output_cmd_vel_topic, 10)
        self._state_publisher = self.create_publisher(
            String, "/dog_hole/pause_state", 10
        )
        self._active_publisher = self.create_publisher(
            Bool, "/dog_hole/pause_active", 10
        )
        self._crosses_publisher = self.create_publisher(
            Bool, "/dog_hole/path_crosses", 10
        )
        self._path_subscription = self.create_subscription(
            Path, path_topic, self._handle_path, 10
        )
        self._pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._handle_pose, 10
        )
        self._cmd_subscription = self.create_subscription(
            Twist, input_cmd_vel_topic, self._handle_cmd, 10
        )
        self._timer = self.create_timer(1.0 / zero_publish_hz, self._tick)
        self.get_logger().warning(
            "DOG-HOLE ENTRY PAUSE GATE ACTIVE: map="
            f"{expected_map_id}@{expected_map_revision}, hold={hold_sec:.3f}s, "
            f"input={input_cmd_vel_topic}, output={output_cmd_vel_topic}. "
            "Velocity is fail-closed until a valid path is available."
        )

    def _required_string(self, name: str) -> str:
        value = self.declare_parameter(name, "").value
        if not isinstance(value, str) or not value.strip():
            raise RegionContractError(f"{name} must be a non-empty string")
        return value.strip()

    def _finite_nonnegative(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RegionContractError(f"{name} must be numeric")
        result = float(value)
        if not math.isfinite(result) or result < 0.0:
            raise RegionContractError(f"{name} must be finite and non-negative")
        return result

    def _finite_positive(self, name: str, default: float) -> float:
        result = self._finite_nonnegative(name, default)
        if result <= 0.0:
            raise RegionContractError(f"{name} must be positive")
        return result

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _handle_path(self, message: Path) -> None:
        now = self._now()
        try:
            if message.header.frame_id != "map":
                raise RegionContractError("dog-hole path frame must be 'map'")
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
            self._gate.set_path(poses, now)
        except RegionContractError as error:
            self._gate.reject_path(now)
            self.get_logger().error(f"Rejecting dog-hole path: {error}")
        self._publish_status()

    def _handle_pose(self, message: PoseWithCovarianceStamped) -> None:
        now = self._now()
        if message.header.frame_id != "map":
            self.get_logger().error(
                "Ignoring global pose whose frame is not canonical 'map'"
            )
            return
        try:
            self._gate.update_pose(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                now,
            )
            self._last_pose_monotonic = now
        except RegionContractError as error:
            self.get_logger().error(f"Rejecting dog-hole pose: {error}")
        self._observe_transition()
        self._publish_status()

    def _handle_cmd(self, message: Twist) -> None:
        now = self._now()
        self._gate.tick(now)
        if self._must_stop(now):
            self._cmd_publisher.publish(Twist())
        else:
            self._cmd_publisher.publish(message)
        self._observe_transition()

    def _tick(self) -> None:
        now = self._now()
        self._refresh_tf_pose(now)
        self._gate.tick(now)
        if self._must_stop(now):
            self._cmd_publisher.publish(Twist())
        self._observe_transition()
        self._publish_status()

    def _refresh_tf_pose(self, now: float) -> None:
        if self._tf_buffer is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException:
            return
        stamp = transform.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec / 1_000_000_000.0
        if stamp_sec <= 0.0 or abs(now - stamp_sec) > self._pose_timeout_sec:
            return
        translation = transform.transform.translation
        try:
            self._gate.update_pose(translation.x, translation.y, now)
            self._last_pose_monotonic = now
        except RegionContractError as error:
            self.get_logger().error(f"Rejecting dog-hole TF pose: {error}")

    def _must_stop(self, now: float) -> bool:
        pose_fresh = (
            self._last_pose_monotonic is not None
            and now - self._last_pose_monotonic <= self._pose_timeout_sec
        )
        return self._gate.must_stop(pose_fresh=pose_fresh)

    def _observe_transition(self) -> None:
        state = self._gate.state
        if state == self._last_state:
            return
        if state in {GateState.BRAKING, GateState.HOLDING}:
            self.get_logger().warning(f"Dog-hole pause state: {state.value}")
        elif state == GateState.INVALID_ENTRY:
            self.get_logger().error(
                "Robot entered committed corridor without completing entry pause; "
                "velocity remains blocked"
            )
        else:
            self.get_logger().info(f"Dog-hole pause state: {state.value}")
        self._last_state = state

    def _publish_status(self) -> None:
        snapshot = self._gate.snapshot()
        state = String()
        state.data = snapshot.state.value
        self._state_publisher.publish(state)
        active = Bool()
        active.data = snapshot.state in {GateState.BRAKING, GateState.HOLDING}
        self._active_publisher.publish(active)
        crosses = Bool()
        crosses.data = snapshot.path_crosses_corridor
        self._crosses_publisher.publish(crosses)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DogHoleEntryPauseGateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
