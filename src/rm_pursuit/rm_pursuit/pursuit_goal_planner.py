import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_competition_interfaces.msg import TargetTrack
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .pursuit_math import PlanarPoint, compute_standoff_goal


def rotate_vector(x, y, z, quaternion):
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-12:
        raise ValueError("invalid transform quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * x
        + 2.0 * (qx * qy - qz * qw) * y
        + 2.0 * (qx * qz + qy * qw) * z,
        2.0 * (qx * qy + qz * qw) * x
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
        + 2.0 * (qy * qz - qx * qw) * z,
        2.0 * (qx * qz - qy * qw) * x
        + 2.0 * (qy * qz + qx * qw) * y
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * z,
    )


class PursuitGoalPlanner(Node):
    def __init__(self):
        super().__init__("pursuit_goal_planner")
        self._input_topic = self.declare_parameter(
            "input_topic", "/perception/target_track"
        ).value
        self._goal_topic = self.declare_parameter(
            "goal_topic", "/mission/pursuit_goal"
        ).value
        self._valid_topic = self.declare_parameter(
            "valid_topic", "/mission/pursuit_goal_valid"
        ).value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        self._min_confidence = float(
            self.declare_parameter("min_confidence", 0.6).value
        )
        self._max_track_age_sec = float(
            self.declare_parameter("max_track_age_sec", 0.3).value
        )
        self._max_position_variance = float(
            self.declare_parameter("max_position_variance", 1.0).value
        )
        self._standoff_distance = float(
            self.declare_parameter("standoff_distance", 1.5).value
        )
        self._prediction_horizon_sec = float(
            self.declare_parameter("prediction_horizon_sec", 0.2).value
        )
        self._max_target_distance = float(
            self.declare_parameter("max_target_distance", 8.0).value
        )
        self._transform_timeout_sec = float(
            self.declare_parameter("transform_timeout_sec", 0.05).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if min(
            self._max_track_age_sec,
            self._max_position_variance,
            self._standoff_distance,
            self._prediction_horizon_sec,
            self._max_target_distance,
            self._transform_timeout_sec,
        ) < 0.0:
            raise ValueError("pursuit limits must not be negative")

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self._valid_pub = self.create_publisher(Bool, self._valid_topic, latched)
        self._subscription = self.create_subscription(
            TargetTrack, self._input_topic, self._handle_track, 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_track = None
        self._last_valid = None
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._update)
        self._publish_valid(False)
        self.get_logger().info(
            f"pursuit goal boundary ready: {self._input_topic} -> {self._goal_topic}"
        )

    def _handle_track(self, message: TargetTrack) -> None:
        self._latest_track = message

    def _update(self) -> None:
        track = self._latest_track
        if track is None or not self._track_is_valid(track):
            self._publish_valid(False)
            return

        try:
            target_transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                track.header.frame_id,
                rclpy.time.Time.from_msg(track.header.stamp),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
            robot_transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"pursuit transform unavailable: {error}", throttle_duration_sec=2.0
            )
            self._publish_valid(False)
            return

        try:
            target_x, target_y, _ = rotate_vector(
                track.pose.pose.position.x,
                track.pose.pose.position.y,
                track.pose.pose.position.z,
                target_transform.transform.rotation,
            )
            target_x += target_transform.transform.translation.x
            target_y += target_transform.transform.translation.y
            velocity_x, velocity_y, _ = rotate_vector(
                track.velocity.twist.linear.x,
                track.velocity.twist.linear.y,
                track.velocity.twist.linear.z,
                target_transform.transform.rotation,
            )
        except ValueError:
            self._publish_valid(False)
            return
        result = compute_standoff_goal(
            PlanarPoint(
                robot_transform.transform.translation.x,
                robot_transform.transform.translation.y,
            ),
            PlanarPoint(target_x, target_y),
            PlanarPoint(velocity_x, velocity_y),
            self._standoff_distance,
            self._prediction_horizon_sec,
            self._max_target_distance,
        )
        if result is None:
            self._publish_valid(False)
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._map_frame
        goal.pose.position.x = result.x
        goal.pose.position.y = result.y
        goal.pose.orientation.z = math.sin(result.yaw * 0.5)
        goal.pose.orientation.w = math.cos(result.yaw * 0.5)
        self._goal_pub.publish(goal)
        self._publish_valid(True)

    def _track_is_valid(self, track: TargetTrack) -> bool:
        if not track.valid or not track.header.frame_id:
            return False
        if not math.isfinite(track.confidence) or track.confidence < self._min_confidence:
            return False
        stamp = rclpy.time.Time.from_msg(track.header.stamp)
        if stamp.nanoseconds <= 0:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1.0e-9
        if age < -self._max_track_age_sec or age > self._max_track_age_sec:
            return False
        pose = track.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            track.velocity.twist.linear.x,
            track.velocity.twist.linear.y,
            track.pose.covariance[0],
            track.pose.covariance[7],
        )
        if not all(math.isfinite(value) for value in values):
            return False
        if (
            track.pose.covariance[0] < 0.0
            or track.pose.covariance[7] < 0.0
            or track.pose.covariance[0] > self._max_position_variance
            or track.pose.covariance[7] > self._max_position_variance
        ):
            return False
        return True

    def _publish_valid(self, value: bool) -> None:
        if self._last_valid == value:
            return
        self._last_valid = value
        message = Bool()
        message.data = value
        self._valid_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PursuitGoalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
