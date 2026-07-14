import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_competition_interfaces.msg import RefereeState
from std_msgs.msg import Bool

from .validation import RefereeSnapshot, validate_snapshot


class RefereeStateGate(Node):
    def __init__(self):
        super().__init__("referee_state_gate")
        self._input_topic = self.declare_parameter(
            "input_topic", "/referee/state_raw"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/referee/state"
        ).value
        self._valid_topic = self.declare_parameter(
            "valid_topic", "/referee/state_valid"
        ).value
        self._max_source_age_sec = float(
            self.declare_parameter("max_source_age_sec", 0.5).value
        )
        self._state_timeout_sec = float(
            self.declare_parameter("state_timeout_sec", 0.5).value
        )
        self._max_stage_time_sec = int(
            self.declare_parameter("max_stage_time_sec", 900).value
        )
        self._max_hp = int(self.declare_parameter("max_hp", 10000).value)
        if self._max_source_age_sec < 0.0 or self._state_timeout_sec < 0.0:
            raise ValueError("referee time limits must not be negative")

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(
            RefereeState, self._output_topic, latched
        )
        self._valid_pub = self.create_publisher(Bool, self._valid_topic, latched)
        self._subscription = self.create_subscription(
            RefereeState, self._input_topic, self._handle_state, 10
        )
        self._last_receive_time = None
        self._last_valid = None
        self._timer = self.create_timer(0.1, self._check_timeout)
        self._publish_valid(False)
        self.get_logger().info(
            f"referee gate ready: {self._input_topic} -> {self._output_topic}"
        )

    def _snapshot(self, message: RefereeState) -> RefereeSnapshot:
        stamp_sec = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec
        ) * 1.0e-9
        return RefereeSnapshot(
            stamp_sec=stamp_sec,
            valid=bool(message.valid),
            game_progress=int(message.game_progress),
            stage_remain_time=int(message.stage_remain_time),
            robot_id=int(message.robot_id),
            current_hp=int(message.current_hp),
            self_outpost_hp=int(message.self_outpost_hp),
            enemy_outpost_hp=int(message.enemy_outpost_hp),
            projectile_allowance_17mm=int(message.projectile_allowance_17mm),
            remaining_gold_coin=int(message.remaining_gold_coin),
        )

    def _handle_state(self, message: RefereeState) -> None:
        now = self.get_clock().now()
        accepted, reason = validate_snapshot(
            self._snapshot(message),
            now.nanoseconds * 1.0e-9,
            self._max_source_age_sec,
            self._max_stage_time_sec,
            self._max_hp,
        )
        self._last_receive_time = now
        if not accepted:
            self.get_logger().warning(f"reject referee state: {reason}")
            self._publish_valid(False)
            return
        output = RefereeState()
        output = message
        output.valid = True
        self._state_pub.publish(output)
        self._publish_valid(True)

    def _check_timeout(self) -> None:
        if self._last_receive_time is None:
            return
        age = self.get_clock().now() - self._last_receive_time
        if age > Duration(seconds=self._state_timeout_sec):
            self._publish_valid(False)

    def _publish_valid(self, value: bool) -> None:
        if self._last_valid == value:
            return
        self._last_valid = value
        message = Bool()
        message.data = value
        self._valid_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RefereeStateGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
