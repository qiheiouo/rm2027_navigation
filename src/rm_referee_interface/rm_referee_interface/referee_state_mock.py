import rclpy
from rclpy.node import Node
from rm_competition_interfaces.msg import RefereeState


class RefereeStateMock(Node):
    def __init__(self):
        super().__init__("referee_state_mock")
        self._topic = self.declare_parameter(
            "output_topic", "/referee/state_raw"
        ).value
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self._game_progress = int(self.declare_parameter("game_progress", 4).value)
        self._stage_remain_time = int(
            self.declare_parameter("stage_remain_time", 300).value
        )
        self._robot_id = int(self.declare_parameter("robot_id", 7).value)
        self._current_hp = int(self.declare_parameter("current_hp", 400).value)
        self._self_outpost_hp = int(
            self.declare_parameter("self_outpost_hp", 1500).value
        )
        self._enemy_outpost_hp = int(
            self.declare_parameter("enemy_outpost_hp", 1500).value
        )
        self._projectiles = int(
            self.declare_parameter("projectile_allowance_17mm", 100).value
        )
        self._coins = int(
            self.declare_parameter("remaining_gold_coin", 0).value
        )
        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._publisher = self.create_publisher(RefereeState, self._topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish)
        self.get_logger().warning(
            "MOCK referee source enabled; values are not real match data"
        )

    def _publish(self) -> None:
        message = RefereeState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.valid = True
        message.game_progress = self._game_progress
        message.stage_remain_time = self._stage_remain_time
        message.robot_id = self._robot_id
        message.current_hp = self._current_hp
        message.self_outpost_hp = self._self_outpost_hp
        message.enemy_outpost_hp = self._enemy_outpost_hp
        message.projectile_allowance_17mm = self._projectiles
        message.remaining_gold_coin = self._coins
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RefereeStateMock()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
