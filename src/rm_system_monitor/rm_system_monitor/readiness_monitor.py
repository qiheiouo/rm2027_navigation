import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rm_competition_interfaces.msg import ChassisMode, SystemReadiness
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from .readiness import RequirementPolicy, evaluate_readiness


class ReadinessMonitor(Node):
    def __init__(self):
        super().__init__("readiness_monitor")
        self._profile = self.declare_parameter(
            "profile", "old_car_2026_competition"
        ).value
        self._timeout_sec = float(self.declare_parameter("data_timeout_sec", 0.5).value)
        self._policy = RequirementPolicy(
            require_lio=bool(self.declare_parameter("require_lio", True).value),
            require_obstacle_input=bool(
                self.declare_parameter("require_obstacle_input", True).value
            ),
            require_localization=bool(
                self.declare_parameter("require_localization", True).value
            ),
            require_nav2=bool(self.declare_parameter("require_nav2", True).value),
            require_referee=bool(
                self.declare_parameter("require_referee", False).value
            ),
            require_chassis_mode=bool(
                self.declare_parameter("require_chassis_mode", False).value
            ),
            require_serial_transport=bool(
                self.declare_parameter("require_serial_transport", False).value
            ),
        )
        self._odom_topic = self.declare_parameter(
            "odom_topic", "/odometry/lio"
        ).value
        self._obstacle_topic = self.declare_parameter(
            "obstacle_topic", "/livox/left/pointcloud_filtered"
        ).value
        self._last_odom = None
        self._last_obstacle = None
        self._localization_valid = False
        self._referee_valid = False
        self._chassis_ready = False

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            SystemReadiness, "/system/readiness", latched
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(
            PointCloud2, self._obstacle_topic, self._on_obstacle, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool,
            "/localization/global_localization_valid",
            lambda message: setattr(self, "_localization_valid", message.data),
            latched,
        )
        self.create_subscription(
            Bool,
            "/referee/state_valid",
            lambda message: setattr(self, "_referee_valid", message.data),
            latched,
        )
        self.create_subscription(
            ChassisMode, "/chassis/mode", self._on_chassis_mode, latched
        )
        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._timer = self.create_timer(0.2, self._publish)

    def _on_odom(self, _message):
        self._last_odom = self.get_clock().now()

    def _on_obstacle(self, _message):
        self._last_obstacle = self.get_clock().now()

    def _on_chassis_mode(self, message):
        self._chassis_ready = (
            message.online
            and message.autonomous_enabled
            and not message.emergency_stop
        )

    def _fresh(self, stamp):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1.0e-9 <= self._timeout_sec

    def _serial_node_present(self):
        return any(name == "serial_transport_node" for name, _namespace in self.get_node_names_and_namespaces())

    def _publish(self):
        available = set()
        if self._fresh(self._last_odom):
            available.add("lio")
        if self._fresh(self._last_obstacle):
            available.add("obstacle_input")
        if self._localization_valid:
            available.add("global_localization")
        if self._nav_client.server_is_ready():
            available.add("nav2_action")
        if self._referee_valid:
            available.add("referee_state")
        if self._chassis_ready:
            available.add("chassis_authority")
        if self._serial_node_present():
            available.add("serial_transport")

        navigation_ready, mission_ready, missing = evaluate_readiness(
            self._policy, available
        )
        message = SystemReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.profile = self._profile
        message.ready_for_navigation = navigation_ready
        message.ready_for_mission = mission_ready
        message.missing_requirements = missing
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
