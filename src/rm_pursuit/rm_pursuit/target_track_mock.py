import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rm_competition_interfaces.msg import TargetTrack


class TargetTrackMock(Node):
    def __init__(self):
        super().__init__("target_track_mock")
        self._topic = self.declare_parameter(
            "output_topic", "/perception/target_track"
        ).value
        self._frame = self.declare_parameter("frame_id", "map").value
        self._x = float(self.declare_parameter("x", 3.0).value)
        self._y = float(self.declare_parameter("y", 0.0).value)
        self._vx = float(self.declare_parameter("vx", 0.0).value)
        self._vy = float(self.declare_parameter("vy", 0.0).value)
        self._confidence = float(self.declare_parameter("confidence", 0.9).value)
        self._publisher = self.create_publisher(TargetTrack, self._topic, 10)
        self._timer = self.create_timer(0.1, self._publish)
        self.get_logger().warning(
            "MOCK target track enabled; it is not an auto-aim observation"
        )

    def _publish(self) -> None:
        message = TargetTrack()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame
        message.track_id = "mock_target"
        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.velocity.twist.linear.x = self._vx
        message.velocity.twist.linear.y = self._vy
        message.confidence = self._confidence
        message.valid = True
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackMock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
