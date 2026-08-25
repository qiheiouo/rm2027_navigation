import os
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


# ament's APPEND_ENV keeps platform path-list semantics and may prefix an
# otherwise empty variable with ':'. Each variable carries exactly one path.
BINARY = Path(
    os.environ["RM_RELOCALIZATION_BRIDGE_BINARY"].lstrip(os.pathsep)
)
CONFIG = Path(
    os.environ["RM_RELOCALIZATION_BRIDGE_OLD_CAR_CONFIG"].lstrip(os.pathsep)
)


class RecoveryProbe(Node):
    def __init__(self):
        super().__init__("test_map_odom_autonomous_recovery_probe")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry/lio", 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/global_pose", 10
        )
        self.manual_initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.upstream_pub = self.create_publisher(
            Bool, "/localization/amcl_backend_valid", latched
        )
        self.valid_messages = []
        self.transforms = []
        self.recovery_states = []
        self.observed_initial_poses = []
        self.create_subscription(
            Bool,
            "/localization/global_localization_valid",
            lambda message: self.valid_messages.append(message.data),
            latched,
        )
        self.create_subscription(
            TransformStamped,
            "/localization/map_to_odom",
            self.transforms.append,
            10,
        )
        self.create_subscription(
            String,
            "/localization/correction_recovery_state",
            lambda message: self.recovery_states.append(message.data),
            latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.observed_initial_poses.append,
            10,
        )

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, predicate, description, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f"timeout: {description}")

    def publish_upstream_valid(self):
        message = Bool()
        message.data = True
        self.upstream_pub.publish(message)

    def publish_odom(self, x=0.0, angular_speed=0.0):
        stamp = self.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.angular.z = angular_speed
        self.odom_pub.publish(message)
        return stamp

    def publish_candidate(self, correction_x, odom_x=0.0, angular_speed=0.0):
        stamp = self.publish_odom(odom_x, angular_speed)
        self.spin_for(0.03)
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.pose.pose.position.x = correction_x + odom_x
        message.pose.pose.orientation.w = 1.0
        self.pose_pub.publish(message)

    def publish_odom_for(self, duration, x=0.0, angular_speed=0.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_odom(x, angular_speed)
            self.spin_for(0.03)

    def publish_manual_initial_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 1.0
        self.manual_initial_pose_pub.publish(message)


def latest_transform_is(probe, expected_x, tolerance=1.0e-3):
    return bool(probe.transforms) and abs(
        probe.transforms[-1].transform.translation.x - expected_x
    ) < tolerance


def test_latched_fault_reseeds_from_trusted_pose_and_recovers_without_tf_flicker():
    os.environ["ROS_DOMAIN_ID"] = "94"
    command = [
        str(BINARY),
        "--ros-args",
        "--params-file",
        str(CONFIG),
        "-p",
        "upstream_valid_topic:=/localization/amcl_backend_valid",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    probe = RecoveryProbe()
    try:
        probe.spin_for(1.0)
        assert process.poll() is None, "map->odom bridge exited during startup"
        probe.publish_upstream_valid()
        probe.spin_for(0.2)

        probe.publish_candidate(0.0)
        probe.wait_for(
            lambda: probe.valid_messages and probe.valid_messages[-1],
            "first correction establishes baseline",
        )
        probe.wait_for(
            lambda: latest_transform_is(probe, 0.0),
            "baseline transform",
        )

        probe.publish_candidate(0.10)
        probe.wait_for(
            lambda: latest_transform_is(probe, 0.10),
            "small correction",
        )

        probe.publish_candidate(1.0)
        probe.wait_for(
            lambda: probe.valid_messages and not probe.valid_messages[-1],
            "large correction invalidates localization",
        )
        assert not latest_transform_is(probe, 1.0)

        probe.publish_candidate(0.12, angular_speed=1.0)
        probe.spin_for(0.2)
        assert not probe.valid_messages[-1], "fault recovered while moving"

        probe.publish_odom_for(0.8, x=0.5)
        probe.wait_for(
            lambda: bool(probe.observed_initial_poses),
            "autonomous initial pose",
        )
        automatic_pose = probe.observed_initial_poses[-1]
        assert abs(automatic_pose.pose.pose.position.x - 0.60) <= 1.0e-3

        probe.publish_odom_for(0.55, x=0.5)
        for _ in range(4):
            probe.publish_candidate(0.12, odom_x=0.5)
            probe.spin_for(0.10)
        assert not probe.valid_messages[-1], "recovered before five-pose streak"

        probe.publish_candidate(0.12, odom_x=0.5)
        probe.wait_for(
            lambda: probe.valid_messages and probe.valid_messages[-1],
            "five-pose autonomous recovery",
        )
        probe.wait_for(
            lambda: latest_transform_is(probe, 0.12),
            "recovered transform",
        )
        assert probe.recovery_states[-1] == "healthy"

        probe.publish_manual_initial_pose()
        probe.wait_for(
            lambda: probe.valid_messages and not probe.valid_messages[-1],
            "manual initial pose resets baseline",
        )
        probe.publish_candidate(1.0)
        probe.wait_for(
            lambda: probe.valid_messages and probe.valid_messages[-1],
            "manual baseline replacement",
        )
        probe.wait_for(
            lambda: latest_transform_is(probe, 1.0),
            "manual replacement transform",
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=3.0)
        if process.returncode not in (0, -15):
            raise AssertionError(
                f"map->odom bridge exited with {process.returncode}:\n{output}"
            )
