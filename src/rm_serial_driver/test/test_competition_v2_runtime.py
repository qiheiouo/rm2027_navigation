import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_competition_interfaces.msg import (
    ChassisHeadingState,
    GimbalState,
    OperatorNavigationTarget,
    PostureRequest,
    PostureState,
    RobotPosture,
    SerialConnectionState,
)
from sensor_msgs.msg import JointState


class CompetitionV2Probe(Node):
    def __init__(self):
        super().__init__("competition_v2_runtime_probe")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.connection = None
        self.gimbal = None
        self.chassis_heading = None
        self.operator_target = None
        self.joint_state = None
        self.posture_states = []
        self.create_subscription(
            SerialConnectionState,
            "/serial/connection_state",
            self._connection_callback,
            latched,
        )
        self.create_subscription(
            PostureState,
            "/robot/posture/state",
            self.posture_states.append,
            latched,
        )
        self.create_subscription(
            GimbalState, "/gimbal/state", self._gimbal_callback, 10
        )
        self.create_subscription(
            ChassisHeadingState,
            "/chassis/heading",
            self._chassis_heading_callback,
            50,
        )
        self.create_subscription(
            OperatorNavigationTarget,
            "/operator/navigation_target_raw",
            self._operator_callback,
            10,
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )
        self.posture_pub = self.create_publisher(
            PostureRequest, "/robot/posture/request", 10
        )
        self.request = PostureRequest()
        self.request.requested_posture.value = RobotPosture.ENHANCED_DEFENSE
        self.request.command_id = 0xFFFFFFFF
        self.request.valid = True
        self.create_timer(0.05, self._publish_request)

    def _connection_callback(self, message):
        self.connection = message

    def _gimbal_callback(self, message):
        self.gimbal = message

    def _chassis_heading_callback(self, message):
        self.chassis_heading = message

    def _operator_callback(self, message):
        self.operator_target = message

    def _joint_callback(self, message):
        self.joint_state = message

    def _publish_request(self):
        self.request.header.stamp = self.get_clock().now().to_msg()
        self.posture_pub.publish(self.request)


def executable(package, name):
    prefix = Path(get_package_prefix(package))
    return str(prefix / "lib" / package / name)


def wait_for(predicate, timeout, label, processes):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError(
                    f"process exited early with {process.returncode}: {label}"
                )
        time.sleep(0.02)
    raise TimeoutError(label)


def test_competition_v2_no_hardware_flow():
    os.environ["ROS_DOMAIN_ID"] = str(200 + os.getpid() % 30)
    commands = [
        [
            executable("rm_serial_driver", "competition_v2_transport_node"),
            "--ros-args",
            "-p", "dry_run:=true",
            "-p", "required_remote_capabilities:=63",
        ],
        [
            executable("rm_serial_driver", "competition_v2_mock_lower_node"),
            "--ros-args",
            "-p", "publish_operator_target:=true",
            "-p", "posture_transition_responses:=10",
            "-p", "gimbal_publish_duration_sec:=2.0",
            "-p", "chassis_heading_publish_duration_sec:=2.0",
            "-p", "heartbeat_publish_duration_sec:=3.0",
        ],
        [
            executable("rm_localization_adapters", "gimbal_state_adapter"),
            "--ros-args",
            "-p", "use_input:=true",
        ],
    ]
    processes = [
        subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        for command in commands
    ]

    rclpy.init()
    node = CompetitionV2Probe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_for(
            lambda: node.connection is not None
            and node.connection.online
            and node.connection.compatible,
            8.0,
            "compatible serial connection",
            processes,
        )
        wait_for(
            lambda: node.gimbal is not None and node.gimbal.valid,
            3.0,
            "valid gimbal state",
            processes,
        )
        wait_for(
            lambda: node.chassis_heading is not None
            and node.chassis_heading.valid,
            3.0,
            "valid chassis heading",
            processes,
        )
        wait_for(
            lambda: node.joint_state is not None
            and "gimbal_yaw_joint" in node.joint_state.name,
            3.0,
            "gimbal joint state",
            processes,
        )
        wait_for(
            lambda: any(
                state.ack_matches_request
                and state.transitioning
                and not state.completed
                for state in node.posture_states
            ),
            3.0,
            "transitional posture acknowledgement",
            processes,
        )
        wait_for(
            lambda: any(
                state.ack_matches_request
                and state.completed
                and not state.transitioning
                and not state.fault
                for state in node.posture_states
            ),
            3.0,
            "completed posture acknowledgement",
            processes,
        )
        wait_for(
            lambda: node.operator_target is not None,
            3.0,
            "operator target",
            processes,
        )

        assert node.connection.last_valid_frame_stamp.sec != 0
        assert math.isclose(node.gimbal.relative_yaw_rad, 0.25, abs_tol=1e-6)
        assert math.isclose(node.chassis_heading.yaw_rad, 0.4, abs_tol=1e-6)
        assert node.chassis_heading.reset_counter == 0
        assert node.chassis_heading.source_boot_id == 0x20270001
        joint_index = node.joint_state.name.index("gimbal_yaw_joint")
        assert math.isclose(node.joint_state.position[joint_index], 0.25, abs_tol=1e-6)
        assert node.operator_target.transport_valid
        assert node.operator_target.target_x == 0.0
        assert node.operator_target.target_y == 0.0
        assert len(node.get_publishers_info_by_topic("/cmd_vel")) == 0
        wait_for(
            lambda: node.gimbal is not None and not node.gimbal.valid,
            4.0,
            "stale gimbal invalidation",
            processes,
        )
        wait_for(
            lambda: node.chassis_heading is not None
            and not node.chassis_heading.valid,
            4.0,
            "stale chassis heading invalidation",
            processes,
        )
        wait_for(
            lambda: node.connection is not None and not node.connection.online,
            4.0,
            "heartbeat disconnect invalidation",
            processes,
        )
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                output, _ = process.communicate(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _ = process.communicate(timeout=3.0)
            print(output)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
