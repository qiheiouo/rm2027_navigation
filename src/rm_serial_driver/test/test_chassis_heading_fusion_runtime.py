import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rm_competition_interfaces.msg import GimbalState
from sensor_msgs.msg import JointState


class FusionProbe(Node):
    def __init__(self):
        super().__init__("chassis_heading_fusion_probe")
        self.odom = None
        self.odom_count = 0
        self.derived_gimbal = None
        self.joint = None
        self.create_subscription(Odometry, "/odometry/lio", self._odom, 10)
        self.create_subscription(
            GimbalState,
            "/gimbal/state_derived",
            self._derived_gimbal,
            50,
        )
        self.create_subscription(JointState, "/joint_states", self._joint, 10)

    def _odom(self, message):
        self.odom = message
        self.odom_count += 1

    def _derived_gimbal(self, message):
        self.derived_gimbal = message

    def _joint(self, message):
        self.joint = message


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


def test_chassis_heading_fusion_static_no_hardware():
    os.environ["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 20)
    commands = [
        [
            executable("rm_serial_driver", "competition_v2_transport_node"),
            "--ros-args",
            "-p", "dry_run:=true",
            "-p", "required_remote_capabilities:=33",
        ],
        [
            executable("rm_serial_driver", "competition_v2_mock_lower_node"),
            "--ros-args",
            "-p", "chassis_heading_publish_duration_sec:=4.0",
        ],
        [
            executable("rm_localization_adapters", "fake_lio_odom_publisher"),
            "--ros-args",
            "-p", "publish_rate_hz:=50.0",
            "-p", "sensor_offset.x:=0.0",
            "-p", "sensor_offset.y:=0.0",
            "-p", "sensor_offset.z:=0.0",
        ],
        [
            executable("rm_localization_adapters", "lio_adapter"),
            "--ros-args",
            "-p", "pose_conversion_mode:=chassis_heading_fusion",
            "-p", "raw_odom_parent_frame_mode:=sensor_initial",
            "-p", "twist_mode:=finite_difference",
            "-p", "use_tf_sensor_to_base:=false",
            "-p", "heading_fusion.initial_alignment_confirmed:=true",
        ],
        [
            executable("rm_localization_adapters", "gimbal_state_adapter"),
            "--ros-args",
            "-p", "use_input:=true",
            "-p", "input_topic:=/gimbal/state_derived",
        ],
    ]
    processes = [
        subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        for command in commands
    ]

    rclpy.init()
    node = FusionProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_for(lambda: node.odom is not None, 8.0, "canonical odometry", processes)
        wait_for(
            lambda: node.joint is not None
            and "gimbal_yaw_joint" in node.joint.name,
            5.0,
            "derived gimbal joint",
            processes,
        )
        wait_for(
            lambda: node.derived_gimbal is not None
            and node.derived_gimbal.valid,
            5.0,
            "valid derived gimbal state",
            processes,
        )
        assert node.odom.header.frame_id == "odom"
        assert node.odom.child_frame_id == "base_link"
        assert math.isclose(node.odom.pose.pose.position.x, 0.0, abs_tol=1e-6)
        assert math.isclose(node.odom.pose.pose.position.y, 0.0, abs_tol=1e-6)
        assert math.isclose(node.odom.pose.pose.orientation.z, 0.0, abs_tol=1e-6)
        assert math.isclose(node.odom.pose.pose.orientation.w, 1.0, abs_tol=1e-6)
        joint_index = node.joint.name.index("gimbal_yaw_joint")
        assert math.isclose(node.joint.position[joint_index], 0.0, abs_tol=1e-6)

        wait_for(
            lambda: node.derived_gimbal is not None
            and not node.derived_gimbal.valid,
            7.0,
            "derived gimbal invalidation after heading timeout",
            processes,
        )
        time.sleep(0.3)
        odom_count_after_invalidation = node.odom_count
        time.sleep(0.5)
        assert node.odom_count == odom_count_after_invalidation
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
