import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rm_competition_interfaces.msg import RefereeState
from std_msgs.msg import Bool


class FakeNav2(Node):
    def __init__(self):
        super().__init__("three_point_spin_fake_nav2")
        self.nav_goals = []
        self.spin_goals = []
        self.held_spin_started = threading.Event()
        self.spin_canceled = threading.Event()
        self.hp = 400
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self.execute_nav,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.spin_server = ActionServer(
            self,
            Spin,
            "/spin",
            execute_callback=self.execute_spin,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.localization_pub = self.create_publisher(
            Bool, "/localization/global_localization_valid", qos
        )
        self.referee_valid_pub = self.create_publisher(Bool, "/referee/state_valid", qos)
        self.referee_pub = self.create_publisher(RefereeState, "/referee/state", qos)
        self.create_timer(0.05, self.publish_safety)

    def execute_nav(self, goal_handle):
        pose = goal_handle.request.pose.pose
        self.nav_goals.append((pose.position.x, pose.position.y))
        goal_handle.succeed()
        return NavigateToPose.Result()

    def execute_spin(self, goal_handle):
        request = goal_handle.request
        allowance = request.time_allowance.sec + request.time_allowance.nanosec / 1e9
        self.spin_goals.append((request.target_yaw, allowance))
        if len(self.spin_goals) <= 3:
            goal_handle.succeed()
            return Spin.Result()
        self.held_spin_started.set()
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            time.sleep(0.02)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.spin_canceled.set()
        else:
            goal_handle.abort()
        return Spin.Result()

    def publish_safety(self):
        valid = Bool(data=True)
        self.localization_pub.publish(valid)
        self.referee_valid_pub.publish(valid)
        state = RefereeState()
        state.valid = True
        state.game_progress = 4
        state.current_hp = self.hp
        self.referee_pub.publish(state)


def wait_for(predicate, timeout, label, process):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        if process.poll() is not None:
            raise RuntimeError(f"mission exited early with {process.returncode}: {label}")
        time.sleep(0.02)
    raise TimeoutError(label)


def test_patrol_spin_and_low_hp_home_action_flow():
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 50)
    rclpy.init()
    node = FakeNav2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    source = os.environ["RM_COMPETITION_MISSION_SOURCE_DIR"].lstrip(os.pathsep)
    prefix = get_package_prefix("rm_competition_mission")
    binary = str(Path(prefix) / "lib" / "rm_competition_mission" / "competition_mission_node")
    config = str(Path(source) / "config" / "mission_fresh03_three_point_spin_test.yaml")
    tree = str(Path(source) / "trees" / "competition_three_point_spin_test.xml")
    command = [
        binary,
        "--ros-args",
        "--params-file", config,
        "-p", f"tree_xml:={tree}",
        "-p", "startup_enabled:=true",
        "-p", "default_mode:=patrol",
        "-p", "require_chassis_mode:=false",
    ]
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    try:
        wait_for(node.held_spin_started.is_set, 10.0, "fourth Spin goal", process)
        assert len(node.nav_goals) >= 4
        expected_patrol = [
            (4.775, 2.075),
            (2.575, 2.125),
            (0.575, 0.225),
            (4.775, 2.075),
        ]
        for actual, expected in zip(node.nav_goals[:4], expected_patrol):
            assert math.isclose(actual[0], expected[0], abs_tol=1e-6)
            assert math.isclose(actual[1], expected[1], abs_tol=1e-6)
        for target_yaw, allowance in node.spin_goals:
            assert math.isclose(target_yaw, 100.0, abs_tol=1e-3)
            assert math.isclose(allowance, 15.0, abs_tol=1e-6)

        node.hp = 199
        wait_for(node.spin_canceled.is_set, 5.0, "low-HP Spin cancellation", process)
        wait_for(lambda: len(node.nav_goals) >= 5, 5.0, "low-HP home goal", process)
        home_x, home_y = node.nav_goals[-1]
        assert math.isclose(home_x, 0.872, abs_tol=1e-6)
        assert math.isclose(home_y, 0.469, abs_tol=1e-6)
        print(
            "PASS patrol_cycles=3 spin=(%.1frad,%.1fs) "
            "low_hp_cancel=true home=(%.3f,%.3f)"
            % (target_yaw, allowance, home_x, home_y)
        )
    finally:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=3.0)
        print(output)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
