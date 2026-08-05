#!/usr/bin/env python3

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def pitch_from_quaternion(quaternion):
    sin_pitch = 2.0 * (
        quaternion.w * quaternion.y
        - quaternion.z * quaternion.x
    )
    return math.asin(max(-1.0, min(1.0, sin_pitch)))


def make_case_id(lateral_m, yaw_offset_deg, repeat_index):
    return (
        f"L{lateral_m * 100.0:+05.1f}cm_"
        f"Y{yaw_offset_deg:+05.1f}deg_R{repeat_index:02d}"
    )


class CaptureMatrixRunner(Node):
    def __init__(self, args, config):
        super().__init__("dog_hole_capture_matrix")
        self.args = args
        self.config = config

        self.center_x = float(config["dog_hole.center_x"])
        self.center_y = float(config["dog_hole.center_y"])
        self.corridor_yaw = float(config["dog_hole.yaw"])
        self.corridor_width = float(config["dog_hole.width"])
        self.corridor_length = float(config["dog_hole.length"])
        self.wall_thickness = float(config["dog_hole.wall_thickness"])
        self.roof_clearance = float(config["dog_hole.roof_clearance"])
        self.deck_height = float(config["dog_hole.deck_height"])
        self.entry_clearance = float(config["dog_hole.entry_clearance"])
        self.robot_length = float(config["robot.length"])
        self.robot_width = float(config["robot.width"])
        self.robot_height = float(config["robot.height"])
        self.localization_lateral_noise_std_m = float(
            config.get(
                "simulation.localization.lateral_noise_std_m", 0.0
            )
        )
        self.localization_yaw_noise_std_deg = math.degrees(
            float(
                config.get(
                    "simulation.localization.yaw_noise_std_rad", 0.0
                )
            )
        )
        self.localization_delay_sec = float(
            config.get("simulation.localization.delay_sec", 0.0)
        )
        self.localization_lateral_drift_amplitude_m = float(
            config.get(
                "simulation.localization.lateral_drift_amplitude_m", 0.0
            )
        )
        self.localization_yaw_drift_amplitude_deg = math.degrees(
            float(
                config.get(
                    "simulation.localization.yaw_drift_amplitude_rad", 0.0
                )
            )
        )
        self.localization_drift_frequency_hz = float(
            config.get(
                "simulation.localization.drift_frequency_hz", 0.0
            )
        )
        self.localization_random_seed = int(
            config.get("simulation.localization.random_seed", 20270728)
        )
        self.chassis_forward_scale = float(
            config.get("simulation.chassis.forward_scale", 1.0)
        )
        self.chassis_lateral_positive_scale = float(
            config.get(
                "simulation.chassis.lateral_positive_scale", 1.0
            )
        )
        self.chassis_lateral_negative_scale = float(
            config.get(
                "simulation.chassis.lateral_negative_scale", 1.0
            )
        )
        self.chassis_angular_scale = float(
            config.get("simulation.chassis.angular_scale", 1.0)
        )
        self.chassis_lateral_time_constant_sec = float(
            config.get(
                "simulation.chassis.lateral_time_constant_sec", 0.0
            )
        )
        self.chassis_angular_time_constant_sec = float(
            config.get(
                "simulation.chassis.angular_time_constant_sec", 0.0
            )
        )

        self.current_state = "UNKNOWN"
        self.latest_odom = None
        self.latest_odom_3d = None
        self.latest_estimated_odom = None
        self.latest_chassis_command = None
        self.latest_applied_chassis_command = None
        self.trial_active = False
        self.state_sequence = []
        self.state_times = {}
        self.metrics = {}

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            String, "/dog_hole/state", self._state_callback, state_qos
        )
        self.create_subscription(
            Odometry,
            "/simulation/ground_truth/odom",
            self._odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            "/simulation/ground_truth/odom_3d",
            self._odom_3d_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/lio",
            self._estimated_odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Twist,
            "/simulation/chassis/cmd_vel",
            self._chassis_command_callback,
            sensor_qos,
        )
        self.create_subscription(
            Twist,
            "/simulation/chassis/cmd_vel_applied",
            self._applied_chassis_command_callback,
            sensor_qos,
        )

        self.start_client = self.create_client(Trigger, "/dog_hole/start")
        self.cancel_client = self.create_client(Trigger, "/dog_hole/cancel")
        self.local_clear_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.global_clear_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )

    def _state_callback(self, message):
        self.current_state = message.data
        if not self.trial_active:
            return
        if not self.state_sequence or self.state_sequence[-1] != message.data:
            self.state_sequence.append(message.data)
            self.state_times.setdefault(message.data, time.monotonic())

    def _evaluate_pose(self, odometry):
        position = odometry.pose.pose.position
        yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        axis_x = math.cos(self.corridor_yaw)
        axis_y = math.sin(self.corridor_yaw)
        normal_x = -axis_y
        normal_y = axis_x
        delta_x = position.x - self.center_x
        delta_y = position.y - self.center_y
        longitudinal = delta_x * axis_x + delta_y * axis_y
        lateral = delta_x * normal_x + delta_y * normal_y
        heading_error = normalize_angle(self.corridor_yaw - yaw)
        projected_half_width = (
            0.5 * self.robot_width * abs(math.cos(heading_error))
            + 0.5 * self.robot_length * abs(math.sin(heading_error))
        )
        clearance = (
            0.5 * self.corridor_width
            - projected_half_width
            - abs(lateral)
        )
        return longitudinal, lateral, heading_error, clearance

    def _odom_callback(self, message):
        self.latest_odom = message
        if not self.trial_active:
            return
        longitudinal, lateral, heading_error, clearance = self._evaluate_pose(
            message
        )
        if self.current_state == "ALIGNING" and math.isnan(
            self.metrics["observed_initial_lateral_m"]
        ):
            self.metrics["observed_initial_lateral_m"] = lateral
            self.metrics["observed_initial_yaw_error_deg"] = math.degrees(
                heading_error
            )

        if self.current_state in ("ALIGNING", "CROSSING"):
            self.metrics["max_abs_lateral_m"] = max(
                self.metrics["max_abs_lateral_m"], abs(lateral)
            )
            self.metrics["max_abs_yaw_error_deg"] = max(
                self.metrics["max_abs_yaw_error_deg"],
                abs(math.degrees(heading_error)),
            )

        if self.current_state == "CROSSING":
            self.metrics["crossing_samples"] += 1
            self.metrics["minimum_clearance_m"] = min(
                self.metrics["minimum_clearance_m"], clearance
            )
            self.metrics["crossing_max_abs_lateral_m"] = max(
                self.metrics["crossing_max_abs_lateral_m"], abs(lateral)
            )
            self.metrics["crossing_max_abs_yaw_error_deg"] = max(
                self.metrics["crossing_max_abs_yaw_error_deg"],
                abs(math.degrees(heading_error)),
            )
            self.metrics["last_crossing_longitudinal_m"] = longitudinal

    def _odom_3d_callback(self, message):
        self.latest_odom_3d = message
        if (
            not self.trial_active
            or self.current_state not in ("ALIGNING", "CROSSING")
        ):
            return
        pitch_deg = math.degrees(
            pitch_from_quaternion(message.pose.pose.orientation)
        )
        self.metrics["maximum_ground_truth_z_m"] = max(
            self.metrics["maximum_ground_truth_z_m"],
            message.pose.pose.position.z,
        )
        self.metrics["max_abs_ground_truth_pitch_deg"] = max(
            self.metrics["max_abs_ground_truth_pitch_deg"],
            abs(pitch_deg),
        )
        if self.current_state == "CROSSING":
            roof_clearances = self._roof_clearances(message)
            if roof_clearances:
                self.metrics["minimum_roof_clearance_m"] = min(
                    self.metrics["minimum_roof_clearance_m"],
                    min(roof_clearances),
                )

    def _roof_clearances(self, odometry):
        position = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        rotation = (
            (
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                2.0 * (q.x * q.y - q.z * q.w),
                2.0 * (q.x * q.z + q.y * q.w),
            ),
            (
                2.0 * (q.x * q.y + q.z * q.w),
                1.0 - 2.0 * (q.x * q.x + q.z * q.z),
                2.0 * (q.y * q.z - q.x * q.w),
            ),
            (
                2.0 * (q.x * q.z - q.y * q.w),
                2.0 * (q.y * q.z + q.x * q.w),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y),
            ),
        )
        axis_x = math.cos(self.corridor_yaw)
        axis_y = math.sin(self.corridor_yaw)
        normal_x = -axis_y
        normal_y = axis_x
        roof_half_width = 0.5 * (
            self.corridor_width + 2.0 * self.wall_thickness
        )
        roof_bottom_z = self.deck_height + self.roof_clearance
        clearances = []
        for local_x in (-0.5 * self.robot_length, 0.5 * self.robot_length):
            for local_y in (-0.5 * self.robot_width, 0.5 * self.robot_width):
                local_point = (local_x, local_y, self.robot_height)
                world_x = position.x + sum(
                    rotation[0][index] * value
                    for index, value in enumerate(local_point)
                )
                world_y = position.y + sum(
                    rotation[1][index] * value
                    for index, value in enumerate(local_point)
                )
                world_z = position.z + sum(
                    rotation[2][index] * value
                    for index, value in enumerate(local_point)
                )
                delta_x = world_x - self.center_x
                delta_y = world_y - self.center_y
                longitudinal = delta_x * axis_x + delta_y * axis_y
                lateral = delta_x * normal_x + delta_y * normal_y
                if (
                    abs(longitudinal) <= 0.5 * self.corridor_length
                    and abs(lateral) <= roof_half_width
                ):
                    clearances.append(roof_bottom_z - world_z)
        return clearances

    def _estimated_odom_callback(self, message):
        self.latest_estimated_odom = message
        if (
            not self.trial_active
            or self.latest_odom is None
            or self.current_state not in ("ALIGNING", "CROSSING")
        ):
            return
        _, true_lateral, true_heading_error, _ = self._evaluate_pose(
            self.latest_odom
        )
        _, estimated_lateral, estimated_heading_error, _ = (
            self._evaluate_pose(message)
        )
        lateral_error = estimated_lateral - true_lateral
        yaw_error_deg = math.degrees(
            normalize_angle(estimated_heading_error - true_heading_error)
        )
        self.metrics[
            "max_abs_effective_localization_lateral_error_m"
        ] = max(
            self.metrics[
                "max_abs_effective_localization_lateral_error_m"
            ],
            abs(lateral_error),
        )
        self.metrics[
            "max_abs_effective_localization_yaw_error_deg"
        ] = max(
            self.metrics["max_abs_effective_localization_yaw_error_deg"],
            abs(yaw_error_deg),
        )
        if self.current_state == "CROSSING":
            self.metrics[
                "crossing_max_abs_effective_localization_lateral_error_m"
            ] = max(
                self.metrics[
                    "crossing_max_abs_effective_localization_lateral_error_m"
                ],
                abs(lateral_error),
            )
            self.metrics[
                "crossing_max_abs_effective_localization_yaw_error_deg"
            ] = max(
                self.metrics[
                    "crossing_max_abs_effective_localization_yaw_error_deg"
                ],
                abs(yaw_error_deg),
            )

    def _update_chassis_response_metrics(self):
        if (
            not self.trial_active
            or self.latest_chassis_command is None
            or self.latest_applied_chassis_command is None
            or self.current_state not in ("ALIGNING", "CROSSING")
        ):
            return
        lateral_error = (
            self.latest_chassis_command.linear.y
            - self.latest_applied_chassis_command.linear.y
        )
        angular_error = (
            self.latest_chassis_command.angular.z
            - self.latest_applied_chassis_command.angular.z
        )
        self.metrics["max_abs_lateral_command_error_mps"] = max(
            self.metrics["max_abs_lateral_command_error_mps"],
            abs(lateral_error),
        )
        self.metrics["max_abs_angular_command_error_radps"] = max(
            self.metrics["max_abs_angular_command_error_radps"],
            abs(angular_error),
        )
        if self.current_state == "CROSSING":
            self.metrics["crossing_max_abs_lateral_command_error_mps"] = max(
                self.metrics[
                    "crossing_max_abs_lateral_command_error_mps"
                ],
                abs(lateral_error),
            )
            self.metrics["crossing_max_abs_angular_command_error_radps"] = max(
                self.metrics[
                    "crossing_max_abs_angular_command_error_radps"
                ],
                abs(angular_error),
            )

    def _chassis_command_callback(self, message):
        self.latest_chassis_command = message
        self._update_chassis_response_metrics()

    def _applied_chassis_command_callback(self, message):
        self.latest_applied_chassis_command = message
        self._update_chassis_response_metrics()

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _call_trigger(self, client, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            return False, "service timeout"
        response = future.result()
        if response is None:
            return False, "empty service response"
        return bool(response.success), response.message

    def _clear_costmap(self, client):
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        future = client.call_async(ClearEntireCostmap.Request())
        return self._spin_until(future.done, 2.0) and future.result() is not None

    def _set_model_pose(self, x, y, yaw):
        request = (
            f'name: "{self.args.model_name}", '
            f"position: {{x: {x:.12f}, y: {y:.12f}, z: 0.0}}, "
            "orientation: {"
            f"z: {math.sin(0.5 * yaw):.12f}, "
            f"w: {math.cos(0.5 * yaw):.12f}"
            "}"
        )
        command = [
            "ign",
            "service",
            "-s",
            f"/world/{self.args.world_name}/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        ]
        last_error = ""
        for attempt in range(1, 4):
            try:
                result = subprocess.run(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=5.0,
                )
                if (
                    result.returncode == 0
                    and "true" in result.stdout.lower()
                ):
                    return True
                last_error = (
                    f"attempt={attempt} returncode={result.returncode} "
                    f"stdout={result.stdout.strip()} "
                    f"stderr={result.stderr.strip()}"
                )
            except subprocess.TimeoutExpired:
                last_error = f"attempt={attempt} timed out"
            if attempt < 3:
                time.sleep(0.2)
        self.get_logger().error(f"Gazebo set_pose failed: {last_error}")
        return False

    def _wait_for_pose(self, x, y, yaw):
        def pose_matches():
            if self.latest_odom is None:
                return False
            position = self.latest_odom.pose.pose.position
            actual_yaw = yaw_from_quaternion(
                self.latest_odom.pose.pose.orientation
            )
            return (
                math.hypot(position.x - x, position.y - y)
                <= self.args.pose_tolerance_m
                and abs(normalize_angle(actual_yaw - yaw))
                <= math.radians(self.args.pose_tolerance_deg)
            )

        return self._spin_until(pose_matches, 4.0)

    def _settle(self):
        self._spin_until(
            lambda: False,
            self.args.settle_sec,
        )

    def _reset_metrics(self):
        self.state_sequence = []
        self.state_times = {}
        self.metrics = {
            "observed_initial_lateral_m": math.nan,
            "observed_initial_yaw_error_deg": math.nan,
            "max_abs_lateral_m": 0.0,
            "max_abs_yaw_error_deg": 0.0,
            "minimum_clearance_m": math.inf,
            "crossing_max_abs_lateral_m": 0.0,
            "crossing_max_abs_yaw_error_deg": 0.0,
            "crossing_samples": 0,
            "last_crossing_longitudinal_m": math.nan,
            "max_abs_effective_localization_lateral_error_m": 0.0,
            "max_abs_effective_localization_yaw_error_deg": 0.0,
            "crossing_max_abs_effective_localization_lateral_error_m": 0.0,
            "crossing_max_abs_effective_localization_yaw_error_deg": 0.0,
            "max_abs_lateral_command_error_mps": 0.0,
            "max_abs_angular_command_error_radps": 0.0,
            "crossing_max_abs_lateral_command_error_mps": 0.0,
            "crossing_max_abs_angular_command_error_radps": 0.0,
            "maximum_ground_truth_z_m": -math.inf,
            "max_abs_ground_truth_pitch_deg": 0.0,
            "minimum_roof_clearance_m": math.inf,
        }

    def run_trial(self, lateral_m, yaw_offset_deg, repeat_index):
        self.trial_active = False
        self._call_trigger(self.cancel_client, timeout_sec=2.0)
        self._spin_until(lambda: self.current_state == "IDLE", 2.0)

        approach_longitudinal = (
            -0.5 * self.corridor_length - self.entry_clearance
        )
        axis_x = math.cos(self.corridor_yaw)
        axis_y = math.sin(self.corridor_yaw)
        normal_x = -axis_y
        normal_y = axis_x
        target_x = (
            self.center_x
            + approach_longitudinal * axis_x
            + lateral_m * normal_x
        )
        target_y = (
            self.center_y
            + approach_longitudinal * axis_y
            + lateral_m * normal_y
        )
        # The controller defines heading_error as corridor_yaw - base_yaw.
        # Apply the requested error with the same sign convention.
        target_yaw = self.corridor_yaw - math.radians(yaw_offset_deg)

        case_id = make_case_id(lateral_m, yaw_offset_deg, repeat_index)
        result = {
            "case_id": case_id,
            "requested_lateral_m": lateral_m,
            "requested_yaw_error_deg": yaw_offset_deg,
            "repeat": repeat_index,
            "localization_lateral_noise_std_m":
                self.localization_lateral_noise_std_m,
            "localization_yaw_noise_std_deg":
                self.localization_yaw_noise_std_deg,
            "localization_delay_sec": self.localization_delay_sec,
            "localization_lateral_drift_amplitude_m":
                self.localization_lateral_drift_amplitude_m,
            "localization_yaw_drift_amplitude_deg":
                self.localization_yaw_drift_amplitude_deg,
            "localization_drift_frequency_hz":
                self.localization_drift_frequency_hz,
            "localization_random_seed": self.localization_random_seed,
            "chassis_forward_scale": self.chassis_forward_scale,
            "chassis_lateral_positive_scale":
                self.chassis_lateral_positive_scale,
            "chassis_lateral_negative_scale":
                self.chassis_lateral_negative_scale,
            "chassis_angular_scale": self.chassis_angular_scale,
            "chassis_lateral_time_constant_sec":
                self.chassis_lateral_time_constant_sec,
            "chassis_angular_time_constant_sec":
                self.chassis_angular_time_constant_sec,
            "pose_reset_ok": False,
            "mission_started": False,
            "success": False,
            "final_state": "NOT_STARTED",
            "dog_hole_traversed": False,
            "collision_or_overlap": False,
            "wall_collision_or_overlap": False,
            "roof_collision_or_overlap": False,
            "observed_initial_lateral_m": math.nan,
            "observed_initial_yaw_error_deg": math.nan,
            "max_abs_lateral_m": math.nan,
            "max_abs_yaw_error_deg": math.nan,
            "crossing_max_abs_lateral_m": math.nan,
            "crossing_max_abs_yaw_error_deg": math.nan,
            "minimum_clearance_m": math.nan,
            "max_abs_effective_localization_lateral_error_m": math.nan,
            "max_abs_effective_localization_yaw_error_deg": math.nan,
            "crossing_max_abs_effective_localization_lateral_error_m":
                math.nan,
            "crossing_max_abs_effective_localization_yaw_error_deg":
                math.nan,
            "max_abs_lateral_command_error_mps": math.nan,
            "max_abs_angular_command_error_radps": math.nan,
            "crossing_max_abs_lateral_command_error_mps": math.nan,
            "crossing_max_abs_angular_command_error_radps": math.nan,
            "maximum_ground_truth_z_m": math.nan,
            "max_abs_ground_truth_pitch_deg": math.nan,
            "minimum_roof_clearance_m": math.nan,
            "alignment_sec": math.nan,
            "crossing_sec": math.nan,
            "traversal_sec": math.nan,
            "mission_sec": math.nan,
            "state_sequence": "",
            "failure_reason": "",
        }

        if not self._set_model_pose(target_x, target_y, target_yaw):
            result["failure_reason"] = "Gazebo set_pose failed"
            return result
        if not self._wait_for_pose(target_x, target_y, target_yaw):
            result["failure_reason"] = "ground-truth pose did not settle"
            return result
        result["pose_reset_ok"] = True

        self._clear_costmap(self.local_clear_client)
        self._clear_costmap(self.global_clear_client)
        self._settle()

        self._reset_metrics()
        self.trial_active = True
        mission_start = time.monotonic()
        started, message = self._call_trigger(
            self.start_client,
            timeout_sec=5.0,
        )
        result["mission_started"] = started
        if not started:
            self.trial_active = False
            result["failure_reason"] = f"mission start rejected: {message}"
            return result

        completed = self._spin_until(
            lambda: self.current_state in ("FINISHED", "FAILED"),
            self.args.trial_timeout_sec,
        )
        mission_end = time.monotonic()
        result["final_state"] = (
            self.current_state if completed else "TIMEOUT"
        )
        result["mission_sec"] = mission_end - mission_start

        self.trial_active = False
        result.update(self.metrics)
        if math.isinf(result["minimum_clearance_m"]):
            result["minimum_clearance_m"] = math.nan
        if math.isinf(result["maximum_ground_truth_z_m"]):
            result["maximum_ground_truth_z_m"] = math.nan
        if math.isinf(result["minimum_roof_clearance_m"]):
            result["minimum_roof_clearance_m"] = math.nan
        result["wall_collision_or_overlap"] = (
            not math.isnan(result["minimum_clearance_m"])
            and result["minimum_clearance_m"] < 0.0
        )
        result["roof_collision_or_overlap"] = (
            not math.isnan(result["minimum_roof_clearance_m"])
            and result["minimum_roof_clearance_m"] < -0.002
        )
        result["collision_or_overlap"] = (
            result["wall_collision_or_overlap"]
            or result["roof_collision_or_overlap"]
        )

        aligning = self.state_times.get("ALIGNING")
        crossing = self.state_times.get("CROSSING")
        exiting = self.state_times.get("EXITING")
        if aligning is not None and crossing is not None:
            result["alignment_sec"] = crossing - aligning
        if crossing is not None and exiting is not None:
            result["crossing_sec"] = exiting - crossing
        if aligning is not None and exiting is not None:
            result["traversal_sec"] = exiting - aligning
        result["state_sequence"] = ">".join(self.state_sequence)
        result["dog_hole_traversed"] = "CROSSING" in self.state_sequence
        result["success"] = (
            result["final_state"] == "FINISHED"
            and result["dog_hole_traversed"]
            and not result["collision_or_overlap"]
        )
        if not result["success"] and not result["failure_reason"]:
            result["failure_reason"] = (
                "dog-hole crossing was bypassed"
                if result["final_state"] == "FINISHED"
                and not result["dog_hole_traversed"]
                else result["final_state"]
            )

        self._call_trigger(self.cancel_client, timeout_sec=2.0)
        self._spin_until(lambda: self.current_state == "IDLE", 2.0)
        return result


RESULT_FIELDS = [
    "case_id",
    "requested_lateral_m",
    "requested_yaw_error_deg",
    "repeat",
    "localization_lateral_noise_std_m",
    "localization_yaw_noise_std_deg",
    "localization_delay_sec",
    "localization_lateral_drift_amplitude_m",
    "localization_yaw_drift_amplitude_deg",
    "localization_drift_frequency_hz",
    "localization_random_seed",
    "chassis_forward_scale",
    "chassis_lateral_positive_scale",
    "chassis_lateral_negative_scale",
    "chassis_angular_scale",
    "chassis_lateral_time_constant_sec",
    "chassis_angular_time_constant_sec",
    "pose_reset_ok",
    "mission_started",
    "success",
    "final_state",
    "dog_hole_traversed",
    "collision_or_overlap",
    "wall_collision_or_overlap",
    "roof_collision_or_overlap",
    "observed_initial_lateral_m",
    "observed_initial_yaw_error_deg",
    "max_abs_lateral_m",
    "max_abs_yaw_error_deg",
    "crossing_max_abs_lateral_m",
    "crossing_max_abs_yaw_error_deg",
    "minimum_clearance_m",
    "crossing_samples",
    "last_crossing_longitudinal_m",
    "max_abs_effective_localization_lateral_error_m",
    "max_abs_effective_localization_yaw_error_deg",
    "crossing_max_abs_effective_localization_lateral_error_m",
    "crossing_max_abs_effective_localization_yaw_error_deg",
    "max_abs_lateral_command_error_mps",
    "max_abs_angular_command_error_radps",
    "crossing_max_abs_lateral_command_error_mps",
    "crossing_max_abs_angular_command_error_radps",
    "maximum_ground_truth_z_m",
    "max_abs_ground_truth_pitch_deg",
    "minimum_roof_clearance_m",
    "alignment_sec",
    "crossing_sec",
    "traversal_sec",
    "mission_sec",
    "state_sequence",
    "failure_reason",
]


def parse_args():
    default_config = (
        Path(get_package_share_directory("rm_dog_hole"))
        / "config"
        / "dog_hole_sim.yaml"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Run dog-hole centerline capture tests. Start dog_hole_sim.launch.py "
            "with auto_start:=false before running this program."
        )
    )
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Dog-hole YAML used by the running simulation.",
    )
    parser.add_argument(
        "--mode",
        choices=("single", "full"),
        default="full",
        help="single runs only zero/one-axis cases; full runs the cross product.",
    )
    parser.add_argument(
        "--lateral-cm",
        nargs="+",
        type=float,
        default=[0.0, -2.0, 2.0, -5.0, 5.0, -10.0, 10.0],
    )
    parser.add_argument(
        "--yaw-deg",
        nargs="+",
        type=float,
        default=[0.0, -2.0, 2.0, -5.0, 5.0, -10.0, 10.0],
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--trial-timeout-sec", type=float, default=30.0)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--pose-tolerance-m", type=float, default=0.01)
    parser.add_argument("--pose-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--world-name", default="phase1_omni")
    parser.add_argument("--model-name", default="rm_sentry_2027")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Defaults to /tmp/rm2027_dog_hole_robustness/<timestamp>.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Append only missing cases to an existing capture_matrix.csv.",
    )
    parser.add_argument("--stop-on-failure", action="store_true")
    return parser.parse_args()


def make_cases(args):
    lateral_values = [value / 100.0 for value in args.lateral_cm]
    yaw_values = list(args.yaw_deg)
    if args.mode == "full":
        return [
            (lateral, yaw)
            for lateral in lateral_values
            for yaw in yaw_values
        ]

    cases = []
    for lateral in lateral_values:
        cases.append((lateral, 0.0))
    for yaw in yaw_values:
        cases.append((0.0, yaw))
    deduplicated = []
    for case in cases:
        if case not in deduplicated:
            deduplicated.append(case)
    return deduplicated


def main():
    args = parse_args()
    if args.repeat <= 0:
        raise ValueError("--repeat must be positive")
    config_path = Path(args.config)
    config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config = config_data["dog_hole_manager"]["ros__parameters"]

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(
        args.output_dir
        or f"/tmp/rm2027_dog_hole_robustness/{timestamp}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "experiment.json").write_text(
        json.dumps(vars(args), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    cases = make_cases(args)
    total = len(cases) * args.repeat
    results_path = output_dir / "capture_matrix.csv"
    completed_case_ids = set()
    append_results = args.resume and results_path.exists()
    if append_results:
        with results_path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != RESULT_FIELDS:
                raise ValueError(
                    "Existing capture_matrix.csv has an incompatible schema"
                )
            completed_case_ids = {
                row["case_id"] for row in reader if row.get("case_id")
            }

    rclpy.init()
    runner = CaptureMatrixRunner(args, config)
    try:
        file_mode = "a" if append_results else "w"
        with results_path.open(
            file_mode, newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
            if not append_results:
                writer.writeheader()
            completed = len(completed_case_ids)
            stop_requested = False
            for lateral, yaw in cases:
                for repeat_index in range(1, args.repeat + 1):
                    case_id = make_case_id(lateral, yaw, repeat_index)
                    if case_id in completed_case_ids:
                        continue
                    completed += 1
                    runner.get_logger().info(
                        f"[{completed}/{total}] lateral={lateral * 100:+.1f} cm "
                        f"yaw={yaw:+.1f} deg"
                    )
                    result = runner.run_trial(
                        lateral,
                        yaw,
                        repeat_index,
                    )
                    writer.writerow(result)
                    stream.flush()
                    runner.get_logger().info(
                        f"{result['case_id']} success={result['success']} "
                        f"clearance={result['minimum_clearance_m']:.4f} m "
                        f"traversal={result['traversal_sec']:.2f} s "
                        f"state={result['final_state']}"
                    )
                    if args.stop_on_failure and not result["success"]:
                        stop_requested = True
                        break
                if stop_requested:
                    break
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    with results_path.open(newline="", encoding="utf-8") as stream:
        results = list(csv.DictReader(stream))
    successes = sum(
        1 for result in results if result["success"].lower() == "true"
    )
    valid_clearances = [
        float(result["minimum_clearance_m"])
        for result in results
        if result["minimum_clearance_m"]
        and not math.isnan(float(result["minimum_clearance_m"]))
    ]
    valid_traversal_times = [
        float(result["traversal_sec"])
        for result in results
        if result["traversal_sec"]
        and not math.isnan(float(result["traversal_sec"]))
    ]
    valid_roof_clearances = [
        float(result["minimum_roof_clearance_m"])
        for result in results
        if result["minimum_roof_clearance_m"]
        and not math.isnan(float(result["minimum_roof_clearance_m"]))
    ]
    summary = {
        "cases_completed": len(results),
        "successes": successes,
        "success_rate": successes / len(results) if results else 0.0,
        "minimum_clearance_m": (
            min(valid_clearances) if valid_clearances else None
        ),
        "maximum_traversal_sec": (
            max(valid_traversal_times) if valid_traversal_times else None
        ),
        "minimum_roof_clearance_m": (
            min(valid_roof_clearances) if valid_roof_clearances else None
        ),
        "failed_cases": [
            result["case_id"]
            for result in results
            if result["success"].lower() != "true"
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"results={results_path}")


if __name__ == "__main__":
    main()
