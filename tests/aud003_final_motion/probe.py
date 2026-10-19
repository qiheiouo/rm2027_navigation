#!/usr/bin/env python3
"""No-hardware AUD-003 probe that observes the encoded V2 chassis frames."""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rm_competition_interfaces.msg import (
    ChassisMode,
    LocalizationStatus,
    ManagedIngressStatus,
    V2MotionGuardState,
    V2MotionTestLease,
)
from std_msgs.msg import UInt8MultiArray


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return value


class Probe(Node):
    def __init__(self, output):
        super().__init__("aud003_final_motion_probe")
        self.output = output
        self.processes = []
        self.logs = []
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.rx_pub = self.create_publisher(UInt8MultiArray, "/serial/v2/mock_rx", 100)
        self.loc_pub = self.create_publisher(
            LocalizationStatus, "/localization/status", reliable
        )
        self.mode_pub = self.create_publisher(ChassisMode, "/chassis/mode", reliable)
        self.lease_pub = self.create_publisher(
            V2MotionTestLease, "/motion/v2_test/lease", reliable
        )
        self.source_pub = self.create_publisher(Twist, "/aud003/source/cmd_vel", reliable)
        self.injector_pub = None
        self.create_subscription(
            UInt8MultiArray, "/serial/v2/mock_tx", self._on_frame, 100
        )
        self.create_subscription(
            V2MotionGuardState, "/motion/v2_guard/state", self._on_guard, reliable
        )
        self.create_subscription(
            ManagedIngressStatus,
            "/aud003/ingress/status",
            self._on_ingress,
            reliable,
        )
        self.frames = []
        self.guard = None
        self.guard_history = []
        self.ingress = None
        self.sequence = 0
        self.loc_sequence = 0
        self.lease_sequence = 0
        self.boot_id = 0x20270018
        self.started = time.monotonic()
        self.loc_started = None
        self.producer_id = uuid.uuid4().hex
        self.owner_id = uuid.uuid4().hex
        self.session_id = uuid.uuid4().hex
        self.lease_template = None
        self.last_lease = None
        self.loc_valid = True
        self.mode_valid = True
        self.heartbeat_on = True
        self.lease_on = True
        self.lease_enabled = True
        self.command_on = True
        self.inject_on = False
        self.command = Twist()
        self.command.linear.x = 0.20
        self.command.linear.y = 0.07
        self.command.angular.z = 0.30
        self.last_tick = 0.0
        self.last_inject = 0.0

    def launch(self, package, executable, arguments=(), label=None):
        prefix = Path(os.environ["AUD003_INSTALL"])
        binary = prefix / package / "lib" / package / executable
        log = (self.output / ((label or executable) + ".log")).open("w")
        process = subprocess.Popen(
            [str(binary), "--ros-args", *arguments],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        self.processes.append(process)
        self.logs.append(log)
        return process

    def launch_serial(self):
        self.guard = None
        return self.launch(
            "rm_serial_driver",
            "competition_v2_transport_node",
            (
                "-p",
                "dry_run:=true",
                "-p",
                "motion_test_mode:=true",
                "-p",
                "required_remote_capabilities:=1",
                "-p",
                "cmd_vel_timeout_sec:=0.2",
            ),
            "v2_" + str(len(self.processes)),
        )

    def launch_ingress(self, command_topic):
        gid = self._single_gid("/aud003/source/cmd_vel")
        return self.launch(
            "rm_chassis_interface",
            "managed_motion_ingress_node",
            (
                "-p",
                "input_topic:=/aud003/source/cmd_vel",
                "-p",
                "output_topic:=" + command_topic,
                "-p",
                "status_topic:=/aud003/ingress/status",
                "-p",
                "allowed_gids:=['" + gid.hex() + "']",
            ),
            "ingress_" + str(len(self.processes)),
        )

    def _single_gid(self, topic):
        infos = self.get_publishers_info_by_topic(topic)
        if len(infos) != 1:
            raise RuntimeError(f"{topic} expected one publisher, got {len(infos)}")
        return bytes(infos[0].endpoint_gid)

    def _on_guard(self, message):
        self.guard = message
        self.guard_history.append(
            (time.monotonic(), int(message.phase), message.reason)
        )

    def _on_ingress(self, message):
        self.ingress = message

    def _on_frame(self, message):
        data = bytes(message.data)
        if len(data) < 10 or data[:3] != b"RM\x02":
            raise AssertionError("invalid V2 frame envelope")
        payload_size = struct.unpack_from("<H", data, 4)[0]
        if len(data) != payload_size + 10:
            raise AssertionError("invalid V2 frame length")
        if crc16(data[2:-2]) != struct.unpack_from("<H", data, len(data) - 2)[0]:
            raise AssertionError("invalid V2 CRC")
        if data[3] == 1:
            self.frames.append(
                (time.monotonic(), struct.unpack("<fffB", data[8:-2]))
            )

    def _publish_inputs(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self.loc_started is None:
            self.loc_started = copy.deepcopy(stamp)
        self.loc_sequence += 1
        localization = LocalizationStatus()
        localization.header.stamp = stamp
        localization.producer_id = self.producer_id
        localization.started_at = self.loc_started
        localization.sequence = self.loc_sequence
        localization.valid = self.loc_valid
        localization.valid_until = (now + Duration(seconds=0.4)).to_msg()
        localization.pose.header.stamp = stamp
        localization.pose.header.frame_id = "map"
        localization.pose.pose.pose.orientation.w = 1.0
        self.loc_pub.publish(localization)

        mode = ChassisMode()
        mode.header.stamp = stamp
        mode.online = self.mode_valid
        mode.autonomous_enabled = self.mode_valid
        mode.emergency_stop = not self.mode_valid
        mode.mode = (
            ChassisMode.MODE_AUTONOMOUS if self.mode_valid else ChassisMode.MODE_ESTOP
        )
        self.mode_pub.publish(mode)

        if self.heartbeat_on:
            uptime_ms = int((time.monotonic() - self.started) * 1000.0)
            payload = struct.pack("<IIIB", uptime_ms, self.boot_id, 1, 1)
            header = struct.pack(
                "<BBBBHH", 0x52, 0x4D, 2, 3, len(payload), self.sequence & 0xFFFF
            )
            self.sequence += 1
            packet = header + payload
            heartbeat = UInt8MultiArray()
            heartbeat.data = list(packet + struct.pack("<H", crc16(packet[2:])))
            self.rx_pub.publish(heartbeat)

        if self.command_on:
            self.source_pub.publish(self.command)

        if self.lease_on and self.lease_template is not None:
            lease = copy.deepcopy(self.lease_template)
            self.lease_sequence += 1
            lease.header.stamp = stamp
            lease.sequence = self.lease_sequence
            lease.valid_until = (now + Duration(seconds=0.4)).to_msg()
            lease.enabled = self.lease_enabled
            self.last_lease = copy.deepcopy(lease)
            self.lease_pub.publish(lease)

    def spin(self, duration, predicate=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if time.monotonic() - self.last_tick >= 0.04:
                self._publish_inputs()
                self.last_tick = time.monotonic()
            if (
                self.inject_on
                and self.injector_pub is not None
                and time.monotonic() - self.last_inject >= 0.08
                and time.monotonic() - self.last_tick >= 0.02
            ):
                injected = Twist()
                injected.linear.y = -0.45
                injected.angular.x = 7.0
                self.injector_pub.publish(injected)
                self.last_inject = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.005)
            if predicate is not None and predicate():
                return True
        return predicate is None or predicate()

    def wait_guard(self):
        if not self.spin(4.0, lambda: self.guard is not None):
            raise RuntimeError("V2 guard did not publish state")
        return self.guard

    def enroll(self):
        state = self.wait_guard()
        ingress = self.launch_ingress(state.command_topic)
        if not self.spin(
            4.0,
            lambda: self.ingress is not None
            and self.ingress.active
            and self.ingress.accepted_count > 0
            and self.guard is not None
            and self.guard.observed_command_count > 0,
        ):
            raise RuntimeError("managed ingress did not reach the V2 private command topic")
        command_gid = self._single_gid(state.command_topic)
        lease = V2MotionTestLease()
        lease.endpoint_instance = state.endpoint_instance
        lease.owner_instance = self.owner_id
        lease.session_id = self.session_id
        lease.command_publisher_gid = list(command_gid)
        self.lease_template = lease
        if not self.spin(
            4.0,
            lambda: self.guard is not None
            and self.guard.phase == V2MotionGuardState.PERMITTED,
        ):
            raise RuntimeError("V2 guard did not admit the managed command source")
        if not self.spin(1.0, lambda: self.snapshot()["nonzero"] >= 20):
            raise RuntimeError("authorized nonzero command did not reach encoded V2 frames")
        return ingress

    @staticmethod
    def is_nonzero(frame):
        return bool(frame[3] & 1) and any(abs(value) > 1.0e-4 for value in frame[:3])

    def snapshot(self):
        cutoff = time.monotonic() - 0.4
        frames = [frame for stamp, frame in self.frames if stamp >= cutoff]
        return {
            "frames": len(frames),
            "nonzero": sum(self.is_nonzero(frame) for frame in frames),
            "last_frame": list(frames[-1]) if frames else None,
            "phase": int(self.guard.phase) if self.guard else None,
            "reason": self.guard.reason if self.guard else None,
            "ingress_accepted": int(self.ingress.accepted_count)
            if self.ingress
            else None,
            "ingress_rejected": int(self.ingress.rejected_count)
            if self.ingress
            else None,
        }

    def expect_blocked_zero(self):
        if not self.spin(
            2.0,
            lambda: self.guard is not None
            and self.guard.phase == V2MotionGuardState.BLOCKED,
        ):
            raise AssertionError("guard did not latch BLOCKED")
        self.spin(0.6)
        snapshot = self.snapshot()
        assert snapshot["frames"] >= 20, snapshot
        assert snapshot["nonzero"] == 0, snapshot
        assert snapshot["last_frame"][3] == 0, snapshot
        return snapshot

    def stop(self, process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

    def close(self):
        for process in reversed(self.processes):
            self.stop(process)
        for log in self.logs:
            log.close()


def run_case(probe, case):
    result = {
        "case": case,
        "scope": "ROS processes, virtual velocity/localization/mode/MCU; no device",
    }
    serial = probe.launch_serial()
    probe.wait_guard()
    if case == "unauthorized":
        probe.spin(1.0)
        snapshot = probe.snapshot()
        assert snapshot["frames"] >= 20 and snapshot["nonzero"] == 0, snapshot
        assert snapshot["last_frame"][3] == 0, snapshot
        assert snapshot["phase"] == V2MotionGuardState.WAITING, snapshot
        result["unauthorized"] = snapshot
    else:
        ingress = probe.enroll()
        result["healthy"] = probe.snapshot()
        if case == "normal":
            probe.spin(1.5)
            stable = probe.snapshot()
            assert stable["nonzero"] >= 20, stable
            assert all(
                phase != V2MotionGuardState.BLOCKED
                for _, phase, _ in probe.guard_history
            )
            result["stable"] = stable
        elif case == "mission_cancel":
            probe.lease_enabled = False
            result["blocked"] = probe.expect_blocked_zero()
        elif case == "localization_lost":
            probe.loc_valid = False
            result["blocked"] = probe.expect_blocked_zero()
        elif case == "authority_timeout":
            probe.lease_on = False
            result["blocked"] = probe.expect_blocked_zero()
        elif case == "source_timeout":
            probe.command_on = False
            result["blocked"] = probe.expect_blocked_zero()
        elif case == "source_nonfinite":
            probe.command.linear.x = math.nan
            result["blocked"] = probe.expect_blocked_zero()
            assert probe.ingress is not None and not probe.ingress.active
        elif case == "injection":
            probe.injector_pub = probe.create_publisher(
                Twist,
                "/aud003/source/cmd_vel",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            )
            probe.inject_on = True
            before = probe.ingress.rejected_count
            probe.spin(1.0)
            injected = probe.snapshot()
            assert injected["nonzero"] >= 20, injected
            assert injected["ingress_rejected"] > before, injected
            recent = [
                frame
                for stamp, frame in probe.frames
                if stamp >= time.monotonic() - 0.8
            ]
            assert all(abs(frame[1] + 0.45) > 1.0e-3 for frame in recent), recent[-3:]
            result["injection_filtered"] = injected
        elif case == "recovery":
            old_endpoint = probe.guard.endpoint_instance
            old_topic = probe.guard.command_topic
            old_lease = copy.deepcopy(probe.last_lease)
            probe.lease_enabled = False
            result["old_blocked"] = probe.expect_blocked_zero()
            probe.stop(serial)
            probe.lease_enabled = True
            probe.lease_template = old_lease
            probe.guard = None
            serial = probe.launch_serial()
            if not probe.spin(
                4.0,
                lambda: probe.guard is not None
                and probe.guard.endpoint_instance != old_endpoint,
            ):
                raise RuntimeError("restarted V2 guard did not publish a new endpoint")
            new_state = probe.guard
            assert new_state.endpoint_instance != old_endpoint
            assert new_state.command_topic != old_topic
            probe.spin(0.8)
            stale = probe.snapshot()
            assert stale["phase"] == V2MotionGuardState.WAITING, stale
            assert stale["nonzero"] == 0 and stale["last_frame"][3] == 0, stale
            result["old_goal_rejected"] = stale
            probe.stop(ingress)
            probe.ingress = None
            probe.lease_template = None
            probe.session_id = uuid.uuid4().hex
            probe.lease_sequence = 0
            probe.enroll()
            result["new_authorization"] = probe.snapshot()
        else:
            raise ValueError(case)

        if case not in ("normal", "injection", "recovery"):
            probe.loc_valid = probe.mode_valid = probe.lease_on = True
            probe.lease_enabled = probe.command_on = probe.heartbeat_on = True
            probe.command = Twist()
            probe.command.linear.x = 0.20
            probe.session_id = uuid.uuid4().hex
            if probe.lease_template is not None:
                probe.lease_template.session_id = probe.session_id
            probe.spin(0.8)
            result["no_in_process_rearm"] = probe.snapshot()
            assert result["no_in_process_rearm"]["nonzero"] == 0
            assert (
                result["no_in_process_rearm"]["phase"]
                == V2MotionGuardState.BLOCKED
            )
    result["pass"] = True
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case",
        required=True,
        choices=(
            "unauthorized",
            "normal",
            "mission_cancel",
            "localization_lost",
            "authority_timeout",
            "source_timeout",
            "source_nonfinite",
            "injection",
            "recovery",
        ),
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    probe = Probe(output)
    try:
        result = run_case(probe, args.case)
    except Exception as error:
        result = {"case": args.case, "pass": False, "error": repr(error)}
        raise
    finally:
        probe.close()
        probe.destroy_node()
        rclpy.shutdown()
        (output / "result.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n"
        )
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
