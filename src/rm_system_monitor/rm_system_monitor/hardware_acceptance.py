"""Bounded, read-only evidence capture for the new-car hardware checklist."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import logging
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from typing import Any

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message
import yaml


PASS = "PASS"
FAIL = "FAIL"
BLOCKED = "BLOCKED"
WARN = "WARN"


def _stamp_ns(stamp: Any) -> int | None:
    if stamp is None or not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        return None
    if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
        return -1
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _field_value(message: Any, path: str) -> Any:
    value = message
    for part in path.split("."):
        if not hasattr(value, part):
            return None
        value = getattr(value, part)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _header_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    return _stamp_ns(getattr(header, "stamp", None))


def _gid_hex(value: Any) -> str:
    try:
        encoded = bytes(value).hex()
        return encoded or "unavailable"
    except (TypeError, ValueError):
        return "unknown"


def _nested_value(mapping: dict[str, Any], path: str) -> Any:
    value: Any = mapping
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _command(args: list[str], cwd: Path) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            args,
            cwd=cwd,
            text=True,
            capture_output=True,
            timeout=8,
            check=False,
        )
        return {
            "command": args,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": args, "error": repr(error)}


@dataclass
class TopicStats:
    count: int = 0
    first_receive: float | None = None
    last_receive: float | None = None
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    stamp_regressions: int = 0
    invalid_stamps: int = 0
    fields: dict[str, Any] = field(default_factory=dict)
    publisher_gids: set[str] = field(default_factory=set)

    def record(
        self,
        message: Any,
        publisher_gid: str,
        field_paths: list[str],
        receive_time: float,
    ) -> None:
        stamp = _header_stamp_ns(message)
        self.count += 1
        self.first_receive = receive_time if self.first_receive is None else self.first_receive
        self.last_receive = receive_time
        self.publisher_gids.add(publisher_gid)
        if stamp == -1:
            self.invalid_stamps += 1
        elif stamp is not None:
            if self.first_stamp_ns is None:
                self.first_stamp_ns = stamp
            if self.last_stamp_ns is not None and stamp < self.last_stamp_ns:
                self.stamp_regressions += 1
            self.last_stamp_ns = stamp
        self.fields = {path: _field_value(message, path) for path in field_paths}

    def rate_hz(self) -> float:
        if self.count < 2 or self.first_receive is None or self.last_receive is None:
            return 0.0
        duration = self.last_receive - self.first_receive
        return 0.0 if duration <= 0.0 else (self.count - 1) / duration


def evaluate_topic(
    rule: dict[str, Any],
    stats: TopicStats,
    publishers: list[dict[str, Any]],
    graph_types: list[str],
    now_ns: int,
) -> dict[str, Any]:
    required = bool(rule.get("required", True))
    missing_status = BLOCKED if required else WARN
    reasons: list[str] = []
    status = PASS
    expected_type = rule["type"]
    if expected_type not in graph_types:
        status = missing_status
        reasons.append(f"type missing: expected {expected_type}, observed {graph_types}")
    if not publishers:
        status = missing_status
        reasons.append("no publisher")
    maximum = int(rule.get("max_publishers", 1))
    if len(publishers) > maximum:
        status = FAIL
        reasons.append(f"publisher count {len(publishers)} exceeds {maximum}")
    if stats.count == 0:
        status = missing_status
        reasons.append("no sample received")
    minimum_rate = float(rule.get("min_rate_hz", 0.0))
    rate = stats.rate_hz()
    if stats.count > 0 and minimum_rate > 0.0 and rate < minimum_rate:
        status = FAIL
        reasons.append(f"rate {rate:.3f} Hz below {minimum_rate:.3f} Hz")
    require_stamp = bool(rule.get("require_stamp", True))
    if require_stamp and stats.count > 0 and stats.last_stamp_ns is None:
        status = FAIL
        reasons.append("required header stamp missing")
    if stats.invalid_stamps:
        status = FAIL
        reasons.append(f"invalid stamps: {stats.invalid_stamps}")
    if stats.stamp_regressions:
        status = FAIL
        reasons.append(f"timestamp regressions: {stats.stamp_regressions}")
    age = None
    if stats.last_stamp_ns is not None:
        age = (now_ns - stats.last_stamp_ns) * 1.0e-9
        maximum_age = rule.get("max_age_sec")
        if age < -0.1:
            status = FAIL
            reasons.append(f"latest stamp is {-age:.3f} s in the future")
        elif maximum_age is not None and age > float(maximum_age):
            status = FAIL
            reasons.append(f"source age {age:.3f} s exceeds {maximum_age} s")
    if stats.count > 0:
        for path, expected in rule.get("fields", {}).items():
            observed = stats.fields.get(path)
            if observed != expected:
                status = FAIL
                reasons.append(f"{path}={observed!r}, expected {expected!r}")
    return {
        "kind": "topic",
        "step": int(rule["step"]),
        "name": rule["name"],
        "required": required,
        "status": status,
        "reasons": reasons,
        "metrics": {
            "samples": stats.count,
            "rate_hz": rate,
            "latest_source_age_sec": age,
            "timestamp_regressions": stats.stamp_regressions,
            "fields": stats.fields,
            "publisher_gids_seen": sorted(stats.publisher_gids),
            "publishers": publishers,
            "graph_types": graph_types,
        },
    }


def collect_preflight(plan: dict[str, Any], workspace: Path) -> dict[str, Any]:
    config = plan.get("preflight", {})
    checks: list[dict[str, Any]] = []
    for relative in config.get("files", []):
        path = workspace / relative
        exists = path.is_file()
        checks.append(
            {
                "kind": "file",
                "name": relative,
                "status": PASS if exists else FAIL,
                "sha256": _sha256(path) if exists else None,
            }
        )
    for package in config.get("packages", []):
        try:
            prefix = get_package_prefix(package)
            status, reason = PASS, None
        except Exception as error:  # ament raises package-specific exceptions
            prefix, status, reason = None, FAIL, repr(error)
        checks.append(
            {
                "kind": "package",
                "name": package,
                "status": status,
                "prefix": prefix,
                "reason": reason,
            }
        )
    manifest_path = workspace / config.get("candidate_manifest", "")
    manifest: dict[str, Any] = {}
    if manifest_path.is_file():
        manifest = yaml.safe_load(manifest_path.read_text()) or {}
    pending = []
    for gate in config.get("pending_gates", []):
        actual = _nested_value(manifest, gate["path"])
        expected = gate["expected"]
        if expected == "not_null":
            ready = actual not in (None, "", [], {})
        elif expected == "Software_Go":
            ready = isinstance(actual, str) and actual.startswith("Software_Go")
        else:
            ready = actual == expected
        entry = {
            "kind": "candidate_gate",
            "name": gate["path"],
            "status": PASS if ready else BLOCKED,
            "actual": actual,
            "expected": expected,
            "blocks_before_step": int(gate["blocks_before_step"]),
            "reason": gate["reason"],
        }
        checks.append(entry)
        if not ready:
            pending.append(entry)
    hard_failures = [item for item in checks if item["status"] == FAIL]
    return {
        "checks": checks,
        "candidate_manifest": manifest,
        "next_blocked_step": min(
            (item["blocks_before_step"] for item in pending), default=None
        ),
        "decision": "STOP" if hard_failures else ("MODIFY" if pending else "GO"),
    }


def collect_versions(workspace: Path) -> dict[str, Any]:
    return {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "workspace": str(workspace),
        "platform": platform.platform(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "git_head": _command(["git", "rev-parse", "HEAD"], workspace),
        "git_status": _command(["git", "status", "--short", "--branch"], workspace),
        "git_submodules": _command(["git", "submodule", "status", "--recursive"], workspace),
    }


class AcceptanceProbe(Node):
    def __init__(self, plan: dict[str, Any], steps: set[int]):
        super().__init__("new_car_hardware_acceptance_probe")
        self.plan = plan
        self.steps = steps
        self.rules = [item for item in plan.get("topics", []) if int(item["step"]) in steps]
        self.stats = {item["name"]: TopicStats() for item in self.rules}
        self.type_errors: dict[str, str] = {}
        self.tf_samples: dict[tuple[str, str], dict[str, TopicStats]] = {}
        self._acceptance_subscriptions = []
        for rule in self.rules:
            try:
                message_type = get_message(rule["type"])
            except Exception as error:
                self.type_errors[rule["name"]] = repr(error)
                continue
            required_fields = list(rule.get("fields", {}).keys())
            capture_fields = list(rule.get("capture_fields", []))
            field_paths = list(dict.fromkeys(required_fields + capture_fields))
            callback = self._topic_callback(rule["name"], field_paths)
            self._acceptance_subscriptions.append(
                self.create_subscription(
                    message_type,
                    rule["name"],
                    callback,
                    qos_profile_sensor_data,
                )
            )
        if any(int(item["step"]) in steps for item in plan.get("tf", [])):
            tf_type = get_message("tf2_msgs/msg/TFMessage")
            self._acceptance_subscriptions.append(
                self.create_subscription(
                    tf_type, "/tf", self._tf_callback, qos_profile_sensor_data
                )
            )
            static_qos = QoSProfile(depth=100)
            static_qos.reliability = ReliabilityPolicy.RELIABLE
            static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._acceptance_subscriptions.append(
                self.create_subscription(tf_type, "/tf_static", self._tf_callback, static_qos)
            )

    def _topic_callback(self, topic: str, field_paths: list[str]):
        # Humble invokes Python subscriptions with the message only; Jazzy may
        # also provide MessageInfo. Graph inventory remains the authoritative
        # endpoint record when per-sample metadata is unavailable.
        def callback(message: Any, info: Any = None) -> None:
            self.stats[topic].record(
                message,
                _gid_hex(getattr(info, "publisher_gid", [])),
                field_paths,
                time.monotonic(),
            )

        return callback

    def _tf_callback(self, message: Any, info: Any = None) -> None:
        gid = _gid_hex(getattr(info, "publisher_gid", []))
        for transform in message.transforms:
            edge = (transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/"))
            owners = self.tf_samples.setdefault(edge, {})
            stats = owners.setdefault(gid, TopicStats())
            stats.record(transform, gid, [], time.monotonic())

    @staticmethod
    def _endpoint(endpoint: Any) -> dict[str, Any]:
        return {
            "node_name": endpoint.node_name,
            "node_namespace": endpoint.node_namespace,
            "topic_type": endpoint.topic_type,
            "gid": _gid_hex(endpoint.endpoint_gid),
        }

    def _topic_inventory(self) -> dict[str, Any]:
        graph = dict(self.get_topic_names_and_types())
        result = {}
        for rule in self.rules:
            topic = rule["name"]
            result[topic] = {
                "types": graph.get(topic, []),
                "publishers": [
                    self._endpoint(item) for item in self.get_publishers_info_by_topic(topic)
                ],
            }
        return result

    def _action_inventory(self) -> dict[str, list[str]]:
        actions: dict[str, set[str]] = {}
        for name, namespace in self.get_node_names_and_namespaces():
            try:
                entries = get_action_server_names_and_types_by_node(
                    self, name, namespace
                )
            except Exception:
                continue
            for action_name, action_types in entries:
                canonical = action_name if action_name.startswith("/") else "/" + action_name
                actions.setdefault(canonical, set()).update(action_types)
        return {name: sorted(types) for name, types in actions.items()}

    def report(self) -> dict[str, Any]:
        now_ns = self.get_clock().now().nanoseconds
        inventory = self._topic_inventory()
        checks = []
        for rule in self.rules:
            topic = rule["name"]
            check = evaluate_topic(
                rule,
                self.stats[topic],
                inventory[topic]["publishers"],
                inventory[topic]["types"],
                now_ns,
            )
            if topic in self.type_errors:
                check["status"] = BLOCKED if rule.get("required", True) else WARN
                check["reasons"].append("message type unavailable: " + self.type_errors[topic])
            checks.append(check)
        by_topic = {item["name"]: item for item in checks}
        for group in self.plan.get("groups", []):
            if int(group["step"]) not in self.steps:
                continue
            members = [by_topic[name] for name in group["topics"] if name in by_topic]
            passed = [item["name"] for item in members if item["status"] == PASS]
            checks.append(
                {
                    "kind": "any_of",
                    "step": int(group["step"]),
                    "name": group["name"],
                    "status": PASS if passed else BLOCKED,
                    "passed_topics": passed,
                    "topics": group["topics"],
                }
            )
        actions = self._action_inventory()
        for rule in self.plan.get("actions", []):
            if int(rule["step"]) not in self.steps:
                continue
            observed = actions.get(rule["name"], [])
            checks.append(
                {
                    "kind": "action",
                    "step": int(rule["step"]),
                    "name": rule["name"],
                    "status": PASS if rule["type"] in observed else BLOCKED,
                    "expected_type": rule["type"],
                    "observed_types": observed,
                }
            )
        tf_publishers = {}
        for topic in ("/tf", "/tf_static"):
            tf_publishers.update(
                {
                    item["gid"]: item
                    for item in (
                        self._endpoint(endpoint)
                        for endpoint in self.get_publishers_info_by_topic(topic)
                    )
                }
            )
        tf_edges = {}
        for edge, owners in self.tf_samples.items():
            key = f"{edge[0]}->{edge[1]}"
            tf_edges[key] = {
                "owners": [tf_publishers.get(gid, {"gid": gid}) for gid in owners],
                "samples": sum(stats.count for stats in owners.values()),
                "timestamp_regressions": sum(
                    stats.stamp_regressions for stats in owners.values()
                ),
            }
        for rule in self.plan.get("tf", []):
            if int(rule["step"]) not in self.steps:
                continue
            key = f"{rule['parent']}->{rule['child']}"
            observed = tf_edges.get(key)
            reasons = []
            status = PASS
            if observed is None:
                status = BLOCKED
                reasons.append("transform not observed")
            else:
                if len(observed["owners"]) > int(rule.get("max_publishers", 1)):
                    status = FAIL
                    reasons.append("duplicate TF owners")
                if observed["timestamp_regressions"]:
                    status = FAIL
                    reasons.append("TF timestamp regression")
            checks.append(
                {
                    "kind": "tf",
                    "step": int(rule["step"]),
                    "name": key,
                    "status": status,
                    "reasons": reasons,
                    "observed": observed,
                }
            )
        tf_graph_publishers = {}
        for topic in ("/tf", "/tf_static"):
            tf_graph_publishers[topic] = [
                self._endpoint(endpoint)
                for endpoint in self.get_publishers_info_by_topic(topic)
            ]
        for rule in self.plan.get("tf_publishers", []):
            if int(rule["step"]) not in self.steps:
                continue
            endpoints = tf_graph_publishers.get(rule["topic"], [])
            names = {item["node_name"] for item in endpoints}
            required = set(rule.get("required_nodes", []))
            allowed = set(rule.get("allowed_nodes", []))
            missing = sorted(required - names)
            unexpected = sorted(names - allowed)
            checks.append(
                {
                    "kind": "tf_publishers",
                    "step": int(rule["step"]),
                    "name": rule["topic"],
                    "status": FAIL if unexpected else (BLOCKED if missing else PASS),
                    "missing_nodes": missing,
                    "unexpected_nodes": unexpected,
                    "publishers": endpoints,
                    "note": (
                        "Python callback metadata cannot attribute each TF sample "
                        "to a GID; exact edges and the allowed publisher set are "
                        "checked separately"
                    ),
                }
            )
        hard = [item for item in checks if item["status"] in (FAIL, BLOCKED)]
        manual = {
            str(step): self.plan.get("manual", {}).get(
                step, self.plan.get("manual", {}).get(str(step), [])
            )
            for step in sorted(self.steps)
        }
        manual = {step: items for step, items in manual.items() if items}
        return {
            "scope": (
                "read-only ROS graph observation; no publishers, goals, services "
                "or device access"
            ),
            "steps": sorted(self.steps),
            "checks": checks,
            "manual_checks": manual,
            "decision": "STOP" if hard else ("MODIFY" if manual else "GO"),
            "automatic_gate_passed": not hard,
            "topic_inventory": inventory,
            "action_inventory": actions,
            "tf_edges": tf_edges,
            "tf_graph_publishers": tf_graph_publishers,
            "safety": {"publishers_created": 0, "motion_commands_sent": 0},
        }


def _load_plan(path: Path) -> dict[str, Any]:
    plan = yaml.safe_load(path.read_text())
    if not isinstance(plan, dict) or plan.get("schema_version") != 1:
        raise ValueError("acceptance plan must be a schema_version 1 mapping")
    return plan


def _prepare_output(path: Path) -> None:
    if path.exists() and not path.is_dir():
        raise FileExistsError(f"evidence output is not a directory: {path}")
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty evidence directory: {path}")
    path.mkdir(parents=True, exist_ok=True)


def _write_output(output: Path, report: dict[str, Any]) -> None:
    (output / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    for key in ("versions", "topic_inventory", "tf_edges"):
        if key in report:
            (output / f"{key}.json").write_text(
                json.dumps(report[key], indent=2, ensure_ascii=False) + "\n"
            )
    with (output / "checks.jsonl").open("w") as stream:
        for check in report.get("checks", []):
            stream.write(json.dumps(check, ensure_ascii=False) + "\n")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Read-only RM2027 new-car hardware acceptance evidence capture"
    )
    parser.add_argument("--mode", choices=("preflight", "observe"), default="preflight")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", default="1,3,4,5,6,7,8")
    parser.add_argument("--duration", type=float, default=15.0)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    workspace = parsed.workspace.resolve()
    output = parsed.output.resolve()
    if parsed.plan is None:
        package_share = Path(get_package_share_directory("rm_system_monitor"))
        plan_path = package_share.joinpath("config", "new_car_hardware_acceptance.yaml")
    else:
        plan_path = parsed.plan
    _prepare_output(output)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.FileHandler(output / "probe.log"), logging.StreamHandler()],
    )
    plan = _load_plan(plan_path.resolve())
    versions = collect_versions(workspace)
    if parsed.mode == "preflight":
        report = collect_preflight(plan, workspace)
        report.update(
            {
                "mode": "preflight",
                "scope": (
                    "filesystem/package/version inspection only; no ROS node or "
                    "device access"
                ),
                "versions": versions,
                "safety": {"publishers_created": 0, "motion_commands_sent": 0},
            }
        )
    else:
        steps = {int(item) for item in parsed.steps.split(",") if item.strip()}
        if not steps or not steps.issubset(set(range(1, 9))):
            parser.error("--steps must contain values from 1 through 8")
        rclpy.init(args=ros_args)
        probe = AcceptanceProbe(plan, steps)
        try:
            deadline = time.monotonic() + parsed.duration
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
            report = probe.report()
            report.update(
                {
                    "mode": "observe",
                    "duration_sec": parsed.duration,
                    "versions": versions,
                }
            )
        finally:
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    _write_output(output, report)
    logging.info("decision=%s report=%s", report["decision"], output / "report.json")
    print(
        json.dumps(
            {
                "decision": report["decision"],
                "report": str(output / "report.json"),
            },
            ensure_ascii=False,
        )
    )
    if report["decision"] == "STOP":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
