from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from .map_bundle import MapBundleError, validate_map_bundle
from .map_quality import load_occupancy_image


def expected_grid_from_manifest(manifest_path: str | Path) -> dict[str, Any]:
    bundle = validate_map_bundle(manifest_path)
    occupancy = load_occupancy_image(bundle["occupancy"]["yaml_path"])
    pgm_values = np.full(occupancy.pixels.shape, -1, dtype=np.int8)
    pgm_values[occupancy.free] = 0
    pgm_values[occupancy.occupied] = 100
    grid_values = pgm_values[::-1, :].reshape(-1)
    return {
        "map_id": bundle["map_id"],
        "revision": bundle["revision"],
        "width": occupancy.width,
        "height": occupancy.height,
        "resolution": occupancy.resolution,
        "origin": occupancy.origin,
        "data": grid_values,
        "histogram": {
            "occupied_100": int((grid_values == 100).sum()),
            "free_0": int((grid_values == 0).sum()),
            "unknown_minus_1": int((grid_values == -1).sum()),
        },
    }


def compare_occupancy_message(message: Any, expected: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    info = message.info
    position = info.origin.position
    orientation = info.origin.orientation
    origin_x, origin_y, origin_yaw = expected["origin"]
    expected_z = math.sin(origin_yaw / 2.0)
    expected_w = math.cos(origin_yaw / 2.0)

    scalar_checks = {
        "frame_id": (message.header.frame_id, "map", 0.0),
        "width": (int(info.width), int(expected["width"]), 0.0),
        "height": (int(info.height), int(expected["height"]), 0.0),
        "resolution": (float(info.resolution), float(expected["resolution"]), 1.0e-8),
        "origin_x": (float(position.x), float(origin_x), 1.0e-8),
        "origin_y": (float(position.y), float(origin_y), 1.0e-8),
        "origin_z": (float(position.z), 0.0, 1.0e-8),
        "orientation_x": (float(orientation.x), 0.0, 1.0e-8),
        "orientation_y": (float(orientation.y), 0.0, 1.0e-8),
        "orientation_z": (float(orientation.z), expected_z, 1.0e-8),
        "orientation_w": (float(orientation.w), expected_w, 1.0e-8),
    }
    for name, (actual, wanted, tolerance) in scalar_checks.items():
        if isinstance(actual, str):
            matches = actual == wanted
        else:
            matches = math.isclose(actual, wanted, rel_tol=0.0, abs_tol=tolerance)
        if not matches:
            failures.append(f"{name}: expected {wanted!r}, got {actual!r}")

    actual_data = np.asarray(message.data, dtype=np.int16)
    expected_data = np.asarray(expected["data"], dtype=np.int16)
    if actual_data.shape != expected_data.shape:
        failures.append(
            f"data_length: expected {expected_data.size}, got {actual_data.size}"
        )
        mismatches = None
    else:
        mismatches = int((actual_data != expected_data).sum())
        if mismatches:
            failures.append(f"occupancy_data: {mismatches} cells differ")

    return {
        "matches": not failures,
        "failures": failures,
        "data_mismatches": mismatches,
        "actual_histogram": {
            "occupied_100": int((actual_data == 100).sum()),
            "free_0": int((actual_data == 0).sum()),
            "unknown_minus_1": int((actual_data == -1).sum()),
            "other": int(
                (~np.isin(actual_data, np.asarray([-1, 0, 100]))).sum()
            ),
        },
        "expected_histogram": expected["histogram"],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a live transient-local nav_msgs/OccupancyGrid with every "
            "cell decoded offline from an immutable map bundle."
        )
    )
    parser.add_argument("manifest", help="map bundle manifest loaded by map_server")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--map-server-node", default="/map_server")
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    if arguments.timeout <= 0.0:
        print("map_server verification failed: timeout must be positive", file=sys.stderr)
        return 2
    try:
        expected = expected_grid_from_manifest(arguments.manifest)
    except (MapBundleError, OSError, ValueError) as exc:
        print(f"map_server verification failed: {exc}", file=sys.stderr)
        return 2

    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=None)
    node = Node("rm_map_tools_map_server_verifier")
    lifecycle_label = None
    received: list[Any] = []
    try:
        state_client = node.create_client(
            GetState, f"{arguments.map_server_node.rstrip('/')}/get_state"
        )
        if state_client.wait_for_service(timeout_sec=arguments.timeout):
            future = state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=arguments.timeout
            )
            if future.done() and future.result() is not None:
                lifecycle_label = future.result().current_state.label

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        subscription = node.create_subscription(
            OccupancyGrid, arguments.topic, received.append, qos
        )
        deadline = time.monotonic() + arguments.timeout
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(subscription)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not received:
        print("map_server verification failed: no OccupancyGrid received", file=sys.stderr)
        return 2
    result = compare_occupancy_message(received[-1], expected)
    result.update({
        "map_id": expected["map_id"],
        "revision": expected["revision"],
        "topic": arguments.topic,
        "lifecycle_state": lifecycle_label,
    })
    if lifecycle_label != "active":
        result["matches"] = False
        result["failures"].append(
            f"lifecycle_state: expected 'active', got {lifecycle_label!r}"
        )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["matches"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
