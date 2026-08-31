"""Strict map-bound route contract for a dog-hole traversal."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import stat
from typing import Any

import yaml

from rm_dog_hole_entry_gate.core import point_in_polygon
from rm_path_annotations.core import (
    MapBinding,
    RegionContractError,
    parse_region_set,
)


ROUTE_SCHEMA = "rm_dog_hole_route/v1"
MAX_ROUTE_FILE_BYTES = 1024 * 1024


@dataclass(frozen=True)
class RoutePose:
    """A planar pose in the route contract's map frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DogHoleRoute:
    """A goal-triggered stop, transition and through-route contract."""

    route_id: str
    revision: str
    contract_sha256: str
    map_binding: MapBinding
    goal_trigger_polygon: tuple[tuple[float, float], ...]
    stop_pose: RoutePose
    exit_pose: RoutePose

    def stages_goal(self, frame_id: str, x: float, y: float) -> bool:
        """Return whether a public goal must use the staged route."""
        if frame_id != self.map_binding.frame_id:
            return False
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        return point_in_polygon((x, y), self.goal_trigger_polygon)


class _RouteLoader(yaml.SafeLoader):
    def compose_node(self, parent: Any, index: Any) -> Any:
        if self.check_event(yaml.AliasEvent):
            raise RegionContractError("YAML aliases are forbidden")
        return super().compose_node(parent, index)


def _construct_unique_mapping(
    loader: _RouteLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as error:
            raise RegionContractError("YAML mapping keys must be scalar") from error
        if duplicate:
            raise RegionContractError(f"duplicate YAML key: {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_RouteLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _mapping(
    value: Any,
    location: str,
    *,
    required: set[str],
    allowed: set[str],
) -> dict[str, Any]:
    if not isinstance(value, dict) or any(not isinstance(k, str) for k in value):
        raise RegionContractError(f"{location} must be a string-keyed mapping")
    missing = required - value.keys()
    unknown = value.keys() - allowed
    if missing:
        raise RegionContractError(f"{location} missing keys: {sorted(missing)}")
    if unknown:
        raise RegionContractError(f"{location} has unknown keys: {sorted(unknown)}")
    return value


def _string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 64:
        raise RegionContractError(
            f"{location} must be a non-empty string no longer than 64 characters"
        )
    return value.strip()


def _number(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RegionContractError(f"{location} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise RegionContractError(f"{location} must be a finite number")
    return result


def _pose(value: Any, location: str) -> RoutePose:
    data = _mapping(
        value,
        location,
        required={"x", "y", "yaw"},
        allowed={"x", "y", "yaw"},
    )
    pose = RoutePose(
        x=_number(data["x"], f"{location}.x"),
        y=_number(data["y"], f"{location}.y"),
        yaw=_number(data["yaw"], f"{location}.yaw"),
    )
    if max(abs(pose.x), abs(pose.y)) > 10_000.0:
        raise RegionContractError(f"{location} exceeds coordinate bound")
    if not -math.pi <= pose.yaw <= math.pi:
        raise RegionContractError(f"{location}.yaw must be in [-pi, pi]")
    return pose


def parse_route(data: Any, *, source_sha256: str = "0" * 64) -> DogHoleRoute:
    """Parse and validate a dog-hole route document."""
    root = _mapping(
        data,
        "root",
        required={
            "schema",
            "route_id",
            "revision",
            "map_binding",
            "goal_trigger_polygon",
            "stop_pose",
            "exit_pose",
        },
        allowed={
            "schema",
            "route_id",
            "revision",
            "map_binding",
            "goal_trigger_polygon",
            "stop_pose",
            "exit_pose",
        },
    )
    if root["schema"] != ROUTE_SCHEMA:
        raise RegionContractError(f"root.schema must be {ROUTE_SCHEMA!r}")
    route_id = _string(root["route_id"], "root.route_id")
    revision = _string(root["revision"], "root.revision")
    binding_data = _mapping(
        root["map_binding"],
        "root.map_binding",
        required={"frame_id", "map_id", "map_revision", "manifest_sha256"},
        allowed={"frame_id", "map_id", "map_revision", "manifest_sha256"},
    )

    # Reuse the semantic-region contract validator for identifiers, SHA256 and
    # polygon geometry so the two dog-hole sidecars have identical rules.
    region_set = parse_region_set({
        "schema": "rm_semantic_regions/v1",
        "region_set_id": route_id,
        "revision": revision,
        "map_binding": dict(binding_data),
        "regions": [{
            "id": "goal_trigger",
            "type": "temporary_structure",
            "polygon": root["goal_trigger_polygon"],
        }],
    })
    binding = region_set.map_binding
    if binding.frame_id != "map":
        raise RegionContractError("dog-hole routes require map_binding.frame_id=map")

    stop_pose = _pose(root["stop_pose"], "root.stop_pose")
    exit_pose = _pose(root["exit_pose"], "root.exit_pose")
    separation = math.hypot(exit_pose.x - stop_pose.x, exit_pose.y - stop_pose.y)
    if separation < 0.20:
        raise RegionContractError("stop_pose and exit_pose must be at least 0.20 m apart")
    trigger = region_set.regions[0].polygon
    if point_in_polygon((stop_pose.x, stop_pose.y), trigger):
        raise RegionContractError("stop_pose must be outside goal_trigger_polygon")
    if point_in_polygon((exit_pose.x, exit_pose.y), trigger):
        raise RegionContractError("exit_pose must be outside goal_trigger_polygon")

    return DogHoleRoute(
        route_id=route_id,
        revision=revision,
        contract_sha256=source_sha256,
        map_binding=binding,
        goal_trigger_polygon=trigger,
        stop_pose=stop_pose,
        exit_pose=exit_pose,
    )


def load_route(path_value: str | Path) -> DogHoleRoute:
    """Load a bounded, regular, non-symlink route contract."""
    path = Path(path_value)
    try:
        metadata = path.lstat()
    except OSError as error:
        raise RegionContractError(f"cannot stat route file {path}: {error}") from error
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise RegionContractError("route file must be a regular non-symlink file")
    if metadata.st_size > MAX_ROUTE_FILE_BYTES:
        raise RegionContractError("route file exceeds the 1 MiB safety bound")
    try:
        payload = path.read_bytes()
        document = yaml.load(payload, Loader=_RouteLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RegionContractError(f"cannot load route file {path}: {error}") from error
    return parse_route(document, source_sha256=hashlib.sha256(payload).hexdigest())


def validate_route_map_binding(
    route: DogHoleRoute,
    *,
    expected_map_id: str,
    expected_map_revision: str,
    expected_manifest_sha256: str,
) -> None:
    """Reject a route sidecar bound to any other immutable map bundle."""
    actual = route.map_binding
    expected = (
        expected_map_id,
        expected_map_revision,
        expected_manifest_sha256,
    )
    observed = (actual.map_id, actual.map_revision, actual.manifest_sha256)
    if observed != expected:
        raise RegionContractError(
            "dog-hole route map binding mismatch: "
            f"expected {expected!r}, got {observed!r}"
        )
