from __future__ import annotations

from dataclasses import dataclass
import hashlib
from enum import IntEnum
import math
import os
from pathlib import Path
import re
import stat
import struct
from typing import Any, Iterable, Sequence

import yaml


REGION_SCHEMA = "rm_semantic_regions/v1"
ANNOTATED_PATH_SCHEMA = "rm_annotated_path/v1"
PATH_REVISION_SCHEMA = b"rm_path_revision/v1\0"
MAX_REGION_FILE_BYTES = 1024 * 1024
MAX_REGIONS = 256
MAX_POLYGON_VERTICES = 256
MAX_PATH_POSES = 20_000
MAX_ANNOTATION_OPERATIONS = 2_000_000
MAX_OUTPUT_SEGMENTS = 10_000
GEOMETRY_EPSILON = 1.0e-9
_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,63}$")
_MAP_BINDING_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class RegionContractError(ValueError):
    """The semantic-region or annotated-path contract is invalid."""


class RegionType(IntEnum):
    SLOW_ZONE = 1
    NO_SPIN = 2
    FORBIDDEN = 3
    DOG_HOLE_APPROACH = 4
    COMMITTED_CORRIDOR = 5
    TEMPORARY_STRUCTURE = 6


class AdmissionPolicy(IntEnum):
    NONE = 0
    DYNAMIC_CLEARANCE = 1


class TraversalPolicy(IntEnum):
    PREEMPTIBLE = 0
    COMMITTED = 1


_REGION_TYPES = {
    "slow_zone": RegionType.SLOW_ZONE,
    "no_spin": RegionType.NO_SPIN,
    "forbidden": RegionType.FORBIDDEN,
    "dog_hole_approach": RegionType.DOG_HOLE_APPROACH,
    "committed_corridor": RegionType.COMMITTED_CORRIDOR,
    "temporary_structure": RegionType.TEMPORARY_STRUCTURE,
}
_ADMISSION_POLICIES = {
    "none": AdmissionPolicy.NONE,
    "dynamic_clearance": AdmissionPolicy.DYNAMIC_CLEARANCE,
}
_TRAVERSAL_POLICIES = {
    "preemptible": TraversalPolicy.PREEMPTIBLE,
    "committed": TraversalPolicy.COMMITTED,
}


@dataclass(frozen=True)
class MapBinding:
    frame_id: str
    map_id: str
    map_revision: str
    manifest_sha256: str


@dataclass(frozen=True)
class SemanticRegion:
    region_id: str
    region_type: RegionType
    polygon: tuple[tuple[float, float], ...]
    max_linear_speed: float | None
    required_heading: float | None
    heading_tolerance: float | None
    no_spin: bool
    admission_policy: AdmissionPolicy
    traversal_policy: TraversalPolicy


@dataclass(frozen=True)
class RegionSet:
    region_set_id: str
    revision: str
    contract_sha256: str
    map_binding: MapBinding
    regions: tuple[SemanticRegion, ...]


@dataclass(frozen=True)
class PathPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class PathIntentSegment:
    start_distance: float
    end_distance: float
    region_ids: tuple[str, ...]
    region_types: tuple[RegionType, ...]
    blocked: bool
    max_linear_speed: float | None
    required_heading: float | None
    heading_tolerance: float | None
    no_spin: bool
    admission_policy: AdmissionPolicy
    traversal_policy: TraversalPolicy


@dataclass(frozen=True)
class AnnotatedPathResult:
    path_length: float
    segments: tuple[PathIntentSegment, ...]


class _ContractLoader(yaml.SafeLoader):
    def compose_node(self, parent: Any, index: Any) -> Any:
        if self.check_event(yaml.AliasEvent):
            raise RegionContractError("YAML aliases are forbidden")
        return super().compose_node(parent, index)


def _construct_unique_mapping(
    loader: _ContractLoader,
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


_ContractLoader.add_constructor(
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
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise RegionContractError(f"{location} must be a string-keyed mapping")
    missing = required - value.keys()
    unknown = value.keys() - allowed
    if missing:
        raise RegionContractError(f"{location} missing keys: {sorted(missing)}")
    if unknown:
        raise RegionContractError(f"{location} has unknown keys: {sorted(unknown)}")
    return value


def _identifier(value: Any, location: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise RegionContractError(f"{location} must match {_ID_PATTERN.pattern}")
    return value


def _map_binding_identifier(value: Any, location: str) -> str:
    """Accept identifiers emitted by rm_map_tools without weakening region IDs."""
    if not isinstance(value, str) or not _MAP_BINDING_ID_PATTERN.fullmatch(value):
        raise RegionContractError(
            f"{location} must match {_MAP_BINDING_ID_PATTERN.pattern}"
        )
    return value


def _finite_number(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RegionContractError(f"{location} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise RegionContractError(f"{location} must be a finite number")
    return result


def _enum_value(value: Any, location: str, choices: dict[str, Any]) -> Any:
    if not isinstance(value, str) or value not in choices:
        raise RegionContractError(f"{location} must be one of {sorted(choices)}")
    return choices[value]


def _cross(
    first: tuple[float, float],
    second: tuple[float, float],
) -> float:
    return first[0] * second[1] - first[1] * second[0]


def _subtract(
    first: tuple[float, float],
    second: tuple[float, float],
) -> tuple[float, float]:
    return first[0] - second[0], first[1] - second[1]


def _point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> bool:
    segment = _subtract(end, start)
    offset = _subtract(point, start)
    if abs(_cross(segment, offset)) > GEOMETRY_EPSILON:
        return False
    dot = offset[0] * segment[0] + offset[1] * segment[1]
    squared_length = segment[0] * segment[0] + segment[1] * segment[1]
    return -GEOMETRY_EPSILON <= dot <= squared_length + GEOMETRY_EPSILON


def _segments_intersect(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> bool:
    first = _subtract(first_end, first_start)
    second = _subtract(second_end, second_start)
    offset = _subtract(second_start, first_start)
    denominator = _cross(first, second)
    if abs(denominator) <= GEOMETRY_EPSILON:
        if abs(_cross(offset, first)) > GEOMETRY_EPSILON:
            return False
        return any(
            _point_on_segment(point, first_start, first_end)
            for point in (second_start, second_end)
        ) or any(
            _point_on_segment(point, second_start, second_end)
            for point in (first_start, first_end)
        )
    first_parameter = _cross(offset, second) / denominator
    second_parameter = _cross(offset, first) / denominator
    return (
        -GEOMETRY_EPSILON <= first_parameter <= 1.0 + GEOMETRY_EPSILON
        and -GEOMETRY_EPSILON <= second_parameter <= 1.0 + GEOMETRY_EPSILON
    )


def _polygon(value: Any, location: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or not 3 <= len(value) <= MAX_POLYGON_VERTICES:
        raise RegionContractError(
            f"{location} must contain 3..{MAX_POLYGON_VERTICES} vertices"
        )
    vertices: list[tuple[float, float]] = []
    for index, raw_vertex in enumerate(value):
        if not isinstance(raw_vertex, list) or len(raw_vertex) != 2:
            raise RegionContractError(f"{location}[{index}] must be [x, y]")
        vertex = (
            _finite_number(raw_vertex[0], f"{location}[{index}][0]"),
            _finite_number(raw_vertex[1], f"{location}[{index}][1]"),
        )
        if max(abs(vertex[0]), abs(vertex[1])) > 10_000.0:
            raise RegionContractError(f"{location}[{index}] exceeds coordinate bound")
        if vertices and vertex == vertices[-1]:
            raise RegionContractError(f"{location} has consecutive duplicate vertices")
        vertices.append(vertex)
    if vertices[0] == vertices[-1]:
        raise RegionContractError(f"{location} must not repeat its first vertex")
    if len(set(vertices)) != len(vertices):
        raise RegionContractError(f"{location} has duplicate vertices")

    edge_count = len(vertices)
    for first_index in range(edge_count):
        first_end_index = (first_index + 1) % edge_count
        for second_index in range(first_index + 1, edge_count):
            second_end_index = (second_index + 1) % edge_count
            if (
                first_index == second_index
                or first_end_index == second_index
                or second_end_index == first_index
            ):
                continue
            if _segments_intersect(
                vertices[first_index],
                vertices[first_end_index],
                vertices[second_index],
                vertices[second_end_index],
            ):
                raise RegionContractError(f"{location} is self-intersecting")
    twice_area = sum(
        vertices[index][0] * vertices[(index + 1) % len(vertices)][1]
        - vertices[(index + 1) % len(vertices)][0] * vertices[index][1]
        for index in range(len(vertices))
    )
    if abs(twice_area) <= GEOMETRY_EPSILON:
        raise RegionContractError(f"{location} has zero area")
    return tuple(vertices)


def _parse_region(value: Any, index: int) -> SemanticRegion:
    location = f"regions[{index}]"
    allowed = {
        "id",
        "type",
        "polygon",
        "max_linear_speed",
        "required_heading",
        "heading_tolerance",
        "no_spin",
        "admission_policy",
        "traversal_policy",
    }
    data = _mapping(
        value,
        location,
        required={"id", "type", "polygon"},
        allowed=allowed,
    )
    region_id = _identifier(data["id"], f"{location}.id")
    region_type = _enum_value(data["type"], f"{location}.type", _REGION_TYPES)
    polygon = _polygon(data["polygon"], f"{location}.polygon")

    max_linear_speed = None
    if "max_linear_speed" in data:
        max_linear_speed = _finite_number(
            data["max_linear_speed"], f"{location}.max_linear_speed"
        )
        if not 0.0 < max_linear_speed <= 20.0:
            raise RegionContractError(
                f"{location}.max_linear_speed must be in (0, 20]"
            )
    if region_type == RegionType.SLOW_ZONE and max_linear_speed is None:
        raise RegionContractError(f"{location} slow_zone requires max_linear_speed")
    if region_type == RegionType.FORBIDDEN and max_linear_speed is not None:
        raise RegionContractError(f"{location} forbidden must not set max_linear_speed")

    required_heading = None
    heading_tolerance = None
    if "required_heading" in data:
        required_heading = _finite_number(
            data["required_heading"], f"{location}.required_heading"
        )
        if not -math.pi <= required_heading <= math.pi:
            raise RegionContractError(
                f"{location}.required_heading must be in [-pi, pi]"
            )
        if "heading_tolerance" not in data:
            raise RegionContractError(
                f"{location}.heading_tolerance is required with required_heading"
            )
        heading_tolerance = _finite_number(
            data["heading_tolerance"], f"{location}.heading_tolerance"
        )
        if not 0.0 < heading_tolerance <= math.pi:
            raise RegionContractError(
                f"{location}.heading_tolerance must be in (0, pi]"
            )
    elif "heading_tolerance" in data:
        raise RegionContractError(
            f"{location}.heading_tolerance requires required_heading"
        )

    raw_no_spin = data.get("no_spin", False)
    if not isinstance(raw_no_spin, bool):
        raise RegionContractError(f"{location}.no_spin must be boolean")
    no_spin = raw_no_spin or region_type == RegionType.NO_SPIN

    default_admission = (
        AdmissionPolicy.DYNAMIC_CLEARANCE
        if region_type in {RegionType.DOG_HOLE_APPROACH, RegionType.COMMITTED_CORRIDOR}
        else AdmissionPolicy.NONE
    )
    admission_policy = _enum_value(
        data.get(
            "admission_policy",
            "dynamic_clearance"
            if default_admission == AdmissionPolicy.DYNAMIC_CLEARANCE
            else "none",
        ),
        f"{location}.admission_policy",
        _ADMISSION_POLICIES,
    )
    if admission_policy != default_admission:
        raise RegionContractError(
            f"{location}.admission_policy conflicts with region type"
        )

    default_traversal = (
        TraversalPolicy.COMMITTED
        if region_type == RegionType.COMMITTED_CORRIDOR
        else TraversalPolicy.PREEMPTIBLE
    )
    traversal_policy = _enum_value(
        data.get(
            "traversal_policy",
            "committed"
            if default_traversal == TraversalPolicy.COMMITTED
            else "preemptible",
        ),
        f"{location}.traversal_policy",
        _TRAVERSAL_POLICIES,
    )
    if traversal_policy != default_traversal:
        raise RegionContractError(
            f"{location}.traversal_policy conflicts with region type"
        )
    return SemanticRegion(
        region_id=region_id,
        region_type=region_type,
        polygon=polygon,
        max_linear_speed=max_linear_speed,
        required_heading=required_heading,
        heading_tolerance=heading_tolerance,
        no_spin=no_spin,
        admission_policy=admission_policy,
        traversal_policy=traversal_policy,
    )


def _update_digest_string(digest: Any, value: str) -> None:
    encoded = value.encode("utf-8")
    digest.update(struct.pack("<I", len(encoded)))
    digest.update(encoded)


def _region_contract_sha256(
    region_set_id: str,
    revision: str,
    map_binding: MapBinding,
    regions: Sequence[SemanticRegion],
) -> str:
    digest = hashlib.sha256()
    digest.update(b"rm_semantic_regions/v1\0")
    for value in (
        region_set_id,
        revision,
        map_binding.frame_id,
        map_binding.map_id,
        map_binding.map_revision,
        map_binding.manifest_sha256,
    ):
        _update_digest_string(digest, value)
    ordered = sorted(regions, key=lambda region: region.region_id)
    digest.update(struct.pack("<I", len(ordered)))
    for region in ordered:
        _update_digest_string(digest, region.region_id)
        digest.update(struct.pack("<B", int(region.region_type)))
        digest.update(struct.pack("<I", len(region.polygon)))
        for point in region.polygon:
            digest.update(struct.pack("<2d", *point))
        digest.update(struct.pack("<?", region.max_linear_speed is not None))
        if region.max_linear_speed is not None:
            digest.update(struct.pack("<d", region.max_linear_speed))
        digest.update(struct.pack("<?", region.required_heading is not None))
        if region.required_heading is not None:
            if region.heading_tolerance is None:
                raise RegionContractError("required heading lacks heading tolerance")
            digest.update(
                struct.pack(
                    "<2d", region.required_heading, region.heading_tolerance
                )
            )
        digest.update(
            struct.pack(
                "<?BB",
                region.no_spin,
                int(region.admission_policy),
                int(region.traversal_policy),
            )
        )
    return digest.hexdigest()


def parse_region_set(data: Any) -> RegionSet:
    root = _mapping(
        data,
        "root",
        required={"schema", "region_set_id", "revision", "map_binding", "regions"},
        allowed={"schema", "region_set_id", "revision", "map_binding", "regions"},
    )
    if root["schema"] != REGION_SCHEMA:
        raise RegionContractError(f"schema must be {REGION_SCHEMA!r}")
    region_set_id = _identifier(root["region_set_id"], "region_set_id")
    revision = _identifier(root["revision"], "revision")

    binding_data = _mapping(
        root["map_binding"],
        "map_binding",
        required={"frame_id", "map_id", "map_revision", "manifest_sha256"},
        allowed={"frame_id", "map_id", "map_revision", "manifest_sha256"},
    )
    frame_id = binding_data["frame_id"]
    if frame_id != "map":
        raise RegionContractError("map_binding.frame_id must be 'map'")
    manifest_sha256 = binding_data["manifest_sha256"]
    if not isinstance(manifest_sha256, str) or not _SHA256_PATTERN.fullmatch(
        manifest_sha256
    ):
        raise RegionContractError(
            "map_binding.manifest_sha256 must be 64 lowercase hex characters"
        )
    map_binding = MapBinding(
        frame_id=frame_id,
        map_id=_map_binding_identifier(
            binding_data["map_id"], "map_binding.map_id"
        ),
        map_revision=_map_binding_identifier(
            binding_data["map_revision"], "map_binding.map_revision"
        ),
        manifest_sha256=manifest_sha256,
    )

    raw_regions = root["regions"]
    if not isinstance(raw_regions, list) or len(raw_regions) > MAX_REGIONS:
        raise RegionContractError(f"regions must be a list with at most {MAX_REGIONS} items")
    regions = tuple(_parse_region(value, index) for index, value in enumerate(raw_regions))
    ids = [region.region_id for region in regions]
    if len(ids) != len(set(ids)):
        raise RegionContractError("region ids must be unique")
    return RegionSet(
        region_set_id=region_set_id,
        revision=revision,
        contract_sha256=_region_contract_sha256(
            region_set_id, revision, map_binding, regions
        ),
        map_binding=map_binding,
        regions=regions,
    )


def load_region_set(path_value: str | Path) -> RegionSet:
    path = Path(path_value)
    if not hasattr(os, "O_NOFOLLOW"):
        raise RegionContractError("platform lacks no-follow region file support")
    descriptor = -1
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    except OSError as error:
        raise RegionContractError(
            f"region file must be a readable regular non-symlink file: {error}"
        ) from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise RegionContractError("region file must be a regular non-symlink file")
        if not 0 < before.st_size <= MAX_REGION_FILE_BYTES:
            raise RegionContractError(
                f"region file size must be in 1..{MAX_REGION_FILE_BYTES} bytes"
            )
        chunks: list[bytes] = []
        remaining = before.st_size + 1
        while remaining > 0:
            chunk = os.read(descriptor, min(64 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        raw = b"".join(chunks)
        after = os.fstat(descriptor)
        if (
            len(raw) != before.st_size
            or after.st_size != before.st_size
            or after.st_mtime_ns != before.st_mtime_ns
            or after.st_ctime_ns != before.st_ctime_ns
        ):
            raise RegionContractError("region file changed while being read")
    except OSError as error:
        raise RegionContractError(f"cannot read region file: {error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    try:
        document = yaml.load(raw.decode("utf-8"), Loader=_ContractLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as error:
        raise RegionContractError(f"cannot parse region file: {error}") from error
    return parse_region_set(document)


def validate_map_binding(
    region_set: RegionSet,
    *,
    expected_map_id: str,
    expected_map_revision: str,
    expected_manifest_sha256: str,
) -> None:
    actual = region_set.map_binding
    expected = (expected_map_id, expected_map_revision, expected_manifest_sha256)
    if any(not value for value in expected):
        raise RegionContractError("all expected map binding values are required")
    if actual.map_id != expected_map_id:
        raise RegionContractError("region map_id does not match expected map")
    if actual.map_revision != expected_map_revision:
        raise RegionContractError("region map_revision does not match expected map")
    if actual.manifest_sha256 != expected_manifest_sha256:
        raise RegionContractError("region manifest_sha256 does not match expected map")


def compute_path_revision(
    frame_id: str,
    stamp_ns: int,
    poses: Sequence[PathPose],
) -> str:
    if not isinstance(frame_id, str) or not frame_id or len(frame_id.encode("utf-8")) > 256:
        raise RegionContractError("path frame_id must be non-empty and at most 256 bytes")
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
        raise RegionContractError("path stamp_ns must be a positive integer")
    if not 0 < len(poses) <= MAX_PATH_POSES:
        raise RegionContractError(f"path must contain 1..{MAX_PATH_POSES} poses")
    frame_bytes = frame_id.encode("utf-8")
    digest = hashlib.sha256()
    digest.update(PATH_REVISION_SCHEMA)
    digest.update(struct.pack("<I", len(frame_bytes)))
    digest.update(frame_bytes)
    digest.update(struct.pack("<Q", stamp_ns))
    digest.update(struct.pack("<I", len(poses)))
    for index, pose in enumerate(poses):
        values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
        if any(not math.isfinite(value) for value in values):
            raise RegionContractError(f"path pose {index} contains non-finite values")
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if quaternion_norm <= GEOMETRY_EPSILON:
            raise RegionContractError(f"path pose {index} has an invalid quaternion")
        digest.update(struct.pack("<7d", *values))
    return digest.hexdigest()


def _point_in_polygon(
    point: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(point, start, end):
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing_x = (
                start[0]
                + (point[1] - start[1]) * (end[0] - start[0])
                / (end[1] - start[1])
            )
            if crossing_x > point[0]:
                inside = not inside
    return inside


def _intersection_parameters(
    start: tuple[float, float],
    end: tuple[float, float],
    edge_start: tuple[float, float],
    edge_end: tuple[float, float],
) -> list[float]:
    path_vector = _subtract(end, start)
    edge_vector = _subtract(edge_end, edge_start)
    offset = _subtract(edge_start, start)
    denominator = _cross(path_vector, edge_vector)
    if abs(denominator) > GEOMETRY_EPSILON:
        path_parameter = _cross(offset, edge_vector) / denominator
        edge_parameter = _cross(offset, path_vector) / denominator
        if (
            -GEOMETRY_EPSILON <= path_parameter <= 1.0 + GEOMETRY_EPSILON
            and -GEOMETRY_EPSILON <= edge_parameter <= 1.0 + GEOMETRY_EPSILON
        ):
            return [min(1.0, max(0.0, path_parameter))]
        return []
    if abs(_cross(offset, path_vector)) > GEOMETRY_EPSILON:
        return []
    squared_length = path_vector[0] ** 2 + path_vector[1] ** 2
    if squared_length <= GEOMETRY_EPSILON:
        return []
    parameters = []
    for point in (edge_start, edge_end):
        difference = _subtract(point, start)
        parameter = (
            difference[0] * path_vector[0] + difference[1] * path_vector[1]
        ) / squared_length
        if -GEOMETRY_EPSILON <= parameter <= 1.0 + GEOMETRY_EPSILON:
            parameters.append(min(1.0, max(0.0, parameter)))
    return parameters


def _unique_sorted(values: Iterable[float]) -> list[float]:
    result: list[float] = []
    for value in sorted(values):
        if not result or abs(value - result[-1]) > GEOMETRY_EPSILON:
            result.append(value)
    return result


def _region_spans(
    points: Sequence[tuple[float, float]],
    region: SemanticRegion,
) -> tuple[list[tuple[float, float]], float]:
    spans: list[tuple[float, float]] = []
    distance = 0.0
    for index in range(len(points) - 1):
        start = points[index]
        end = points[index + 1]
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length <= GEOMETRY_EPSILON:
            continue
        parameters = [0.0, 1.0]
        for edge_index, edge_start in enumerate(region.polygon):
            edge_end = region.polygon[(edge_index + 1) % len(region.polygon)]
            parameters.extend(
                _intersection_parameters(start, end, edge_start, edge_end)
            )
        parameters = _unique_sorted(parameters)
        for first, second in zip(parameters, parameters[1:]):
            if second - first <= GEOMETRY_EPSILON:
                continue
            midpoint = 0.5 * (first + second)
            point = (
                start[0] + midpoint * (end[0] - start[0]),
                start[1] + midpoint * (end[1] - start[1]),
            )
            if _point_in_polygon(point, region.polygon):
                span = (distance + first * length, distance + second * length)
                if spans and span[0] <= spans[-1][1] + GEOMETRY_EPSILON:
                    spans[-1] = (spans[-1][0], max(spans[-1][1], span[1]))
                else:
                    spans.append(span)
        distance += length
    return spans, distance


def _angle_difference(first: float, second: float) -> float:
    return math.remainder(first - second, 2.0 * math.pi)


def _intent_key(segment: PathIntentSegment) -> tuple[Any, ...]:
    return (
        segment.region_ids,
        segment.region_types,
        segment.blocked,
        segment.max_linear_speed,
        segment.required_heading,
        segment.heading_tolerance,
        segment.no_spin,
        segment.admission_policy,
        segment.traversal_policy,
    )


def _resolve_intent(
    start_distance: float,
    end_distance: float,
    regions: Sequence[SemanticRegion],
) -> PathIntentSegment:
    ordered = tuple(sorted(regions, key=lambda region: region.region_id))
    speed_limits = [
        region.max_linear_speed
        for region in ordered
        if region.max_linear_speed is not None
    ]
    headings = [region for region in ordered if region.required_heading is not None]
    required_heading = None
    heading_tolerance = None
    if headings:
        required_heading = headings[0].required_heading
        heading_tolerance = min(region.heading_tolerance for region in headings)
        for region in headings[1:]:
            tolerance = min(heading_tolerance, region.heading_tolerance)
            if abs(_angle_difference(required_heading, region.required_heading)) > tolerance:
                ids = ", ".join(item.region_id for item in headings)
                raise RegionContractError(
                    f"overlapping regions have incompatible headings: {ids}"
                )
    return PathIntentSegment(
        start_distance=start_distance,
        end_distance=end_distance,
        region_ids=tuple(region.region_id for region in ordered),
        region_types=tuple(region.region_type for region in ordered),
        blocked=any(region.region_type == RegionType.FORBIDDEN for region in ordered),
        max_linear_speed=min(speed_limits) if speed_limits else None,
        required_heading=required_heading,
        heading_tolerance=heading_tolerance,
        no_spin=any(region.no_spin for region in ordered),
        admission_policy=max(region.admission_policy for region in ordered),
        traversal_policy=max(region.traversal_policy for region in ordered),
    )


def annotate_path(
    poses: Sequence[PathPose],
    region_set: RegionSet,
) -> AnnotatedPathResult:
    if not poses:
        raise RegionContractError("path must contain at least one pose")
    if len(poses) > MAX_PATH_POSES:
        raise RegionContractError(f"path exceeds {MAX_PATH_POSES} poses")
    points = tuple((pose.x, pose.y) for pose in poses)
    if any(not math.isfinite(value) for point in points for value in point):
        raise RegionContractError("path contains non-finite planar coordinates")
    region_intervals: list[tuple[float, float, SemanticRegion]] = []
    edge_count = sum(len(region.polygon) for region in region_set.regions)
    operation_budget = max(0, len(points) - 1) * edge_count
    if operation_budget > MAX_ANNOTATION_OPERATIONS:
        raise RegionContractError(
            f"path-region intersection exceeds {MAX_ANNOTATION_OPERATIONS} operations"
        )
    path_length = 0.0
    for region in region_set.regions:
        spans, measured_length = _region_spans(points, region)
        path_length = measured_length
        if len(region_intervals) + len(spans) > MAX_OUTPUT_SEGMENTS:
            raise RegionContractError(
                f"annotation exceeds {MAX_OUTPUT_SEGMENTS} region intervals"
            )
        region_intervals.extend((start, end, region) for start, end in spans)
    if not region_set.regions:
        path_length = sum(
            math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
            for index in range(len(points) - 1)
        )
    boundaries = _unique_sorted(
        value
        for start, end, unused_region in region_intervals
        for value in (start, end)
    )
    if max(0, len(boundaries) - 1) > MAX_OUTPUT_SEGMENTS:
        raise RegionContractError(
            f"annotation exceeds {MAX_OUTPUT_SEGMENTS} output segments"
        )
    segments: list[PathIntentSegment] = []
    for start, end in zip(boundaries, boundaries[1:]):
        if end - start <= GEOMETRY_EPSILON:
            continue
        midpoint = 0.5 * (start + end)
        active = [
            region
            for interval_start, interval_end, region in region_intervals
            if interval_start - GEOMETRY_EPSILON <= midpoint
            and midpoint <= interval_end + GEOMETRY_EPSILON
        ]
        if not active:
            continue
        segment = _resolve_intent(start, end, active)
        if (
            segments
            and abs(segments[-1].end_distance - segment.start_distance)
            <= GEOMETRY_EPSILON
            and _intent_key(segments[-1]) == _intent_key(segment)
        ):
            previous = segments[-1]
            segments[-1] = PathIntentSegment(
                start_distance=previous.start_distance,
                end_distance=segment.end_distance,
                region_ids=previous.region_ids,
                region_types=previous.region_types,
                blocked=previous.blocked,
                max_linear_speed=previous.max_linear_speed,
                required_heading=previous.required_heading,
                heading_tolerance=previous.heading_tolerance,
                no_spin=previous.no_spin,
                admission_policy=previous.admission_policy,
                traversal_policy=previous.traversal_policy,
            )
        else:
            segments.append(segment)
    return AnnotatedPathResult(path_length=path_length, segments=tuple(segments))
