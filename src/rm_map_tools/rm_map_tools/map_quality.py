from __future__ import annotations

import csv
import hashlib
import json
import math
import struct
import zlib
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from .map_bundle import MapBundleError, validate_map_bundle


DEFAULT_Z_LAYERS = (
    ("below_0_00", None, 0.0),
    ("z_0_00_to_0_10", 0.0, 0.10),
    ("z_0_10_to_0_20", 0.10, 0.20),
    ("z_0_20_to_0_50", 0.20, 0.50),
    ("z_0_50_to_1_00", 0.50, 1.00),
    ("z_1_00_to_1_80", 1.00, 1.80),
    ("z_1_80_to_2_50", 1.80, 2.50),
    ("above_2_50", 2.50, None),
)

DEFAULT_SUPPORT_RADII_M = (0.05, 0.10, 0.25)

REQUIRED_MAPPING_BAG_TOPICS = (
    "/livox/left/pointcloud",
    "/livox/left/pointcloud_filtered",
    "/mapping/sensor_cloud",
    "/lio/cloud_registered_transformed",
    "/odometry/lio",
    "/tf",
    "/tf_static",
    "/mapping/projected_map",
    "/mapping/octomap_full",
    "/mapping/recording",
)


@dataclass(frozen=True)
class OccupancyImage:
    pixels: np.ndarray
    occupied: np.ndarray
    free: np.ndarray
    unknown: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    yaml_path: Path
    image_path: Path

    @property
    def height(self) -> int:
        return int(self.pixels.shape[0])

    @property
    def width(self) -> int:
        return int(self.pixels.shape[1])


@dataclass(frozen=True)
class Component:
    component_id: int
    size: int
    bbox_x: int
    bbox_y: int
    bbox_w: int
    bbox_h: int
    centroid_x: float
    centroid_y: float
    perimeter_4: int

    def as_dict(self) -> dict[str, int | float]:
        return {
            "id": self.component_id,
            "size": self.size,
            "bbox_x": self.bbox_x,
            "bbox_y": self.bbox_y,
            "bbox_w": self.bbox_w,
            "bbox_h": self.bbox_h,
            "centroid_x": self.centroid_x,
            "centroid_y": self.centroid_y,
            "perimeter_4": self.perimeter_4,
            "perimeter_area_ratio": self.perimeter_4 / float(self.size),
        }


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_or_none(value: float) -> float | None:
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _next_pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    size = len(data)
    while offset < size:
        value = data[offset]
        if value in b" \t\r\n":
            offset += 1
            continue
        if value == ord("#"):
            newline = data.find(b"\n", offset)
            if newline < 0:
                raise MapBundleError("PGM comment reaches end of file")
            offset = newline + 1
            continue
        break
    start = offset
    while offset < size and data[offset] not in b" \t\r\n#":
        offset += 1
    if start == offset:
        raise MapBundleError("PGM header is truncated")
    return data[start:offset], offset


def read_pgm(path: str | Path) -> tuple[np.ndarray, int, str]:
    source = Path(path)
    data = source.read_bytes()
    magic, offset = _next_pgm_token(data, 0)
    width_token, offset = _next_pgm_token(data, offset)
    height_token, offset = _next_pgm_token(data, offset)
    max_token, offset = _next_pgm_token(data, offset)
    try:
        width = int(width_token)
        height = int(height_token)
        max_value = int(max_token)
    except ValueError as exc:
        raise MapBundleError("PGM dimensions or max value are invalid") from exc
    if width <= 0 or height <= 0:
        raise MapBundleError("PGM dimensions must be positive")
    if max_value != 255:
        raise MapBundleError("map quality analysis requires 8-bit PGM max value 255")

    if magic == b"P5":
        if offset >= len(data) or data[offset] not in b" \t\r\n":
            raise MapBundleError("binary PGM is missing raster separator")
        if data[offset:offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        payload = data[offset:]
        expected = width * height
        if len(payload) != expected:
            raise MapBundleError(
                f"binary PGM payload size {len(payload)} does not match {expected}"
            )
        pixels = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()
        return pixels, max_value, "P5"

    if magic == b"P2":
        values: list[int] = []
        while True:
            try:
                token, offset = _next_pgm_token(data, offset)
            except MapBundleError:
                break
            try:
                values.append(int(token))
            except ValueError as exc:
                raise MapBundleError("ASCII PGM contains a non-integer pixel") from exc
        expected = width * height
        if len(values) != expected:
            raise MapBundleError(
                f"ASCII PGM pixel count {len(values)} does not match {expected}"
            )
        pixels = np.asarray(values, dtype=np.uint8).reshape((height, width))
        return pixels, max_value, "P2"

    raise MapBundleError("map quality analysis supports only P2 or P5 PGM")


def load_occupancy_image(yaml_path: str | Path) -> OccupancyImage:
    metadata_path = Path(yaml_path).resolve()
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapBundleError(f"cannot read occupancy YAML: {exc}") from exc
    if not isinstance(metadata, dict):
        raise MapBundleError("occupancy YAML root must be a mapping")
    if metadata.get("mode", "trinary") != "trinary":
        raise MapBundleError("map quality analysis currently requires trinary mode")
    image_name = metadata.get("image")
    if not isinstance(image_name, str) or not image_name:
        raise MapBundleError("occupancy YAML image must be a non-empty string")
    image_path = (metadata_path.parent / image_name).resolve()
    pixels, max_value, _ = read_pgm(image_path)
    try:
        resolution = float(metadata["resolution"])
        origin_values = [float(value) for value in metadata["origin"]]
        negate = int(metadata.get("negate", 0))
        occupied_threshold = float(metadata["occupied_thresh"])
        free_threshold = float(metadata["free_thresh"])
    except (KeyError, TypeError, ValueError) as exc:
        raise MapBundleError("occupancy YAML numeric fields are invalid") from exc
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise MapBundleError("occupancy resolution must be positive and finite")
    if len(origin_values) != 3 or not all(math.isfinite(value) for value in origin_values):
        raise MapBundleError("occupancy origin must contain finite x, y and yaw")
    if negate not in (0, 1):
        raise MapBundleError("occupancy negate must be 0 or 1")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise MapBundleError("occupancy thresholds must satisfy 0 <= free < occupied <= 1")

    normalized = pixels.astype(np.float64) / float(max_value)
    probability = normalized if negate else 1.0 - normalized
    occupied = probability >= occupied_threshold
    free = probability <= free_threshold
    unknown = ~(occupied | free)
    return OccupancyImage(
        pixels=pixels,
        occupied=occupied,
        free=free,
        unknown=unknown,
        resolution=resolution,
        origin=(origin_values[0], origin_values[1], origin_values[2]),
        yaml_path=metadata_path,
        image_path=image_path,
    )


def read_ascii_xyz_pcd(path: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    source = Path(path).resolve()
    try:
        lines = source.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeError) as exc:
        raise MapBundleError(f"cannot read ASCII PCD: {exc}") from exc
    header: dict[str, list[str]] = {}
    data_line = -1
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == "DATA":
            data_line = index
            break
    if data_line < 0:
        raise MapBundleError("PCD is missing DATA header")
    mode = " ".join(header.get("DATA", [])).lower()
    if mode != "ascii":
        raise MapBundleError(
            f"map quality analysis requires ASCII PCD; detected DATA {mode or '<empty>'}"
        )
    fields = [
        value.lower() for value in (header.get("FIELDS") or header.get("FIELD") or [])
    ]
    if not fields or not {"x", "y", "z"}.issubset(fields):
        raise MapBundleError("PCD FIELDS must include x, y and z")
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if len(counts) != len(fields) or any(count != 1 for count in counts):
        raise MapBundleError("map quality analysis requires scalar PCD fields")
    try:
        declared_points = int((header.get("POINTS") or ["0"])[0])
    except ValueError as exc:
        raise MapBundleError("PCD POINTS is invalid") from exc
    payload = "\n".join(lines[data_line + 1:]).strip()
    if not payload and declared_points > 0:
        raise MapBundleError("PCD data is empty")
    values = np.fromstring(payload, sep=" ", dtype=np.float64)
    if values.size != declared_points * len(fields):
        raise MapBundleError(
            "PCD ASCII value count does not match POINTS multiplied by field count"
        )
    matrix = values.reshape((declared_points, len(fields)))
    xyz = matrix[:, [fields.index("x"), fields.index("y"), fields.index("z")]]
    if xyz.size == 0 or not np.isfinite(xyz).all():
        raise MapBundleError("PCD XYZ data must be non-empty and finite")
    return xyz.astype(np.float64, copy=False), {
        "path": str(source),
        "points": declared_points,
        "fields": fields,
        "data_mode": mode,
    }


def world_to_pgm(
    occupancy: OccupancyImage,
    world_x: np.ndarray | Sequence[float] | float,
    world_y: np.ndarray | Sequence[float] | float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x = np.asarray(world_x, dtype=np.float64)
    y = np.asarray(world_y, dtype=np.float64)
    origin_x, origin_y, origin_yaw = occupancy.origin
    dx = x - origin_x
    dy = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    columns = np.floor(local_x / occupancy.resolution).astype(np.int64)
    grid_y = np.floor(local_y / occupancy.resolution).astype(np.int64)
    rows = occupancy.height - 1 - grid_y
    inside = (
        (columns >= 0)
        & (columns < occupancy.width)
        & (rows >= 0)
        & (rows < occupancy.height)
    )
    return rows, columns, inside


def pgm_cell_centers_world(occupancy: OccupancyImage) -> tuple[np.ndarray, np.ndarray]:
    rows, columns = np.indices((occupancy.height, occupancy.width), dtype=np.float64)
    grid_y = occupancy.height - 1 - rows
    local_x = (columns + 0.5) * occupancy.resolution
    local_y = (grid_y + 0.5) * occupancy.resolution
    origin_x, origin_y, yaw = occupancy.origin
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    world_x = origin_x + cosine * local_x - sine * local_y
    world_y = origin_y + sine * local_x + cosine * local_y
    return world_x, world_y


def project_points(
    points: np.ndarray,
    occupancy: OccupancyImage,
    minimum_z: float | None = None,
    maximum_z: float | None = None,
) -> tuple[np.ndarray, int, int]:
    selected = np.ones(points.shape[0], dtype=bool)
    if minimum_z is not None:
        selected &= points[:, 2] >= minimum_z
    if maximum_z is not None:
        selected &= points[:, 2] < maximum_z
    layer = points[selected]
    counts = np.zeros((occupancy.height, occupancy.width), dtype=np.uint32)
    if layer.size == 0:
        return counts, 0, 0
    rows, columns, inside = world_to_pgm(occupancy, layer[:, 0], layer[:, 1])
    np.add.at(counts, (rows[inside], columns[inside]), 1)
    return counts, int(layer.shape[0]), int(inside.sum())


def connected_components_8(mask: np.ndarray) -> tuple[np.ndarray, list[Component]]:
    if mask.ndim != 2 or mask.dtype != np.bool_:
        mask = np.asarray(mask, dtype=bool)
    height, width = mask.shape
    labels = np.zeros((height, width), dtype=np.int32)
    components: list[Component] = []
    next_id = 0
    for start_row in range(height):
        for start_column in range(width):
            if not mask[start_row, start_column] or labels[start_row, start_column] != 0:
                continue
            next_id += 1
            labels[start_row, start_column] = next_id
            queue: deque[tuple[int, int]] = deque([(start_row, start_column)])
            size = 0
            sum_x = 0.0
            sum_y = 0.0
            minimum_x = maximum_x = start_column
            minimum_y = maximum_y = start_row
            perimeter = 0
            while queue:
                row, column = queue.popleft()
                size += 1
                sum_x += column
                sum_y += row
                minimum_x = min(minimum_x, column)
                maximum_x = max(maximum_x, column)
                minimum_y = min(minimum_y, row)
                maximum_y = max(maximum_y, row)
                for delta_y, delta_x in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    neighbor_y = row + delta_y
                    neighbor_x = column + delta_x
                    if (
                        neighbor_y < 0
                        or neighbor_y >= height
                        or neighbor_x < 0
                        or neighbor_x >= width
                        or not mask[neighbor_y, neighbor_x]
                    ):
                        perimeter += 1
                for delta_y in (-1, 0, 1):
                    for delta_x in (-1, 0, 1):
                        if delta_x == 0 and delta_y == 0:
                            continue
                        neighbor_y = row + delta_y
                        neighbor_x = column + delta_x
                        if not (0 <= neighbor_y < height and 0 <= neighbor_x < width):
                            continue
                        if (
                            mask[neighbor_y, neighbor_x]
                            and labels[neighbor_y, neighbor_x] == 0
                        ):
                            labels[neighbor_y, neighbor_x] = next_id
                            queue.append((neighbor_y, neighbor_x))
            components.append(
                Component(
                    component_id=next_id,
                    size=size,
                    bbox_x=minimum_x,
                    bbox_y=minimum_y,
                    bbox_w=maximum_x - minimum_x + 1,
                    bbox_h=maximum_y - minimum_y + 1,
                    centroid_x=sum_x / size,
                    centroid_y=sum_y / size,
                    perimeter_4=perimeter,
                )
            )
    return labels, components


def _ellipse_offsets(radius_cells: int) -> list[tuple[int, int]]:
    if radius_cells < 0:
        raise ValueError("radius_cells must be non-negative")
    if radius_cells == 0:
        return [(0, 0)]
    offsets: list[tuple[int, int]] = []
    radius_squared = float(radius_cells * radius_cells)
    for delta_y in range(-radius_cells, radius_cells + 1):
        horizontal = int(
            round(radius_cells * math.sqrt(max(0.0, 1.0 - delta_y * delta_y / radius_squared)))
        )
        for delta_x in range(-horizontal, horizontal + 1):
            offsets.append((delta_y, delta_x))
    return offsets


def dilate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    source = np.asarray(mask, dtype=bool)
    result = np.zeros_like(source)
    height, width = source.shape
    for delta_y, delta_x in _ellipse_offsets(radius_cells):
        source_y_start = max(0, -delta_y)
        source_y_end = min(height, height - delta_y)
        source_x_start = max(0, -delta_x)
        source_x_end = min(width, width - delta_x)
        if source_y_start >= source_y_end or source_x_start >= source_x_end:
            continue
        target_y_start = source_y_start + delta_y
        target_y_end = source_y_end + delta_y
        target_x_start = source_x_start + delta_x
        target_x_end = source_x_end + delta_x
        result[target_y_start:target_y_end, target_x_start:target_x_end] |= source[
            source_y_start:source_y_end,
            source_x_start:source_x_end,
        ]
    return result


def _distance_transform_1d(values: np.ndarray) -> np.ndarray:
    count = int(values.shape[0])
    sites = np.zeros(count, dtype=np.int64)
    boundaries = np.zeros(count + 1, dtype=np.float64)
    output = np.empty(count, dtype=np.float64)
    active = 0
    sites[0] = 0
    boundaries[0] = -np.inf
    boundaries[1] = np.inf
    for query in range(1, count):
        while True:
            site = int(sites[active])
            intersection = (
                (values[query] + query * query)
                - (values[site] + site * site)
            ) / float(2 * query - 2 * site)
            if intersection > boundaries[active]:
                break
            active -= 1
        active += 1
        sites[active] = query
        boundaries[active] = intersection
        boundaries[active + 1] = np.inf
    active = 0
    for query in range(count):
        while boundaries[active + 1] < query:
            active += 1
        site = int(sites[active])
        output[query] = (query - site) * (query - site) + values[site]
    return output


def euclidean_distance_to_mask(mask: np.ndarray, resolution: float) -> np.ndarray:
    features = np.asarray(mask, dtype=bool)
    if not features.any():
        return np.full(features.shape, np.inf, dtype=np.float64)
    large = float(
        features.shape[0] * features.shape[0]
        + features.shape[1] * features.shape[1]
        + 1
    )
    initial = np.where(features, 0.0, large)
    vertical = np.empty_like(initial)
    for column in range(initial.shape[1]):
        vertical[:, column] = _distance_transform_1d(initial[:, column])
    squared = np.empty_like(vertical)
    for row in range(vertical.shape[0]):
        squared[row, :] = _distance_transform_1d(vertical[row, :])
    return np.sqrt(squared) * resolution


def _region_summary(
    mask: np.ndarray,
    labels: np.ndarray,
    components: Sequence[Component],
    region_cells: int | None = None,
) -> dict[str, Any]:
    selected_ids = np.unique(labels[mask])
    selected_ids = selected_ids[selected_ids > 0]
    component_by_id = {component.component_id: component for component in components}
    return {
        "pixels": int(mask.sum()),
        "density": (
            float(mask.sum()) / float(region_cells)
            if region_cells is not None and region_cells > 0
            else float(mask.mean()) if mask.size else 0.0
        ),
        "component_ids_touching_region": int(selected_ids.size),
        "small_component_pixels_le_25": int(
            sum(
                component_by_id[int(component_id)].size
                for component_id in selected_ids
                if component_by_id[int(component_id)].size <= 25
            )
        ),
    }


def _point_in_polygon_mask(
    world_x: np.ndarray,
    world_y: np.ndarray,
    polygon: Sequence[Sequence[float]],
) -> np.ndarray:
    if len(polygon) < 3:
        raise MapBundleError("known-free polygon requires at least three points")
    vertices = [(float(point[0]), float(point[1])) for point in polygon]
    inside = np.zeros(world_x.shape, dtype=bool)
    previous_x, previous_y = vertices[-1]
    for current_x, current_y in vertices:
        crosses = (current_y > world_y) != (previous_y > world_y)
        denominator = previous_y - current_y
        if abs(denominator) < 1.0e-15:
            intersection_x = np.full(world_x.shape, np.inf)
        else:
            intersection_x = (
                (previous_x - current_x) * (world_y - current_y) / denominator
                + current_x
            )
        inside ^= crosses & (world_x < intersection_x)
        previous_x, previous_y = current_x, current_y
    return inside


def load_quality_labels(path: str | Path) -> dict[str, Any]:
    label_path = Path(path).resolve()
    try:
        data = yaml.safe_load(label_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapBundleError(f"cannot read map-quality labels: {exc}") from exc
    if not isinstance(data, dict):
        raise MapBundleError("map-quality labels root must be a mapping")
    if data.get("frame_id") != "map":
        raise MapBundleError("map-quality labels frame_id must be 'map'")
    for key in ("known_free", "protected_obstacles", "landmarks"):
        value = data.get(key, [])
        if not isinstance(value, list):
            raise MapBundleError(f"map-quality labels {key} must be a list")
    data["path"] = str(label_path)
    return data


def evaluate_labels(
    occupancy: OccupancyImage,
    labels: dict[str, Any],
    map_image_sha256: str,
) -> dict[str, Any]:
    expected_hash = labels.get("map_image_sha256")
    if expected_hash and expected_hash != map_image_sha256:
        raise MapBundleError("map-quality labels image hash does not match analyzed PGM")
    world_x, world_y = pgm_cell_centers_world(occupancy)
    known_free_entries = []
    combined_known_free = np.zeros(occupancy.occupied.shape, dtype=bool)
    safety_critical_known_free = np.zeros(occupancy.occupied.shape, dtype=bool)
    for index, region in enumerate(labels.get("known_free", [])):
        if not isinstance(region, dict):
            raise MapBundleError("known_free entries must be mappings")
        polygon = region.get("polygon")
        if not isinstance(polygon, list):
            raise MapBundleError("known_free entry must contain polygon")
        mask = _point_in_polygon_mask(world_x, world_y, polygon)
        combined_known_free |= mask
        safety_critical = bool(region.get("safety_critical", False))
        if safety_critical:
            safety_critical_known_free |= mask
        known_free_entries.append({
            "name": str(region.get("name", f"known_free_{index}")),
            "safety_critical": safety_critical,
            "cells": int(mask.sum()),
            "occupied_cells": int((mask & occupancy.occupied).sum()),
            "unknown_cells": int((mask & occupancy.unknown).sum()),
        })

    occupied_distance = euclidean_distance_to_mask(
        occupancy.occupied, occupancy.resolution
    )
    protected_entries = []
    retained_total = 0
    point_total = 0
    critical_total = 0
    critical_retained = 0
    maximum_wall_gap_m = 0.0
    wall_entries = 0
    for index, obstacle in enumerate(labels.get("protected_obstacles", [])):
        if not isinstance(obstacle, dict):
            raise MapBundleError("protected_obstacles entries must be mappings")
        points = obstacle.get("points")
        if not isinstance(points, list) or not points:
            raise MapBundleError("protected obstacle must contain non-empty points")
        tolerance = float(obstacle.get("tolerance_m", 0.10))
        critical = bool(obstacle.get("critical", False))
        kind = str(obstacle.get("kind", "obstacle"))
        sample_spacing_m = float(obstacle.get("sample_spacing_m", 0.05))
        if sample_spacing_m <= 0.0 or not math.isfinite(sample_spacing_m):
            raise MapBundleError("protected obstacle sample_spacing_m must be positive")
        coordinates = np.asarray(points, dtype=np.float64)
        if coordinates.ndim != 2 or coordinates.shape[1] != 2:
            raise MapBundleError("protected obstacle points must be Nx2")
        rows, columns, inside = world_to_pgm(
            occupancy, coordinates[:, 0], coordinates[:, 1]
        )
        retained = np.zeros(coordinates.shape[0], dtype=bool)
        retained[inside] = occupied_distance[rows[inside], columns[inside]] <= tolerance
        entry_total = int(coordinates.shape[0])
        entry_retained = int(retained.sum())
        point_total += entry_total
        retained_total += entry_retained
        if critical:
            critical_total += entry_total
            critical_retained += entry_retained
        longest_missing_run = 0
        current_missing_run = 0
        for is_retained in retained:
            if is_retained:
                current_missing_run = 0
            else:
                current_missing_run += 1
                longest_missing_run = max(longest_missing_run, current_missing_run)
        gap_m = longest_missing_run * sample_spacing_m
        if kind == "wall":
            wall_entries += 1
            maximum_wall_gap_m = max(maximum_wall_gap_m, gap_m)
        protected_entries.append({
            "name": str(obstacle.get("name", f"protected_{index}")),
            "critical": critical,
            "kind": kind,
            "tolerance_m": tolerance,
            "sample_spacing_m": sample_spacing_m,
            "points": entry_total,
            "retained": entry_retained,
            "recall": entry_retained / float(entry_total),
            "longest_missing_run": longest_missing_run,
            "estimated_gap_m": gap_m,
        })

    landmark_entries = []
    landmark_deviations: list[float] = []
    for index, landmark in enumerate(labels.get("landmarks", [])):
        if not isinstance(landmark, dict):
            raise MapBundleError("landmark entries must be mappings")
        point = np.asarray(landmark.get("point"), dtype=np.float64)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise MapBundleError("landmark point must contain finite map x and y")
        tolerance = float(landmark.get("tolerance_m", 0.10))
        rows, columns, inside = world_to_pgm(occupancy, point[0], point[1])
        distance = (
            float(occupied_distance[int(rows), int(columns)])
            if bool(inside)
            else math.inf
        )
        if math.isfinite(distance):
            landmark_deviations.append(distance)
        landmark_entries.append({
            "name": str(landmark.get("name", f"landmark_{index}")),
            "tolerance_m": tolerance,
            "inside_map": bool(inside),
            "nearest_occupied_distance_m": _finite_or_none(distance),
            "within_tolerance": distance <= tolerance,
        })

    return {
        "status": "evaluated",
        "labels_path": labels["path"],
        "known_free": {
            "regions": known_free_entries,
            "cells": int(combined_known_free.sum()),
            "occupied_cells": int((combined_known_free & occupancy.occupied).sum()),
            "unknown_cells": int((combined_known_free & occupancy.unknown).sum()),
            "safety_critical_cells": int(safety_critical_known_free.sum()),
            "safety_critical_occupied_cells": int(
                (safety_critical_known_free & occupancy.occupied).sum()
            ),
        },
        "protected_obstacles": {
            "entries": protected_entries,
            "points": point_total,
            "retained": retained_total,
            "recall": retained_total / float(point_total) if point_total else None,
            "critical_points": critical_total,
            "critical_retained": critical_retained,
            "critical_recall": (
                critical_retained / float(critical_total) if critical_total else None
            ),
            "wall_entries": wall_entries,
            "maximum_wall_gap_m": maximum_wall_gap_m,
        },
        "landmarks": {
            "entries": landmark_entries,
            "count": len(landmark_entries),
            "within_tolerance": sum(
                1 for entry in landmark_entries if entry["within_tolerance"]
            ),
            "maximum_deviation_m": (
                max(landmark_deviations)
                if len(landmark_deviations) == len(landmark_entries)
                and landmark_deviations
                else None
            ),
        },
    }


def read_rosbag_metadata(path: str | Path) -> dict[str, Any]:
    bag_path = Path(path).resolve()
    metadata_path = bag_path if bag_path.name == "metadata.yaml" else bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise MapBundleError(f"rosbag metadata not found: {metadata_path}")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapBundleError(f"cannot read rosbag metadata: {exc}") from exc
    information = metadata.get("rosbag2_bagfile_information", {})
    topics = information.get("topics_with_message_count", [])
    counts: dict[str, int] = {}
    types: dict[str, str] = {}
    for entry in topics:
        topic_metadata = entry.get("topic_metadata", {})
        name = topic_metadata.get("name")
        if isinstance(name, str):
            counts[name] = int(entry.get("message_count", 0))
            types[name] = str(topic_metadata.get("type", ""))
    missing = [topic for topic in REQUIRED_MAPPING_BAG_TOPICS if counts.get(topic, 0) <= 0]
    duration = information.get("duration", {})
    duration_ns = int(duration.get("nanoseconds", 0)) if isinstance(duration, dict) else 0
    return {
        "path": str(bag_path),
        "metadata_path": str(metadata_path),
        "metadata_sha256": sha256_file(metadata_path),
        "duration_sec": duration_ns / 1.0e9,
        "message_count": int(information.get("message_count", 0)),
        "topic_counts": counts,
        "topic_types": types,
        "required_topics": list(REQUIRED_MAPPING_BAG_TOPICS),
        "missing_or_empty_required_topics": missing,
        "topic_coverage_complete": not missing,
        "message_time_overlap_checked": False,
        "tf_timestamp_coverage_checked": False,
        "ready_for_full_mapping_replay": False,
        "per_frame_analysis_status": (
            "requires_message_level_preflight"
            if not missing
            else "blocked_missing_topics"
        ),
    }


def analyze_quality(
    manifest_path: str | Path,
    labels_path: str | Path | None = None,
    bag_path: str | Path | None = None,
) -> tuple[dict[str, Any], list[Component], dict[str, np.ndarray]]:
    bundle = validate_map_bundle(manifest_path)
    if bundle["pcd"] is None:
        raise MapBundleError("map-quality analysis requires an occupancy_with_pcd bundle")
    occupancy = load_occupancy_image(bundle["occupancy"]["yaml_path"])
    points, pcd_metadata = read_ascii_xyz_pcd(bundle["pcd"]["path"])
    labels_grid, components = connected_components_8(occupancy.occupied)

    layer_summaries: dict[str, Any] = {}
    images: dict[str, np.ndarray] = {}
    for name, minimum_z, maximum_z in DEFAULT_Z_LAYERS:
        counts, point_count, inside_count = project_points(
            points, occupancy, minimum_z, maximum_z
        )
        layer_summaries[name] = {
            "minimum_z": minimum_z,
            "maximum_z": maximum_z,
            "points": point_count,
            "points_inside_map": inside_count,
            "occupied_xy_cells": int((counts > 0).sum()),
            "maximum_points_per_cell": int(counts.max()) if counts.size else 0,
        }
        images[f"layer_{name}"] = counts

    band_counts, band_points, band_inside = project_points(points, occupancy, 0.10, 1.80)
    band_mask = band_counts > 0
    distance_to_band = euclidean_distance_to_mask(band_mask, occupancy.resolution)
    occupied_distances = distance_to_band[occupancy.occupied]
    support_summaries: dict[str, Any] = {}
    support_masks: dict[str, np.ndarray] = {}
    component_sizes = np.asarray([component.size for component in components], dtype=np.int64)
    for radius in DEFAULT_SUPPORT_RADII_M:
        radius_cells = int(round(radius / occupancy.resolution))
        support = dilate_mask(band_mask, radius_cells)
        supported_occupied = occupancy.occupied & support
        unsupported_occupied = occupancy.occupied & ~support
        supported_ids = np.unique(labels_grid[supported_occupied])
        supported_ids = supported_ids[supported_ids > 0]
        unsupported_ids = {
            component.component_id for component in components
        } - {int(value) for value in supported_ids}
        small_unsupported_ids = {
            component.component_id
            for component in components
            if component.size <= 25 and component.component_id in unsupported_ids
        }
        key = f"{radius:.2f}"
        support_summaries[key] = {
            "radius_m": radius,
            "radius_cells": radius_cells,
            "supported_occupied_pixels": int(supported_occupied.sum()),
            "unsupported_occupied_pixels": int(unsupported_occupied.sum()),
            "supported_components": int(supported_ids.size),
            "unsupported_components": len(unsupported_ids),
            "small_unsupported_components_le_25": len(small_unsupported_ids),
            "small_unsupported_pixels_le_25": int(
                sum(components[index - 1].size for index in small_unsupported_ids)
            ),
        }
        support_masks[key] = support

    sizes = component_sizes
    top_mask = np.zeros_like(occupancy.occupied)
    top_mask[:occupancy.height // 2, :] = occupancy.occupied[:occupancy.height // 2, :]
    bottom_mask = np.zeros_like(occupancy.occupied)
    bottom_mask[occupancy.height // 2:, :] = occupancy.occupied[occupancy.height // 2:, :]
    middle_x = occupancy.width // 2
    middle_y = occupancy.height // 2
    quadrants = {}
    for name, row_slice, column_slice in (
        ("top_left", slice(0, middle_y), slice(0, middle_x)),
        ("top_right", slice(0, middle_y), slice(middle_x, occupancy.width)),
        ("bottom_left", slice(middle_y, occupancy.height), slice(0, middle_x)),
        ("bottom_right", slice(middle_y, occupancy.height), slice(middle_x, occupancy.width)),
    ):
        region = occupancy.occupied[row_slice, column_slice]
        quadrants[name] = {
            "pixels": int(region.sum()),
            "density": float(region.mean()) if region.size else 0.0,
        }

    total_cells = occupancy.width * occupancy.height
    pixel_values, pixel_counts = np.unique(occupancy.pixels, return_counts=True)
    map_image_hash = sha256_file(occupancy.image_path)
    summary: dict[str, Any] = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "bundle": {
            "manifest": str(Path(manifest_path).resolve()),
            "map_id": bundle["map_id"],
            "revision": bundle["revision"],
            "deployment_status": bundle["deployment_status"],
            "manifest_sha256": sha256_file(manifest_path),
            "pcd_sha256": sha256_file(bundle["pcd"]["path"]),
            "occupancy_yaml_sha256": sha256_file(bundle["occupancy"]["yaml_path"]),
            "occupancy_image_sha256": map_image_hash,
        },
        "map": {
            "width": occupancy.width,
            "height": occupancy.height,
            "resolution": occupancy.resolution,
            "origin": list(occupancy.origin),
            "pixel_values": {
                str(int(value)): int(count)
                for value, count in zip(pixel_values, pixel_counts)
            },
            "occupied_pixels": int(occupancy.occupied.sum()),
            "free_pixels": int(occupancy.free.sum()),
            "unknown_pixels": int(occupancy.unknown.sum()),
            "known_fraction": float(
                (occupancy.occupied.sum() + occupancy.free.sum()) / total_cells
            ),
            "coordinate_contract": {
                "occupancy_grid_cell_0_0": "origin-side, lowest local-Y row",
                "pgm_first_row": "highest local-Y row",
                "world_to_grid": "inverse origin yaw, then floor(local/resolution)",
                "grid_to_pgm_row": "height - 1 - grid_y",
            },
        },
        "components": {
            "count": len(components),
            "largest": int(sizes.max()) if sizes.size else 0,
            "median": float(np.median(sizes)) if sizes.size else 0.0,
            "le_1": int((sizes <= 1).sum()),
            "le_4": int((sizes <= 4).sum()),
            "le_10": int((sizes <= 10).sum()),
            "le_25": int((sizes <= 25).sum()),
            "pixels_in_components_le_25": int(sizes[sizes <= 25].sum()),
            "support": support_summaries,
        },
        "regions": {
            "pgm_top_half": _region_summary(
                top_mask,
                labels_grid,
                components,
                (occupancy.height // 2) * occupancy.width,
            ),
            "pgm_bottom_half": _region_summary(
                bottom_mask,
                labels_grid,
                components,
                (occupancy.height - occupancy.height // 2) * occupancy.width,
            ),
            "quadrants": quadrants,
        },
        "pcd": {
            **pcd_metadata,
            "points_total": int(points.shape[0]),
            "points_inside_map_total": project_points(points, occupancy)[2],
            "points_in_0_10_to_1_80m": band_points,
            "points_inside_map_height_band": band_inside,
            "height_layers": layer_summaries,
            "occupied_distance_to_height_band_pcd_m": {
                "median": _finite_or_none(np.median(occupied_distances)),
                "p90": _finite_or_none(np.percentile(occupied_distances, 90)),
                "p99": _finite_or_none(np.percentile(occupied_distances, 99)),
                "le_0_05": int((occupied_distances <= 0.05).sum()),
                "le_0_10": int((occupied_distances <= 0.10).sum()),
                "le_0_25": int((occupied_distances <= 0.25).sum()),
                "gt_0_25": int((occupied_distances > 0.25).sum()),
                "gt_0_50": int((occupied_distances > 0.50).sum()),
            },
        },
        "labels": {"status": "not_provided"},
        "bag": {"status": "not_provided"},
        "limitations": [
            "Saved PCD support is diagnostic evidence, not ground truth.",
            "A saved PCD has no sensor origins and cannot reconstruct free/unknown rays.",
            (
                "mapping_session latest-TF fallback counters belong to the registered-"
                "cloud PCD path, not the OctoMap projected-map path."
            ),
            (
                "octomap_full stores posterior node state, not exact hit/free-ray "
                "counts; exact counts must be derived during replay."
            ),
            "Per-frame and OctoMap TF attribution requires a complete mapping rosbag.",
        ],
    }
    if labels_path is not None:
        summary["labels"] = evaluate_labels(
            occupancy, load_quality_labels(labels_path), map_image_hash
        )
    if bag_path is not None:
        summary["bag"] = read_rosbag_metadata(bag_path)

    overlay = np.repeat(occupancy.pixels[:, :, None], 3, axis=2)
    support_010 = support_masks["0.10"]
    supported = occupancy.occupied & support_010
    unsupported = occupancy.occupied & ~support_010
    overlay[supported] = np.asarray([0, 180, 0], dtype=np.uint8)
    overlay[unsupported] = np.asarray([255, 0, 0], dtype=np.uint8)
    overlay[band_mask & ~occupancy.occupied] = np.asarray([0, 100, 255], dtype=np.uint8)
    images["support_overlay"] = overlay
    images["occupancy"] = occupancy.pixels
    return summary, components, images


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind)
    checksum = zlib.crc32(payload, checksum)
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def write_png(path: str | Path, image: np.ndarray) -> None:
    destination = Path(path)
    pixels = np.asarray(image)
    if pixels.dtype != np.uint8:
        raise ValueError("PNG image must use uint8 pixels")
    if pixels.ndim == 2:
        color_type = 0
    elif pixels.ndim == 3 and pixels.shape[2] == 3:
        color_type = 2
    else:
        raise ValueError("PNG image must be HxW grayscale or HxWx3 RGB")
    height, width = pixels.shape[:2]
    scanlines = b"".join(b"\x00" + pixels[row].tobytes() for row in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(scanlines, level=9))
        + _png_chunk(b"IEND", b"")
    )
    destination.write_bytes(payload)


def density_heatmap(counts: np.ndarray) -> np.ndarray:
    values = np.log1p(np.asarray(counts, dtype=np.float64))
    maximum = float(values.max()) if values.size else 0.0
    normalized = values / maximum if maximum > 0.0 else values
    red = np.clip(2.0 * normalized - 1.0, 0.0, 1.0)
    green = np.clip(1.0 - 2.0 * np.abs(normalized - 0.5), 0.0, 1.0)
    blue = np.clip(1.0 - 2.0 * normalized, 0.0, 1.0) * np.clip(
        4.0 * normalized, 0.0, 1.0
    )
    return (np.stack((red, green, blue), axis=2) * 255.0).astype(np.uint8)


def write_quality_outputs(
    output_directory: str | Path,
    summary: dict[str, Any],
    components: Sequence[Component],
    images: dict[str, np.ndarray],
) -> Path:
    destination = Path(output_directory).resolve()
    if destination.exists():
        raise MapBundleError(f"quality output directory already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir()
    (destination / "layers").mkdir()
    (destination / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    with (destination / "components.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(Component(0, 1, 0, 0, 0, 0, 0.0, 0.0, 0).as_dict()),
        )
        writer.writeheader()
        for component in components:
            writer.writerow(component.as_dict())

    write_png(destination / "support_overlay.png", images["support_overlay"])
    write_png(destination / "occupancy.png", images["occupancy"])
    for name, image in images.items():
        if not name.startswith("layer_"):
            continue
        write_png(destination / "layers" / f"{name}.png", density_heatmap(image))

    legend = {
        "support_overlay.png": {
            "green": "occupied cell supported by 0.10 m PCD height-band evidence",
            "red": "occupied cell without 0.10 m PCD height-band evidence",
            "orange": "PCD height-band evidence in a non-occupied cell",
            "white": "free",
            "gray": "unknown",
        },
        "layers/*.png": {
            "black_blue": "zero or low point density",
            "green_yellow_red": "increasing log-scaled point density",
        },
    }
    (destination / "legend.json").write_text(
        json.dumps(legend, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    support = summary["components"]["support"]["0.10"]
    map_summary = summary["map"]
    component_summary = summary["components"]
    report = "\n".join([
        "# Map Quality Analysis",
        "",
        f"- Map: `{summary['bundle']['map_id']}` revision `{summary['bundle']['revision']}`",
        f"- Manifest: `{summary['bundle']['manifest']}`",
        (
            f"- PGM: {map_summary['width']}x{map_summary['height']} at "
            f"{map_summary['resolution']:.6f} m/cell"
        ),
        (
            "- Occupied/free/unknown: "
            f"{map_summary['occupied_pixels']}/{map_summary['free_pixels']}/"
            f"{map_summary['unknown_pixels']}"
        ),
        f"- Occupied components: {component_summary['count']}",
        (
            f"- Components <=25 cells: {component_summary['le_25']} "
            f"({component_summary['pixels_in_components_le_25']} cells)"
        ),
        f"- PCD points: {summary['pcd']['points_total']}",
        f"- PCD points in 0.10-1.80 m: {summary['pcd']['points_in_0_10_to_1_80m']}",
        f"- Occupied cells unsupported within 0.10 m: {support['unsupported_occupied_pixels']}",
        "",
        (
            "The support overlay is diagnostic evidence only. It must not be "
            "used as a standalone deletion mask."
        ),
    ])
    (destination / "report.md").write_text(report + "\n", encoding="utf-8")
    return destination


def unique_default_output_root() -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return Path("/tmp/rm27_pcd_pgm_diag") / timestamp
