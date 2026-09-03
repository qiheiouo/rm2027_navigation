from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class StrategyError(ValueError):
    """Raised when a strategy profile or field override is unsafe to load."""


@dataclass(frozen=True)
class ResolvedStrategy:
    profile: str
    description: str
    risk: str
    strategy_id: str
    tree_path: Path
    parameters: dict[str, Any]
    map_id: str | None
    map_revision: str | None
    map_manifest_sha256: str | None
    home_pose_count: int
    patrol_waypoint_count: int


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_HASH = re.compile(r"^[0-9a-fA-F]{64}$")
_FIELD_KEYS = {
    "schema_version",
    "strategy_id",
    "profile",
    "map_binding",
    "minimum_goal_clearance_m",
    "waypoints",
    "notes",
}


def _load_yaml(path: Path, label: str) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise StrategyError(f"cannot read {label} '{path}': {exc}") from exc
    if not isinstance(value, dict):
        raise StrategyError(f"{label} root must be a mapping: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise StrategyError(f"cannot hash '{path}': {exc}") from exc
    return digest.hexdigest()


def _non_empty_string(parent: dict[str, Any], key: str, label: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise StrategyError(f"{label}.{key} must be a non-empty string")
    return value.strip()


def _contained_file(root: Path, relative_value: Any, label: str) -> Path:
    if not isinstance(relative_value, str) or not relative_value.strip():
        raise StrategyError(f"{label} must be a non-empty relative path")
    relative = Path(relative_value)
    if relative.is_absolute() or ".." in relative.parts:
        raise StrategyError(f"{label} must be relative to the package share")
    # Keep the installed path lexical.  With ``--symlink-install``, resolving a
    # valid package file follows the install-space symlink back into the source
    # tree and would incorrectly make it look as if it escaped the share.
    path = root / relative
    if not path.is_file():
        raise StrategyError(f"{label} does not exist: {path}")
    return path


def list_profiles(package_share: str | Path) -> dict[str, dict[str, Any]]:
    share = Path(package_share).resolve()
    catalog = _load_yaml(share / "config" / "strategy_profiles.yaml", "profile catalog")
    if catalog.get("schema_version") != 1:
        raise StrategyError("profile catalog schema_version must be 1")
    profiles = catalog.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise StrategyError("profile catalog profiles must be a non-empty mapping")
    return profiles


def _base_parameters(config_path: Path) -> dict[str, Any]:
    document = _load_yaml(config_path, "mission base config")
    try:
        parameters = dict(document["competition_mission_node"]["ros__parameters"])
    except (KeyError, TypeError, ValueError) as exc:
        raise StrategyError(
            "mission base config must contain competition_mission_node.ros__parameters"
        ) from exc
    if parameters.get("startup_enabled") is not False:
        raise StrategyError("strategy base config must set startup_enabled: false")
    for key in (
        "require_localization_valid",
        "require_referee_state",
        "require_game_running",
    ):
        if parameters.get(key) is not True:
            raise StrategyError(f"strategy base config must set {key}: true")
    for optional_array in ("home_pose", "patrol_waypoints"):
        if parameters.get(optional_array) == []:
            parameters.pop(optional_array)
    return parameters


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise StrategyError(f"{label} must be numeric")
    output = float(value)
    if not math.isfinite(output):
        raise StrategyError(f"{label} must be finite")
    return output


def _profile_minimum_clearance(profile: dict[str, Any], profile_name: str) -> float:
    clearance = _number(
        profile.get("minimum_goal_clearance_m", 0.35),
        f"profile {profile_name}.minimum_goal_clearance_m",
    )
    if clearance < 0.0 or clearance > 2.0:
        raise StrategyError(
            f"profile {profile_name}.minimum_goal_clearance_m must be in [0, 2]"
        )
    return clearance


def _pose(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, dict):
        raise StrategyError(f"{label} must be a mapping with x, y and yaw")
    unknown = set(value) - {"x", "y", "yaw"}
    if unknown:
        raise StrategyError(f"{label} has unknown keys: {sorted(unknown)}")
    if set(value) != {"x", "y", "yaw"}:
        raise StrategyError(f"{label} must contain exactly x, y and yaw")
    return (
        _number(value["x"], f"{label}.x"),
        _number(value["y"], f"{label}.y"),
        _number(value["yaw"], f"{label}.yaw"),
    )


def _map_identity(manifest_path: Path) -> tuple[dict[str, Any], str, str, str]:
    manifest = _load_yaml(manifest_path, "map bundle")
    map_id = _non_empty_string(manifest, "map_id", "map bundle")
    revision = _non_empty_string(manifest, "revision", "map bundle")
    return manifest, map_id, revision, _sha256(manifest_path)


def _read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise StrategyError(f"cannot read occupancy image '{path}': {exc}") from exc
    position = 0

    def token() -> bytes:
        nonlocal position
        while position < len(data):
            if data[position] == ord("#"):
                newline = data.find(b"\n", position)
                position = len(data) if newline < 0 else newline + 1
            elif chr(data[position]).isspace():
                position += 1
            else:
                break
        start = position
        while position < len(data) and not chr(data[position]).isspace():
            position += 1
        if start == position:
            raise StrategyError(f"invalid PGM header: {path}")
        return data[start:position]

    magic = token()
    try:
        width = int(token())
        height = int(token())
        maximum = int(token())
    except ValueError as exc:
        raise StrategyError(f"invalid PGM dimensions: {path}") from exc
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
        raise StrategyError(f"invalid PGM dimensions/range: {path}")
    count = width * height
    if magic == b"P2":
        try:
            pixels = [int(token()) for _ in range(count)]
        except ValueError as exc:
            raise StrategyError(f"invalid ASCII PGM payload: {path}") from exc
    elif magic == b"P5":
        if position >= len(data) or not chr(data[position]).isspace():
            raise StrategyError(f"binary PGM header has no payload separator: {path}")
        if data[position:position + 2] == b"\r\n":
            position += 2
        else:
            position += 1
        bytes_per_pixel = 1 if maximum < 256 else 2
        payload = data[position:position + count * bytes_per_pixel]
        if len(payload) != count * bytes_per_pixel:
            raise StrategyError(f"binary PGM payload is truncated: {path}")
        if bytes_per_pixel == 1:
            pixels = list(payload)
        else:
            pixels = [
                (payload[index] << 8) | payload[index + 1]
                for index in range(0, len(payload), 2)
            ]
    else:
        raise StrategyError(f"unsupported occupancy image format {magic!r}: {path}")
    if any(pixel < 0 or pixel > maximum for pixel in pixels):
        raise StrategyError(f"PGM pixel is outside declared range: {path}")
    return width, height, maximum, pixels


@dataclass(frozen=True)
class _OccupancyGrid:
    width: int
    height: int
    maximum: int
    pixels: list[int]
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: bool
    free_threshold: float

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = math.floor(local_x / self.resolution)
        map_row = math.floor(local_y / self.resolution)
        return int(column), self.height - 1 - int(map_row)

    def _is_free_cell(self, column: int, row: int) -> bool:
        if column < 0 or column >= self.width or row < 0 or row >= self.height:
            return False
        pixel = self.pixels[row * self.width + column]
        normalized = pixel / self.maximum
        occupancy = normalized if self.negate else 1.0 - normalized
        return occupancy < self.free_threshold

    def require_clear(
        self,
        pose: tuple[float, float, float],
        clearance: float,
        label: str,
    ) -> None:
        center_column, center_row = self._cell(pose[0], pose[1])
        radius = int(math.ceil(clearance / self.resolution))
        for row_offset in range(-radius, radius + 1):
            for column_offset in range(-radius, radius + 1):
                if math.hypot(column_offset, row_offset) * self.resolution > clearance:
                    continue
                if not self._is_free_cell(
                    center_column + column_offset, center_row + row_offset
                ):
                    raise StrategyError(
                        f"{label} is outside known free space or lacks "
                        f"{clearance:.3f} m map clearance"
                    )


def _occupancy_grid(manifest_path: Path, manifest: dict[str, Any]) -> _OccupancyGrid:
    try:
        occupancy = manifest["artifacts"]["occupancy"]
        yaml_relative = occupancy["yaml_path"]
    except (KeyError, TypeError) as exc:
        raise StrategyError("map bundle has no artifacts.occupancy.yaml_path") from exc
    if not isinstance(yaml_relative, str) or not yaml_relative:
        raise StrategyError("map bundle occupancy yaml_path must be a non-empty string")
    yaml_relative_path = Path(yaml_relative)
    if yaml_relative_path.is_absolute():
        raise StrategyError("map bundle occupancy yaml_path must be relative")
    map_yaml_path = (manifest_path.parent / yaml_relative_path).resolve()
    try:
        map_yaml_path.relative_to(manifest_path.parent.resolve())
    except ValueError as exc:
        raise StrategyError("map bundle occupancy yaml_path escapes its revision") from exc
    map_yaml = _load_yaml(map_yaml_path, "occupancy map YAML")
    image_value = map_yaml.get("image")
    if not isinstance(image_value, str) or not image_value:
        raise StrategyError("occupancy map YAML image must be a non-empty path")
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = (map_yaml_path.parent / image_path).resolve()
    if not image_path.is_file():
        raise StrategyError(f"occupancy image does not exist: {image_path}")
    resolution = _number(map_yaml.get("resolution"), "occupancy resolution")
    if resolution <= 0.0:
        raise StrategyError("occupancy resolution must be positive")
    origin = map_yaml.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise StrategyError("occupancy origin must contain [x, y, yaw]")
    origin_values = tuple(_number(item, "occupancy origin") for item in origin)
    free_threshold = _number(map_yaml.get("free_thresh", 0.196), "free_thresh")
    if not 0.0 <= free_threshold <= 1.0:
        raise StrategyError("free_thresh must be in [0, 1]")
    negate_value = map_yaml.get("negate", 0)
    if isinstance(negate_value, bool):
        negate = negate_value
    elif isinstance(negate_value, int) and negate_value in (0, 1):
        negate = bool(negate_value)
    else:
        raise StrategyError("occupancy negate must be 0 or 1")
    width, height, maximum, pixels = _read_pgm(image_path)
    return _OccupancyGrid(
        width=width,
        height=height,
        maximum=maximum,
        pixels=pixels,
        resolution=resolution,
        origin_x=origin_values[0],
        origin_y=origin_values[1],
        origin_yaw=origin_values[2],
        negate=negate,
        free_threshold=free_threshold,
    )


def _field_waypoints(
    field_path: Path,
    expected_profile: str,
    map_manifest_path: Path,
    profile_minimum_clearance: float,
) -> tuple[
    str,
    str,
    str,
    str,
    float,
    tuple[float, float, float] | None,
    list[tuple[float, float, float]],
]:
    field = _load_yaml(field_path, "field strategy")
    unknown = set(field) - _FIELD_KEYS
    if unknown:
        raise StrategyError(f"field strategy has unknown keys: {sorted(unknown)}")
    if field.get("schema_version") != 1:
        raise StrategyError("field strategy schema_version must be 1")
    strategy_id = _non_empty_string(field, "strategy_id", "field strategy")
    if not _IDENTIFIER.fullmatch(strategy_id):
        raise StrategyError("field strategy strategy_id contains unsupported characters")
    profile = _non_empty_string(field, "profile", "field strategy")
    if profile != expected_profile:
        raise StrategyError(
            f"field strategy profile {profile!r} does not match selected profile "
            f"{expected_profile!r}"
        )
    binding = field.get("map_binding")
    if not isinstance(binding, dict):
        raise StrategyError("field strategy map_binding must be a mapping")
    unknown_binding = set(binding) - {"map_id", "map_revision", "manifest_sha256"}
    if unknown_binding:
        raise StrategyError(f"map_binding has unknown keys: {sorted(unknown_binding)}")
    bound_map_id = _non_empty_string(binding, "map_id", "map_binding")
    bound_revision = _non_empty_string(binding, "map_revision", "map_binding")
    bound_hash = _non_empty_string(binding, "manifest_sha256", "map_binding").lower()
    if not _HASH.fullmatch(bound_hash):
        raise StrategyError("map_binding.manifest_sha256 must contain 64 hex characters")
    manifest, map_id, revision, manifest_hash = _map_identity(map_manifest_path)
    if (bound_map_id, bound_revision, bound_hash) != (map_id, revision, manifest_hash):
        raise StrategyError(
            "field strategy map binding mismatch: expected "
            f"{map_id}/{revision}/{manifest_hash}, got "
            f"{bound_map_id}/{bound_revision}/{bound_hash}"
        )
    clearance = _number(
        field.get("minimum_goal_clearance_m", 0.35),
        "minimum_goal_clearance_m",
    )
    if clearance < profile_minimum_clearance or clearance > 2.0:
        raise StrategyError(
            "minimum_goal_clearance_m must be in "
            f"[{profile_minimum_clearance}, 2] for profile {expected_profile!r}"
        )
    waypoints = field.get("waypoints")
    if not isinstance(waypoints, dict):
        raise StrategyError("field strategy waypoints must be a mapping")
    unknown_waypoints = set(waypoints) - {"home", "patrol"}
    if unknown_waypoints:
        raise StrategyError(f"waypoints has unknown keys: {sorted(unknown_waypoints)}")
    home_value = waypoints.get("home")
    home = None if home_value is None else _pose(home_value, "waypoints.home")
    patrol_value = waypoints.get("patrol", [])
    if not isinstance(patrol_value, list):
        raise StrategyError("waypoints.patrol must be a sequence")
    patrol = [
        _pose(value, f"waypoints.patrol[{index}]")
        for index, value in enumerate(patrol_value)
    ]
    grid = _occupancy_grid(map_manifest_path, manifest)
    if home is not None:
        grid.require_clear(home, clearance, "waypoints.home")
    for index, pose in enumerate(patrol):
        grid.require_clear(pose, clearance, f"waypoints.patrol[{index}]")
    return strategy_id, map_id, revision, manifest_hash, clearance, home, patrol


def resolve_strategy(
    package_share: str | Path,
    profile_name: str,
    field_strategy_file: str | Path | None = None,
    map_bundle_manifest: str | Path | None = None,
) -> ResolvedStrategy:
    share = Path(package_share).resolve()
    profile_name = profile_name.strip()
    profiles = list_profiles(share)
    if profile_name not in profiles:
        raise StrategyError(
            f"unknown strategy profile {profile_name!r}; choices: {', '.join(sorted(profiles))}"
        )
    profile = profiles[profile_name]
    if not isinstance(profile, dict):
        raise StrategyError(f"profile {profile_name!r} must be a mapping")
    description = _non_empty_string(profile, "description", f"profile {profile_name}")
    risk = _non_empty_string(profile, "risk", f"profile {profile_name}")
    tree_path = _contained_file(share, profile.get("tree"), f"profile {profile_name}.tree")
    config_path = _contained_file(
        share, profile.get("base_config"), f"profile {profile_name}.base_config"
    )
    parameters = _base_parameters(config_path)
    field_required = profile.get("field_strategy_required")
    if not isinstance(field_required, bool):
        raise StrategyError(
            f"profile {profile_name}.field_strategy_required must be boolean"
        )
    minimum_patrol = profile.get("minimum_patrol_waypoints", 0)
    if (
        isinstance(minimum_patrol, bool)
        or not isinstance(minimum_patrol, int)
        or minimum_patrol < 0
    ):
        raise StrategyError(
            f"profile {profile_name}.minimum_patrol_waypoints must be a non-negative integer"
        )
    require_home = profile.get("require_home", False)
    if not isinstance(require_home, bool):
        raise StrategyError(f"profile {profile_name}.require_home must be boolean")
    minimum_clearance = _profile_minimum_clearance(profile, profile_name)

    field_value = "" if field_strategy_file is None else str(field_strategy_file).strip()
    map_value = "" if map_bundle_manifest is None else str(map_bundle_manifest).strip()
    strategy_id = profile_name
    map_id = None
    revision = None
    manifest_hash = None
    home = None
    patrol: list[tuple[float, float, float]] = []
    if field_required and not field_value:
        raise StrategyError(f"profile {profile_name!r} requires a field strategy file")
    if not field_required and field_value:
        raise StrategyError(f"profile {profile_name!r} does not accept a field strategy file")
    if field_value:
        if not map_value:
            raise StrategyError("map bundle manifest is required with a field strategy file")
        field_path = Path(field_value).resolve()
        map_path = Path(map_value).resolve()
        if not field_path.is_file():
            raise StrategyError(f"field strategy file does not exist: {field_path}")
        if not map_path.is_file():
            raise StrategyError(f"map bundle manifest does not exist: {map_path}")
        (
            strategy_id,
            map_id,
            revision,
            manifest_hash,
            _clearance,
            home,
            patrol,
        ) = _field_waypoints(
            field_path,
            profile_name,
            map_path,
            minimum_clearance,
        )
        if home is not None:
            parameters["home_pose"] = list(home)
        if patrol:
            parameters["patrol_waypoints"] = [value for pose in patrol for value in pose]
    if require_home and home is None:
        raise StrategyError(f"profile {profile_name!r} requires waypoints.home")
    if len(patrol) < minimum_patrol:
        raise StrategyError(
            f"profile {profile_name!r} requires at least {minimum_patrol} patrol waypoint(s)"
        )
    return ResolvedStrategy(
        profile=profile_name,
        description=description,
        risk=risk,
        strategy_id=strategy_id,
        tree_path=tree_path,
        parameters=parameters,
        map_id=map_id,
        map_revision=revision,
        map_manifest_sha256=manifest_hash,
        home_pose_count=1 if home is not None else 0,
        patrol_waypoint_count=len(patrol),
    )


def make_field_strategy_template(
    package_share: str | Path,
    profile_name: str,
    map_bundle_manifest: str | Path,
) -> dict[str, Any]:
    profiles = list_profiles(package_share)
    if profile_name not in profiles:
        raise StrategyError(
            f"unknown strategy profile {profile_name!r}; choices: {', '.join(sorted(profiles))}"
        )
    profile = profiles[profile_name]
    if not isinstance(profile, dict) or profile.get("field_strategy_required") is not True:
        raise StrategyError(f"profile {profile_name!r} does not use a field strategy file")
    minimum_clearance = _profile_minimum_clearance(profile, profile_name)
    manifest_path = Path(map_bundle_manifest).resolve()
    if not manifest_path.is_file():
        raise StrategyError(f"map bundle manifest does not exist: {manifest_path}")
    _manifest, map_id, revision, manifest_hash = _map_identity(manifest_path)
    safe_revision = re.sub(r"[^A-Za-z0-9_.-]+", "-", revision).strip("-") or "revision"
    return {
        "schema_version": 1,
        "strategy_id": f"{profile_name}_{safe_revision}",
        "profile": profile_name,
        "map_binding": {
            "map_id": map_id,
            "map_revision": revision,
            "manifest_sha256": manifest_hash,
        },
        "minimum_goal_clearance_m": minimum_clearance,
        "waypoints": {
            "home": {"x": None, "y": None, "yaw": None},
            "patrol": [{"x": None, "y": None, "yaw": None}],
        },
        "notes": "Replace placeholder map-frame poses, then run validate_match_strategy.",
    }
