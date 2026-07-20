from __future__ import annotations

import argparse
import copy
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .map_bundle import MapBundleError
from .map_quality import read_rosbag_metadata, sha256_file, unique_default_output_root


BASELINE_PARAMETERS: dict[str, Any] = {
    "octomap.resolution": 0.05,
    "octomap.occupancy_min_z": 0.10,
    "octomap.occupancy_max_z": 1.80,
    "octomap.point_cloud_min_z": -0.30,
    "octomap.point_cloud_max_z": 2.50,
    "octomap.filter_ground_plane": False,
    "octomap.ground_filter.distance": None,
    "octomap.sensor_model.max_range": 12.0,
    "octomap.sensor_model.hit": 0.70,
    "octomap.sensor_model.miss": 0.40,
    "sampler.output_rate": 5.0,
    "evidence.minimum_independent_frames": 2,
    "evidence.minimum_independent_viewpoints": 1,
    "octomap.incremental_2D_projection": False,
}


SINGLE_VARIABLE_AXES: tuple[tuple[str, tuple[Any, ...]], ...] = (
    ("octomap.occupancy_min_z", (0.05, 0.10, 0.15, 0.20)),
    ("octomap.occupancy_max_z", (1.50, 1.80, 2.00)),
    ("octomap.point_cloud_min_z", (-0.30, -0.15, 0.00)),
    ("octomap.point_cloud_max_z", (2.00, 2.50, 2.75)),
    ("octomap.sensor_model.max_range", (8.0, 10.0, 12.0)),
    ("sampler.output_rate", (2.5, 5.0, 10.0)),
    ("octomap.sensor_model.hit", (0.55, 0.60, 0.70)),
    ("octomap.sensor_model.miss", (0.35, 0.40, 0.45)),
    ("evidence.minimum_independent_frames", (2, 3, 4)),
    ("evidence.minimum_independent_viewpoints", (1, 2)),
)


def _slug_value(value: Any) -> str:
    if isinstance(value, bool):
        return "on" if value else "off"
    return str(value).replace("-", "m").replace(".", "p")


def generate_sweep_candidates() -> list[dict[str, Any]]:
    candidates = [{
        "candidate_id": "baseline",
        "changed_parameters": {},
        "parameters": copy.deepcopy(BASELINE_PARAMETERS),
    }]
    for parameter, values in SINGLE_VARIABLE_AXES:
        for value in values:
            if value == BASELINE_PARAMETERS[parameter]:
                continue
            parameters = copy.deepcopy(BASELINE_PARAMETERS)
            parameters[parameter] = value
            candidates.append({
                "candidate_id": f"onevar__{parameter.replace('.', '_')}__{_slug_value(value)}",
                "changed_parameters": {parameter: value},
                "parameters": parameters,
            })
    for distance in (0.03, 0.05):
        parameters = copy.deepcopy(BASELINE_PARAMETERS)
        parameters["octomap.filter_ground_plane"] = True
        parameters["octomap.ground_filter.distance"] = distance
        candidates.append({
            "candidate_id": f"ground_filter__distance_{_slug_value(distance)}",
            "changed_parameters": {
                "octomap.filter_ground_plane": True,
                "octomap.ground_filter.distance": distance,
            },
            "parameters": parameters,
        })
    return candidates


def generate_sweep_plan(
    bag_paths: list[str | Path],
    baseline_config: str | Path | None = None,
) -> dict[str, Any]:
    bags = [read_rosbag_metadata(path) for path in bag_paths]
    covered_bags = sum(1 for bag in bags if bag["topic_coverage_complete"])
    return {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "strategy": "baseline_then_single_variable_then_evidence_gated_combinations",
        "fixed_invariants": {
            "resolution": 0.05,
            "incremental_2D_projection": False,
            "forbid_yaml_threshold_cleanup": True,
            "forbid_component_size_deletion": True,
            "forbid_baseline_overwrite": True,
        },
        "baseline_config": (
            {
                "path": str(Path(baseline_config).resolve()),
                "sha256": sha256_file(baseline_config),
            }
            if baseline_config is not None
            else None
        ),
        "bags": bags,
        "topic_complete_independent_bags": covered_bags,
        "execution_status": (
            "requires_message_level_preflight"
            if covered_bags >= 2
            else "blocked_need_two_topic_complete_bags"
        ),
        "candidates": generate_sweep_candidates(),
        "selection_order": [
            "pass all obstacle-retention hard gates",
            "minimize known-free false occupancy and single-frame evidence",
            "minimize unsupported small-component pixels",
            "maximize known-space coverage",
            "prefer the smallest change nearest baseline",
        ],
    }


def _load_summary(path: str | Path) -> dict[str, Any]:
    source = Path(path).resolve()
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise MapBundleError(f"cannot read quality summary {source}: {exc}") from exc
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise MapBundleError(f"unsupported quality summary schema: {source}")
    data["_summary_path"] = str(source)
    return data


def _reduction(candidate: int, baseline: int) -> float | None:
    if baseline <= 0:
        return 1.0 if candidate <= 0 else None
    return 1.0 - candidate / float(baseline)


def evaluate_candidate(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    failures: list[str] = []
    baseline_labels = baseline.get("labels", {})
    labels = candidate.get("labels", {})
    if baseline_labels.get("status") != "evaluated" or labels.get("status") != "evaluated":
        failures.append("missing_known_free_and_protected_obstacle_labels")

    known_free_baseline = baseline_labels.get("known_free", {})
    known_free = labels.get("known_free", {})
    if int(known_free_baseline.get("cells", 0)) <= 0 or int(
        known_free.get("cells", 0)
    ) <= 0:
        failures.append("known_free_labels_have_no_map_cells")
    baseline_false = int(known_free_baseline.get("occupied_cells", 0))
    candidate_false = int(known_free.get("occupied_cells", 0))
    known_free_reduction = _reduction(candidate_false, baseline_false)
    if int(known_free.get("safety_critical_occupied_cells", -1)) != 0:
        failures.append("safety_critical_known_free_not_zero")
    if known_free_reduction is None or known_free_reduction < 0.50:
        failures.append("known_free_false_occupancy_reduction_below_50_percent")

    protected = labels.get("protected_obstacles", {})
    if protected.get("recall") is None or float(protected["recall"]) < 0.99:
        failures.append("protected_obstacle_recall_below_99_percent")
    critical_recall = protected.get("critical_recall")
    if int(protected.get("critical_points", 0)) <= 0:
        failures.append("missing_critical_low_obstacle_labels")
    if critical_recall is None or float(critical_recall) < 1.0:
        failures.append("critical_low_obstacle_recall_below_100_percent")
    if int(protected.get("wall_entries", 0)) <= 0:
        failures.append("missing_wall_continuity_labels")
    if float(protected.get("maximum_wall_gap_m", float("inf"))) > 0.15:
        failures.append("new_wall_gap_exceeds_0_15_m")

    landmarks = labels.get("landmarks", {})
    if int(landmarks.get("count", 0)) < 3:
        failures.append("fewer_than_three_labeled_landmarks")
    deviation = landmarks.get("maximum_deviation_m")
    if deviation is None or float(deviation) > 0.10:
        failures.append("landmark_deviation_exceeds_0_10_m")

    baseline_known = float(baseline.get("map", {}).get("known_fraction", 0.0))
    candidate_known = float(candidate.get("map", {}).get("known_fraction", 0.0))
    if candidate_known < baseline_known - 0.02:
        failures.append("known_space_coverage_drop_exceeds_2_points")

    baseline_small = int(
        baseline.get("components", {})
        .get("support", {})
        .get("0.10", {})
        .get("small_unsupported_pixels_le_25", 0)
    )
    candidate_small = int(
        candidate.get("components", {})
        .get("support", {})
        .get("0.10", {})
        .get("small_unsupported_pixels_le_25", 0)
    )
    small_reduction = _reduction(candidate_small, baseline_small)
    if small_reduction is None or small_reduction < 0.30:
        failures.append("unsupported_small_component_reduction_below_30_percent")

    baseline_single = baseline.get("temporal_evidence", {}).get(
        "single_frame_only_occupied_pixels"
    )
    candidate_single = candidate.get("temporal_evidence", {}).get(
        "single_frame_only_occupied_pixels"
    )
    single_reduction = None
    if baseline_single is None or candidate_single is None:
        failures.append("missing_per_frame_temporal_evidence")
    else:
        single_reduction = _reduction(int(candidate_single), int(baseline_single))
        if single_reduction is None or single_reduction < 0.50:
            failures.append("single_frame_only_reduction_below_50_percent")

    independent_datasets = int(
        candidate.get("replication", {}).get("independent_datasets_passed", 0)
    )
    if independent_datasets < 2:
        failures.append("candidate_not_replicated_on_two_independent_datasets")

    return {
        "summary_path": candidate.get("_summary_path"),
        "map_id": candidate.get("bundle", {}).get("map_id"),
        "revision": candidate.get("bundle", {}).get("revision"),
        "eligible_for_parameter_selection": not failures,
        "hard_gate_failures": failures,
        "metrics": {
            "known_free_false_occupancy_reduction": known_free_reduction,
            "single_frame_only_reduction": single_reduction,
            "unsupported_small_component_reduction": small_reduction,
            "known_fraction_change": candidate_known - baseline_known,
            "protected_obstacle_recall": protected.get("recall"),
            "critical_obstacle_recall": critical_recall,
            "maximum_wall_gap_m": protected.get("maximum_wall_gap_m"),
            "maximum_landmark_deviation_m": deviation,
            "independent_datasets_passed": independent_datasets,
        },
    }


def rank_candidates(
    baseline_path: str | Path, candidate_paths: list[str | Path]
) -> dict[str, Any]:
    baseline = _load_summary(baseline_path)
    results = [
        evaluate_candidate(baseline, _load_summary(path)) for path in candidate_paths
    ]
    results.sort(
        key=lambda result: (
            not result["eligible_for_parameter_selection"],
            len(result["hard_gate_failures"]),
            -(
                result["metrics"]["known_free_false_occupancy_reduction"]
                if result["metrics"]["known_free_false_occupancy_reduction"] is not None
                else -1.0
            ),
        )
    )
    return {
        "schema_version": 1,
        "baseline_summary": baseline["_summary_path"],
        "eligible_candidates": sum(
            1 for result in results if result["eligible_for_parameter_selection"]
        ),
        "results": results,
    }


def _new_output_directory(path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    root = Path("/tmp/rm27_pcd_pgm_diag").resolve()
    try:
        destination.relative_to(root)
    except ValueError as exc:
        raise MapBundleError(f"sweep output must be under {root}") from exc
    if destination == root or destination.exists():
        raise MapBundleError("sweep output must be a new run directory")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir()
    return destination


def _write_json(directory: Path, name: str, data: dict[str, Any]) -> None:
    (directory / name).write_text(
        json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create a constrained projection sweep plan or rank quality summaries."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan = subparsers.add_parser("plan", help="create a non-Cartesian replay plan")
    plan.add_argument("--bag", action="append", default=[], help="rosbag directory")
    plan.add_argument("--baseline-config", help="current mapping_octomap.yaml")
    plan.add_argument("--output", help="new diagnostic run directory")

    rank = subparsers.add_parser("rank", help="apply over-filtering hard gates")
    rank.add_argument("--baseline", required=True, help="baseline summary.json")
    rank.add_argument(
        "--candidate", action="append", required=True, help="candidate summary.json"
    )
    rank.add_argument("--output", help="new diagnostic run directory")
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    output = arguments.output or unique_default_output_root()
    try:
        destination = _new_output_directory(output)
        if arguments.command == "plan":
            result = generate_sweep_plan(arguments.bag, arguments.baseline_config)
            filename = "sweep_plan.json"
        else:
            result = rank_candidates(arguments.baseline, arguments.candidate)
            filename = "ranking.json"
        _write_json(destination, filename, result)
    except (MapBundleError, OSError, ValueError) as exc:
        print(f"projection sweep failed: {exc}", file=sys.stderr)
        return 2
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
