"""Machine-readable navigation integrity regression summaries."""

from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
from typing import Any, Iterable

MAXIMUM_METRICS = (
    "global_translation_jump_m",
    "global_yaw_jump_rad",
    "correction_translation_jump_m",
    "correction_yaw_jump_rad",
    "correction_translation_m",
    "correction_yaw_rad",
    "correction_translation_rate_mps",
    "correction_yaw_rate_rps",
    "scan_map_mean_residual_m",
    "scan_map_p95_residual_m",
    "global_pose_age_sec",
    "scan_age_sec",
    "pose_scan_dt_sec",
    "pose_odom_dt_sec",
    "tf_scan_dt_sec",
    "scan_nan_ratio",
    "scan_infinite_ratio",
    "scan_out_of_range_ratio",
)

MINIMUM_METRICS = (
    "scan_map_agreement",
    "valid_scan_points",
    "scan_points_in_map",
    "scan_rate_hz",
)


def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
    source = Path(path)
    records = []
    with source.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                record = json.loads(stripped)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSONL line {line_number}: {error}") from error
            if record.get("schema_version") != 1 or not isinstance(
                record.get("metrics"), dict
            ):
                raise ValueError(f"unsupported integrity record at line {line_number}")
            records.append(record)
    if not records:
        raise ValueError("integrity JSONL contains no records")
    return records


def summarize_records(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    materialized = list(records)
    if not materialized:
        raise ValueError("cannot summarize an empty record set")
    states = Counter(str(record.get("state", "UNKNOWN")) for record in materialized)
    reasons = Counter(
        str(reason)
        for record in materialized
        for reason in record.get("reasons", [])
    )
    metric_rows = [record["metrics"] for record in materialized]
    extrema: dict[str, Any] = {}
    for name in MAXIMUM_METRICS:
        values = _finite_metric_values(metric_rows, name)
        extrema[name] = {"max": max(values), "samples": len(values)} if values else None
    for name in MINIMUM_METRICS:
        values = _finite_metric_values(metric_rows, name)
        extrema[name] = {"min": min(values), "samples": len(values)} if values else None
    stamps = _finite_metric_values(metric_rows, "stamp_sec")
    return {
        "schema_version": 1,
        "evaluation_status": "RAW_METRICS_ONLY",
        "sample_count": len(materialized),
        "state_counts": dict(sorted(states.items())),
        "reason_counts": dict(sorted(reasons.items())),
        "time_range": {
            "start_sec": min(stamps) if stamps else None,
            "end_sec": max(stamps) if stamps else None,
            "duration_sec": max(stamps) - min(stamps) if len(stamps) >= 2 else 0.0,
        },
        "extrema": extrema,
    }


def compare_summaries(
    current: dict[str, Any], baseline: dict[str, Any]
) -> dict[str, Any]:
    deltas: dict[str, Any] = {}
    current_extrema = current.get("extrema", {})
    baseline_extrema = baseline.get("extrema", {})
    for name in sorted(set(current_extrema) | set(baseline_extrema)):
        current_entry = current_extrema.get(name)
        baseline_entry = baseline_extrema.get(name)
        if not current_entry or not baseline_entry:
            deltas[name] = None
            continue
        selector = "max" if "max" in current_entry else "min"
        current_value = current_entry.get(selector)
        baseline_value = baseline_entry.get(selector)
        if _finite_number(current_value) and _finite_number(baseline_value):
            deltas[name] = {
                "selector": selector,
                "current": current_value,
                "baseline": baseline_value,
                "delta": current_value - baseline_value,
            }
        else:
            deltas[name] = None
    return {
        "comparison_status": "DELTA_ONLY_NO_PASS_FAIL",
        "sample_count_delta": current.get("sample_count", 0)
        - baseline.get("sample_count", 0),
        "metric_deltas": deltas,
    }


def load_scenario_definition(path: str | Path, scenario_id: str) -> dict[str, Any]:
    import yaml

    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("unsupported regression scenario schema")
    scenarios = document.get("scenarios")
    if not isinstance(scenarios, dict) or scenario_id not in scenarios:
        raise ValueError(f"unknown regression scenario: {scenario_id}")
    scenario = scenarios[scenario_id]
    if not isinstance(scenario, dict):
        raise ValueError(f"scenario {scenario_id} must be a mapping")
    return {"id": scenario_id, **scenario}


def write_json(path: str | Path, document: dict[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _finite_metric_values(rows: Iterable[dict[str, Any]], name: str) -> list[float]:
    return [
        float(row[name])
        for row in rows
        if name in row and _finite_number(row[name])
    ]


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
