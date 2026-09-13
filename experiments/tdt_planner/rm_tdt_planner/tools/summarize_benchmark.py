#!/usr/bin/env python3
"""Validate paired requests and summarize offline P2 evidence without rerunning it."""
import argparse
import collections
import csv
import json
import math
import statistics


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def summarize(rows):
    requests = collections.defaultdict(list)
    groups = collections.defaultdict(list)
    for row in rows:
        requests[(row["scenario"], row["trial"])].append(row)
        groups[(row["scenario"], row["planner"])].append(row)
    if not rows:
        raise ValueError("empty benchmark")
    required = {"open", "wall_detour", "passage_080m", "passage_140m",
                "blocker_present", "blocker_removed", "unknown_barrier"}
    scenarios = {r["scenario"] for r in rows}
    if not required <= scenarios or scenarios - required - {"field_map"}:
        raise ValueError("missing or unknown scenario")
    trial_ids = {int(r["trial"]) for r in rows}
    if trial_ids != set(range(len(trial_ids))) or any(
            {int(r["trial"]) for r in rows if r["scenario"] == scenario} != trial_ids
            for scenario in scenarios):
        raise ValueError("incomplete or noncontiguous trials")
    for key, values in requests.items():
        if {r["planner"] for r in values} != {"navfn", "smac2d", "tdt_astar", "tdt_qp"} or len(values) != 4:
            raise ValueError(f"incomplete or duplicate comparison: {key}")
        if len({tuple(r[k] for k in ("seed", "start_x", "start_y", "goal_x", "goal_y")) for r in values}) != 1:
            raise ValueError(f"unpaired goals: {key}")
    for (scenario, trial), values in requests.items():
        if scenario == "blocker_present":
            removed = requests.get(("blocker_removed", trial))
            if not removed or any(values[0][k] != removed[0][k] for k in
                                  ("seed", "start_x", "start_y", "goal_x", "goal_y")):
                raise ValueError(f"unpaired obstacle removal: {trial}")
    result = []
    for (scenario, planner), values in sorted(groups.items()):
        accepted = [r for r in values if r["accepted"] == "1"]
        times = [float(r["prepare_plus_plan_ms"]) for r in values]
        if not all(math.isfinite(t) and t >= 0 for t in times):
            raise ValueError("invalid timing")
        result.append({
            "scenario": scenario, "planner": planner, "requests": len(values),
            "returned": sum(int(r["returned"]) for r in values), "accepted": len(accepted),
            "optimized": sum(int(r["optimized"]) for r in values),
            "returned_geometry_rejected": sum(r["returned"] == "1" and r["common_geometry_ok"] == "0" for r in values),
            "returned_physical_collision": sum(r["returned"] == "1" and float(r["physical_clearance_m"]) < -1e-9 for r in values),
            "over_250ms": sum(t > 250 for t in times),
            "p50_ms": statistics.median(times), "p95_ms": percentile(times, .95),
            "p99_ms": percentile(times, .99), "max_ms": max(times),
            "mean_accepted_length_m": statistics.mean(float(r["length_m"]) for r in accepted) if accepted else None,
            "mean_accepted_turning_rad": statistics.mean(float(r["turning_rad"]) for r in accepted) if accepted else None,
            "min_accepted_physical_clearance_m": min(float(r["physical_clearance_m"]) for r in accepted) if accepted else None,
            "reasons": dict(collections.Counter(r["reason"] for r in values)),
        })
    paired = collections.defaultdict(list)
    for (scenario, _), values in requests.items():
        by_name = {r["planner"]: r for r in values}
        for baseline in ("navfn", "smac2d", "tdt_astar"):
            a, b = by_name[baseline], by_name["tdt_qp"]
            if a["accepted"] == b["accepted"] == "1":
                paired[(scenario, baseline)].append((a, b))
    comparisons = []
    for (scenario, baseline), pairs in sorted(paired.items()):
        comparisons.append({
            "scenario": scenario, "baseline": baseline, "candidate": "tdt_qp",
            "both_accepted": len(pairs),
            "mean_length_delta_m": statistics.mean(float(b["length_m"]) - float(a["length_m"]) for a, b in pairs),
            "mean_turning_delta_rad": statistics.mean(float(b["turning_rad"]) - float(a["turning_rad"]) for a, b in pairs),
            "mean_time_delta_ms": statistics.mean(float(b["prepare_plus_plan_ms"]) - float(a["prepare_plus_plan_ms"]) for a, b in pairs),
        })
    negative = {"blocker_present", "passage_080m", "unknown_barrier"}
    checks = {
        "no_returned_disk_clearance_violations": all(g["returned_physical_collision"] == 0 for g in result),
        "infeasible_cases_return_no_path": all(g["returned"] == 0 for g in result if g["scenario"] in negative),
        "tdt_feasible_cases_all_accepted": all(g["accepted"] == g["requests"] for g in result
                                              if g["scenario"] not in negative and g["planner"].startswith("tdt_")),
        "no_modeled_latency_over_250ms": all(g["over_250ms"] == 0 for g in result),
    }
    return {"paired_requests": len(requests), "planner_calls": len(rows), "groups": result,
            "paired_deltas_candidate_minus_baseline": comparisons, "offline_checks": checks}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(args.csv_file) as stream:
        result = summarize(list(csv.DictReader(stream)))
    with open(args.output, "x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    if not all(result["offline_checks"].values()):
        raise SystemExit("offline checks failed; see saved report")


if __name__ == "__main__":
    main()
