#!/usr/bin/env python3
"""Leave-trial-out check of scan-center future residuals, without MPPI fitting.

Four T-DT trials build the empirical reference. Both Navfn recordings and the
separate goal trial are held out. The target collision trial is never a fit
input. Gazebo obstacle poses supply validation labels only.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze import interpolated_pose, rows_from_transport
from heldout_geometry_audit import digest, recorded_messages, training_messages
from raw_scan_support_audit import (near_full_span, one_trial, static_map,
                                    trajectory_pose)


HORIZONS = (.1, .3, .6, .9)


def residual_rows(trial, occupancy, message_source):
    messages = (training_messages(trial) if message_source == "consumed"
                else recorded_messages(trial))
    audit, rows = one_trial(trial, messages, occupancy,
                            return_source_rows=True,
                            pose_source=("gazebo" if message_source == "recorded"
                                         else "recorded_odom"))
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    if message_source == "recorded":
        observer_times = np.asarray(times)
        observer_y = np.asarray([row["robot"][1] for row in truth])
    else:
        trajectory = [json.loads(line) for line in
                      (trial / "observation/trajectory.jsonl").open()]
        trajectory_times = [row["t"] for row in trajectory]
    output = []
    source_observations = []
    for row in rows:
        if not near_full_span(row) or "scan_center_velocity" not in row:
            continue
        center = np.asarray(row["raw_bbox_mid"], dtype=float)
        velocity = np.asarray(row["scan_center_velocity"], dtype=float)
        t = row["source_t"]
        robot_y = (float(np.interp(t, observer_times, observer_y))
                   if message_source == "recorded" else
                   trajectory_pose(trajectory, trajectory_times, t)[1])
        view_y = "north" if robot_y > center[1] else "south"
        source_observations.append({
            "source_t": row["source_t"], "view_y": view_y,
            "source_y_residual_m": -row["raw_bbox_mid_error"][1]})
        for horizon in HORIZONS:
            future_t = row["source_t"] + horizon
            if future_t >= times[-1]:
                continue
            future = interpolated_pose(truth, times, future_t)
            error = np.asarray(future[:2]) - center - velocity * horizon
            output.append({"source_t": row["source_t"], "horizon_s": horizon,
                           "side": row["side"], "view_y": view_y,
                           "residual_xy_m": error.tolist()})
    return {"trial": str(trial), "message_source": message_source,
            "source_sha256": audit["source_sha256"],
            "eligible_sources": len(set(r["source_t"] for r in output)),
            "source_observations": source_observations,
            "rows": output}


def empirical_pit(training, horizon, axis, value, view_y=None):
    cdfs = []
    for trial in training:
        values = np.asarray([row["residual_xy_m"][axis]
                             for row in trial["rows"]
                             if row["horizon_s"] == horizon and
                             (view_y is None or row["view_y"] == view_y)])
        if len(values) < (10 if view_y is None else 5):
            if view_y is not None:
                continue
            raise ValueError(f"insufficient training support at {horizon}")
        cdfs.append(float((np.count_nonzero(values < value) +
                           .5 * np.count_nonzero(values == value)) / len(values)))
    if len(cdfs) < 2:
        return None
    return float(np.mean(cdfs))


def evaluate(training, trial, conditional=False):
    grouped = {}
    for row in trial["rows"]:
        h = row["horizon_s"]
        for axis, name in ((0, "x"), (1, "y")):
            view = row["view_y"] if conditional else None
            key = f"{h:.1f}s/{name}" + (f"/{view}" if conditional else "")
            pit = empirical_pit(training, h, axis,
                                row["residual_xy_m"][axis], view)
            if pit is not None:
                grouped.setdefault(key, []).append(pit)
    return {key: summarize(values) for key, values in sorted(grouped.items())}


def summarize(values):
    ordered = np.sort(values)
    n = len(ordered)
    return {"n": n,
            "outer_10pct_fraction": float(np.mean((ordered < .1) | (ordered > .9))),
            "outer_05pct_fraction": float(np.mean((ordered < .05) | (ordered > .95))),
            "mean_pit": float(np.mean(ordered)),
            "max_empirical_cdf_gap": float(max(
                np.max(np.abs(np.arange(1, n + 1) / n - ordered)),
                np.max(np.abs(np.arange(n) / n - ordered))))}


def residual_summary(trial):
    report = {}
    for horizon in HORIZONS:
        for axis, name in ((0, "x"), (1, "y")):
            values = np.asarray([row["residual_xy_m"][axis]
                                 for row in trial["rows"]
                                 if row["horizon_s"] == horizon])
            report[f"{horizon:.1f}s/{name}"] = {
                "n": len(values), "median_m": float(np.median(values)),
                "p05_m": float(np.quantile(values, .05)),
                "p95_m": float(np.quantile(values, .95))}
    return report


def source_summary(trial):
    output = {}
    for view in ("north", "south"):
        values = np.asarray([row["source_y_residual_m"]
                             for row in trial["source_observations"]
                             if row["view_y"] == view])
        output[view] = {"n": len(values),
                        "median_source_y_residual_m":
                            float(np.median(values)) if len(values) else None}
    return output


def run(args):
    model = json.loads(args.model_evidence.read_text())
    paths = [Path(row["trial"]) for row in model["historical_leave_one_trial_out"]]
    if len(paths) != 6 or any(not path.exists() for path in paths):
        raise ValueError("expected six available documented historical trials")
    if any("tdt_" not in path.name for path in paths[:4]):
        raise ValueError("first four trials must be the independent T-DT fit set")
    if any("navfn_" not in path.name for path in paths[4:]):
        raise ValueError("last two trials must be Navfn holdouts")
    occupancy, _ = static_map()
    data = [residual_rows(path, occupancy,
                          "recorded" if index < 4 else "recorded_odom")
            for index, path in enumerate(paths)]
    goal = residual_rows(args.goal_trial, occupancy, "consumed")
    training = data[:4]
    assessments = []
    for index, trial in enumerate(training):
        others = [item for j, item in enumerate(training) if j != index]
        assessments.append({"trial": trial["trial"], "split": "tdt_leave_one_out",
                            "eligible_sources": trial["eligible_sources"],
                            "message_source": trial["message_source"],
                            "source_sha256": trial["source_sha256"],
                            "source_by_view": source_summary(trial),
                            "residual": residual_summary(trial),
                            "pit": evaluate(others, trial),
                            "pit_by_view": evaluate(others, trial, True)})
    for trial in [*data[4:], goal]:
        assessment = {"trial": trial["trial"],
                      "split": "unseen_navfn_or_goal",
                      "eligible_sources": trial["eligible_sources"],
                      "message_source": trial["message_source"],
                      "source_sha256": trial["source_sha256"],
                      "source_by_view": source_summary(trial),
                      "residual": residual_summary(trial),
                      "pit": evaluate(training, trial),
                      "pit_by_view": evaluate(training, trial, True)}
        if trial in data[4:]:
            alternate = residual_rows(Path(trial["trial"]), occupancy,
                                      "recorded")
            assessment["gazebo_robot_pose_projection_check"] = {
                "eligible_sources": alternate["eligible_sources"],
                "source_by_view": source_summary(alternate),
                "pit_0_1s_y": evaluate(training, alternate)["0.1s/y"]}
        assessments.append(assessment)
    return {"schema": "rm_dynamic_prediction/historical_scan_residual_calibration/v1",
            "scope": "Causal >=3 near-full raw scan midpoint velocity. Equal trial mass empirical per-axis, per-horizon residual CDF; four prior T-DT trials only in Navfn/goal model, no fitting on collision or goal trial. Report unconditional and observer-north/south conditional diagnostics. Conditional estimate requires >=5 observations per training trial and >=2 contributing trials. Within-trial frames and phase patterns are correlated; descriptive calibration, not confidence intervals or runtime safety bounds. Fit scans use Gazebo robot pose; old Navfn recorder scans and goal consumed scans use recorded odometry. Old Navfn archives lack frozen MPPI cycles, so their recorder messages are not exact consumed inputs. Only known 0.45x0.55 m simulated obstacle.",
            "model_evidence_sha256": digest(args.model_evidence),
            "horizons_s": HORIZONS,
            "fit_trial_count": 4,
            "assessments": assessments}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-evidence", type=Path, required=True)
    parser.add_argument("--goal-trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = run(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"output": str(args.output),
                      "assessments": len(report["assessments"])}, sort_keys=True))
