#!/usr/bin/env python3
"""Check whether the frozen source history signals a coming turn."""
import argparse
import json
from pathlib import Path

import numpy as np

from heldout_geometry_audit import digest, recorded_messages
from raw_scan_support_audit import static_map
from scan_midpoint_soft_probe import scan_sources


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    score = json.loads(args.score_evidence.read_text())
    target = next(row for row in score["cases"]
                  if row["name"] == "intrusion_162")
    source = target["scan_source_t"]
    occupancy, _ = static_map()
    scan_hashes, scans = scan_sources(args.trial, occupancy)
    history = [row for row in scans if 0 <= source - row["source_t"] <= .4][-5:]
    if len(history) != target["scan_recent_count"] or len(history) < 4:
        raise ValueError("frozen recent scan history differs")
    t = np.asarray([row["source_t"] - source for row in history])
    y = np.asarray([row["raw_bbox_mid"][1] for row in history])
    quadratic = np.polyfit(t, y, 2)
    linear = np.polyfit(t, y, 1)
    if abs(linear[0] - target["scan_velocity_xy"][1]) > 1e-9:
        raise ValueError("causal scan velocity changed")
    tracker = []
    for row in recorded_messages(args.trial):
        if not 0 <= source - row["source_t"] <= .4:
            continue
        confirmed = [track for track in row["tracks"]
                     if track["state"] == 2]
        if len(confirmed) == 1:
            tracker.append({"source_t": row["source_t"],
                            "velocity_y_mps": confirmed[0]["vxy"][1]})
    if len(tracker) < 4:
        raise ValueError("insufficient recent confirmed tracker messages")
    tt = np.asarray([row["source_t"] - source for row in tracker])
    vv = np.asarray([row["velocity_y_mps"] for row in tracker])
    tracker_slope = np.polyfit(tt, vv, 1)[0]
    causal = {
        "source_t": source,
        "observer_view_y": target["observer_view_y"],
        "scan_history": [{"source_t": row["source_t"],
                          "midpoint_y_m": row["raw_bbox_mid"][1]}
                         for row in history],
        "scan_linear_velocity_y_mps": float(linear[0]),
        "scan_quadratic_source_center_y_m": float(quadratic[2]),
        "scan_quadratic_tangent_velocity_y_mps": float(quadratic[1]),
        "scan_quadratic_acceleration_y_mps2": float(2 * quadratic[0]),
        "tracker_history": tracker,
        "tracker_velocity_y_trend_mps2": float(tracker_slope)}
    # The future residual is a label and is read only after the causal fit.
    residual = json.loads(args.residual_evidence.read_text())
    if residual["source_t"] != source:
        raise ValueError("future label belongs to another source")
    sign = -1. if target["observer_view_y"] == "north" else 1.
    future = []
    for step in residual["steps"]:
        horizon = step["source_horizon_s"]
        projected = sign * (np.polyval(quadratic, horizon) -
                            y[-1] - linear[0] * horizon)
        future.append({"step": step["step"], "source_horizon_s": horizon,
                       "quadratic_vs_source_anchored_linear_y_shift_m":
                           float(projected),
                       "true_canonical_y_residual_m": step[
                           "canonical_true_residual_xy_m"][1]})
    result = {
        "schema": "rm_dynamic_prediction/causal_turn_signal_audit/v1",
        "scope": "One frozen source at cycle 162. Fit a quadratic to only the last four eligible source-scan midpoints and a line to recent confirmed tracker vy, without future obstacle truth. Compare the quadratic displacement with previously frozen future truth residual labels afterward. This is a local observability diagnostic, not a calibrated acceleration predictor or runtime proposal.",
        "source_sha256": {"score_evidence": digest(args.score_evidence),
                          "residual_evidence": digest(args.residual_evidence),
                          "scans": scan_hashes,
                          "recorded_predictions": digest(
                              args.trial / "predictions.jsonl")},
        "causal": causal,
        "future_label_comparison": future}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"eligible_scans": len(history),
                      "confirmed_tracker_messages": len(tracker),
                      "output": str(args.output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--score-evidence", type=Path, required=True)
    parser.add_argument("--residual-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())
