#!/usr/bin/env python3
"""Test a fixed north/south reflection of causal scan-center residuals.

The transform follows only the observer's side of the visible box. It is
evaluated leave-trial-out and on Navfn holdouts, never tuned to their truth.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from heldout_geometry_audit import digest
from historical_scan_residual_calibration import (HORIZONS, evaluate,
                                                   residual_rows, source_summary,
                                                   summarize)
from raw_scan_support_audit import static_map


def reflected(trial):
    sign = {"north": -1., "south": 1.}
    return {**trial,
            "rows": [{**row, "residual_xy_m": [row["residual_xy_m"][0],
                        sign[row["view_y"]] * row["residual_xy_m"][1]]}
                     for row in trial["rows"]],
            "source_observations": [
                {**row, "source_y_residual_m":
                    sign[row["view_y"]] * row["source_y_residual_m"]}
                for row in trial["source_observations"]]}


def source_pit(training, trial):
    pits = []
    for row in trial["source_observations"]:
        cdfs = []
        for other in training:
            values = np.asarray([item["source_y_residual_m"]
                                 for item in other["source_observations"]])
            if len(values) < 10:
                raise ValueError("insufficient training source observations")
            value = row["source_y_residual_m"]
            cdfs.append((np.count_nonzero(values < value) +
                         .5 * np.count_nonzero(values == value)) / len(values))
        pits.append(float(np.mean(cdfs)))
    return summarize(pits)


def one_assessment(training, heldout, split):
    train_reflected = [reflected(trial) for trial in training]
    test_reflected = reflected(heldout)
    plain = evaluate(training, heldout)
    mirrored = evaluate(train_reflected, test_reflected)
    return {"trial": heldout["trial"], "split": split,
            "source_sha256": heldout["source_sha256"],
            "eligible_sources": heldout["eligible_sources"],
            "observer_side_sources": source_summary(heldout),
            "source_y_pit": {"plain": source_pit(training, heldout),
                             "mirrored": source_pit(train_reflected,
                                                    test_reflected)},
            "future_y_pit": {f"{horizon:.1f}s": {
                "plain": plain[f"{horizon:.1f}s/y"],
                "mirrored": mirrored[f"{horizon:.1f}s/y"]}
                for horizon in HORIZONS}}


def run(args):
    evidence = json.loads(args.model_evidence.read_text())
    paths = [Path(item["trial"])
             for item in evidence["historical_leave_one_trial_out"]]
    if len(paths) != 6 or any(not path.exists() for path in paths):
        raise ValueError("six historical trial paths required")
    occupancy, _ = static_map()
    data = [residual_rows(path, occupancy,
                          "recorded" if index < 4 else "recorded_odom")
            for index, path in enumerate(paths)]
    goal = residual_rows(args.goal_trial, occupancy, "consumed")
    training = data[:4]
    assessments = []
    for index, heldout in enumerate(training):
        others = [trial for j, trial in enumerate(training) if j != index]
        assessments.append(one_assessment(others, heldout,
                                          "tdt_leave_one_out"))
    for heldout in [*data[4:], goal]:
        assessments.append(one_assessment(training, heldout,
                                          "unseen_navfn_or_goal"))
    return {"schema": "rm_dynamic_prediction/view_reflection_calibration/v1",
            "scope": "Fixed sign reflection of y residual when observer lies north of raw scan midpoint; south remains positive. Four T-DT trials fit an equal-trial-mass empirical CDF, two old Navfn recorder trials and a separate goal consumed trial are fit holdouts. Both mirrored and plain CDFs are reported; future Gazebo box centers evaluate only. This geometric symmetry hypothesis has no fitted parameters, but holdouts were seen in prior research, frames are correlated, and no runtime probability or safety claim follows.",
            "model_evidence_sha256": digest(args.model_evidence),
            "horizons_s": HORIZONS,
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
    print(json.dumps({"assessments": len(report["assessments"]),
                      "output": str(args.output)}))
