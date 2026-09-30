#!/usr/bin/env python3
"""Replay frozen Nav2 critic prefixes and attribute filtered cycle-613 ranks."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from batch_sampling_probe import digest

ROOT = Path(__file__).resolve().parents[3]
WORK = ROOT / "build"
FIXTURE = WORK / "dynamic_prediction_phase2_holdout_20260929_setupfix/candidate_navfn_1/filtered_fixture_613"
NATIVE = WORK / "phase2_native_mask_head_20260929/install/costmap_mask_probe_cpp/lib/costmap_mask_probe_cpp/frozen_critic_score"
INSTALL = WORK / "phase2_native_mask_head_20260929/install/setup.bash"
EVIDENCE = ROOT / "docs/dynamic_navigation/evidence/phase2_holdout_20260929"
OVERLAP = ROOT / "docs/dynamic_navigation/evidence/cycle613_face_xy_overlap_20260930"
IMAGE = "rm2027_navigation:dynamic-prediction-20260924"
IMAGE_ID = "sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6"
CRITICS = ("ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
           "PathAlignCritic", "PathFollowCritic", "PathAngleCritic")


def check_clean():
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("native attribution rule must be committed")
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                   text=True).strip()


def image_id():
    return subprocess.check_output(["docker", "image", "inspect", "--format",
                                    "{{.Id}}", IMAGE], text=True).strip()


def source_hashes():
    fixed = json.loads((EVIDENCE / "filtered_summary.json").read_text())
    fixture = json.loads((EVIDENCE / "filtered_inputs.json").read_text())
    paths = {"native_binary": NATIVE,
             "filtered_meta.json": FIXTURE / "filtered_meta.json",
             "filtered_map_and_poses.bin": FIXTURE / "filtered_map_and_poses.bin",
             "filtered_controls.bin": FIXTURE / "filtered_controls.bin",
             "filtered_native_scores.bin": EVIDENCE / "filtered_native_scores.bin",
             "filtered_candidates.csv": EVIDENCE / "filtered_candidates.csv",
             "overlap_summary.json": OVERLAP / "summary.json",
             "overlap_candidates.csv": OVERLAP / "candidates.csv"}
    hashes = {name: digest(path) for name, path in paths.items()}
    if hashes["native_binary"] != fixed["input_sha256"]["native_binary"] or \
            hashes["filtered_native_scores.bin"] != fixed["input_sha256"]["filtered_native_scores"] or \
            hashes["filtered_candidates.csv"] != fixed["detail_sha256"]:
        raise ValueError("prior native evidence changed")
    for name, expected in fixture["fixture_sha256"].items():
        if hashes[name] != expected:
            raise ValueError(f"filtered native fixture changed: {name}")
    overlap = json.loads((OVERLAP / "summary.json").read_text())
    if hashes["overlap_candidates.csv"] != overlap["details_sha256"] or \
            overlap["weighted_total_rank"]["top_rollout"] != 190:
        raise ValueError("fixed overlap result changed")
    return hashes


def prepare(output):
    if output.exists():
        raise FileExistsError(output)
    commit = check_clean()
    hashes = source_hashes()
    if image_id() != IMAGE_ID:
        raise ValueError("runtime image changed")
    meta = json.loads((FIXTURE / "filtered_meta.json").read_text())
    if tuple(meta["parameters"]["FollowPath.critics"]) != CRITICS or meta["batch"] != 300:
        raise ValueError("original critic order or batch changed")
    output.mkdir(parents=True)
    derived = {}
    for n in range(1, 8):
        single = json.loads(json.dumps(meta))
        single["parameters"]["FollowPath.critics"] = list(CRITICS[:n])
        path = output / f"prefix_{n}_meta.json"
        path.write_text(json.dumps(single, indent=2, sort_keys=True) + "\n")
        derived[path.name] = digest(path)
    plan = {"schema": "rm_dynamic_prediction_cycle613_native_terms_plan/v1",
            "scope": "Seven sequential prefixes of original native critic list; fixed filtered poses and raw costmap.",
            "evaluation_commit": commit, "image_id": IMAGE_ID,
            "source_sha256": hashes, "derived_meta_sha256": derived,
            "original_critics": CRITICS}
    (output / "plan.json").write_text(json.dumps(plan, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"fixture": str(output), "commit": commit,
                      "prefixes": len(derived)}))


def mounted(path):
    return "/work/" + str(path.resolve().relative_to(WORK))


def run_native(output):
    plan = json.loads((output / "plan.json").read_text())
    if check_clean() != plan["evaluation_commit"] or image_id() != plan["image_id"] or \
            source_hashes() != plan["source_sha256"]:
        raise ValueError("frozen native run input changed")
    for name, expected in plan["derived_meta_sha256"].items():
        if digest(output / name) != expected:
            raise ValueError(f"derived prefix changed: {name}")
    for n in range(1, 8):
        scores = output / f"prefix_{n}_scores.bin"
        log = output / f"prefix_{n}.log"
        if scores.exists() or log.exists():
            raise FileExistsError(f"prefix already attempted: {n}")
        cmd = (f"source {mounted(INSTALL)} && {mounted(NATIVE)} "
               f"{mounted(output / f'prefix_{n}_meta.json')} "
               f"{mounted(FIXTURE / 'filtered_map_and_poses.bin')} "
               f"{mounted(FIXTURE / 'filtered_controls.bin')} {mounted(scores)}")
        args = ["docker", "run", "--rm", "--init", "--network", "none",
                "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
                "--pids-limit", "1024", "--security-opt", "no-new-privileges",
                "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
                "-e", "ROS_DOMAIN_ID=177", "-e", "ROS_LOCALHOST_ONLY=1",
                "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1",
                "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
                "--entrypoint", "bash", IMAGE, "-c", cmd]
        with log.open("x") as stream:
            status = subprocess.call(args, stdout=stream, stderr=subprocess.STDOUT)
        if status != 0:
            raise RuntimeError(f"native scorer prefix {n} failed with {status}; see {log}")
        if scores.stat().st_size != 300 * 4:
            raise ValueError(f"native scorer prefix {n} result length differs")
        print(json.dumps({"prefix": n, "score_sha256": digest(scores)}))


def auc(values, safe, bad):
    differences = values[safe, None] - values[None, bad]
    return float(np.mean(differences < -1e-3) +
                 .5 * np.mean(np.abs(differences) <= 1e-3))


def evaluate(output, evidence):
    if evidence.exists():
        raise FileExistsError(evidence)
    plan = json.loads((output / "plan.json").read_text())
    if check_clean() != plan["evaluation_commit"] or \
            source_hashes() != plan["source_sha256"]:
        raise ValueError("native attribution inputs changed")
    cumulative = np.stack([np.fromfile(output / f"prefix_{n}_scores.bin", dtype="<f4")
                           for n in range(1, 8)])
    if cumulative.shape != (7, 300) or not np.isfinite(cumulative).all():
        raise ValueError("native scorer outputs invalid")
    full = np.fromfile(EVIDENCE / "filtered_native_scores.bin", dtype="<f4")
    parity = float(np.max(np.abs(cumulative[-1] - full)))
    if parity > 1e-3:
        raise ValueError(f"seven-prefix total does not reproduce sealed native score: {parity}")
    terms = np.diff(np.vstack((np.zeros((1, 300), dtype=np.float64),
                               cumulative.astype(np.float64))), axis=0)
    with (EVIDENCE / "filtered_candidates.csv").open() as stream:
        labels = list(csv.DictReader(stream))
    with (OVERLAP / "candidates.csv").open() as stream:
        overlap = list(csv.DictReader(stream))
    safe = np.asarray([r["filtered_joint_safe"] == "1" for r in labels])
    bad = np.asarray([r["filtered_joint_safe"] == "0" and
                      r["native_static_collision"] == "0" for r in labels])
    if int(safe.sum()) != 3 or int(bad.sum()) != 281:
        raise ValueError("fixed safe/unsafe partition changed")
    new = json.loads((OVERLAP / "summary.json").read_text())
    grade = np.asarray([float(r["grade_score"]) for r in overlap])
    weighted = np.asarray([float(r["new_weighted_total"]) for r in overlap])
    regularizer = weighted - full.astype(np.float64) - \
        np.float32((3.81 / 254.) * 1_000_000. / 9.) - grade
    good, selected = 176, 190
    if not safe[good] or not bad[selected]:
        raise ValueError("preselected safety contrast changed")
    pair = {name: float(values[good] - values[selected]) for name, values in
            zip(CRITICS, terms)}
    pair.update({"PredictionOverlapGrade": float(grade[good] - grade[selected]),
                 "ControlRegularizer": float(regularizer[good] - regularizer[selected]),
                 "NativeTotal": float(full[good] - full[selected]),
                 "WeightedTotal": float(weighted[good] - weighted[selected])})
    group = {name: {"score_span": float(np.ptp(values)),
                    "safe_lower_than_dynamic_unsafe_auc": auc(values, safe, bad)}
             for name, values in zip(CRITICS, terms)}
    group["PredictionOverlapGrade"] = {"score_span": float(np.ptp(grade)),
        "safe_lower_than_dynamic_unsafe_auc": auc(grade, safe, bad)}
    group["ControlRegularizer"] = {"score_span": float(np.ptp(regularizer)),
        "safe_lower_than_dynamic_unsafe_auc": auc(regularizer, safe, bad)}
    evidence.mkdir(parents=True)
    detail = evidence / "terms.csv"
    with detail.open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("rollout", "joint_safe", "dynamic_unsafe_static_clear",
                         *CRITICS, "PredictionOverlapGrade", "ControlRegularizer",
                         "WeightedTotal"))
        for i in range(300):
            writer.writerow((i, int(safe[i]), int(bad[i]),
                             *(term[i] for term in terms), grade[i], regularizer[i],
                             weighted[i]))
    report = {"schema": "rm_dynamic_prediction_cycle613_native_terms/v1",
              "scope": "Operational differences of sequential native critic prefixes on fixed individually filtered trajectories. Already inspected one cycle; no weight changes.",
              "evaluation_commit": plan["evaluation_commit"],
              "input_sha256": {**plan["source_sha256"], "plan": digest(output / "plan.json"),
                               **{f"prefix_{i}": digest(output / f"prefix_{i}_scores.bin")
                                  for i in range(1, 8)}},
              "native_total_max_abs_parity_error": parity,
              "safe_count": int(safe.sum()), "dynamic_unsafe_static_clear_count": int(bad.sum()),
              "fixed_safe_rollout": good, "fixed_unsafe_top_rollout": selected,
              "safe_minus_unsafe_pair": pair, "terms": group,
              "detail_sha256": digest(detail),
              "prior_overlap_candidate_truth": new["candidate_truth"]}
    (evidence / "summary.json").write_text(json.dumps(report, indent=2,
                                                       sort_keys=True) + "\n")
    print(json.dumps({"native_parity": parity, "pair": pair,
                      "terms": group}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run", "evaluate"))
    parser.add_argument("fixture", type=Path)
    parser.add_argument("evidence", nargs="?", type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.fixture.resolve())
    elif args.command == "run":
        run_native(args.fixture.resolve())
    else:
        if args.evidence is None:
            parser.error("evaluate requires evidence directory")
        evaluate(args.fixture.resolve(), args.evidence.resolve())
