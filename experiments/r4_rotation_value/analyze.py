"""Effect-oriented A18 summary. No hash registry or broad regression suite."""
import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), median=statistics.median(values),
                p95=values[min(len(values)-1, int(.95*len(values)))], max=max(values))


def summary(rows):
    valid = [r for r in rows if r["valid"] == "1"]
    return dict(samples=len(rows), valid=len(valid), valid_ratio=len(valid)/len(rows),
        reasons=dict(Counter(r["reason"] for r in rows)),
        solver_ms=stats([float(r["solver_ms"]) for r in rows if float(r["solver_ms"]) > 0]),
        elapsed_ms=stats([float(r["elapsed_ms"]) for r in rows]),
        warm=sum(r["used_warm"] == "1" for r in valid),
        nominal_nonzero=sum(float(r["nominal_cost"]) > 1e-8 for r in rows),
        valid_dynamic_nonzero=sum(float(r["solved_cost"]) > 1e-8 for r in valid),
        solved_cost=stats([float(r["solved_cost"]) for r in valid]),
        cost_increased=sum(float(r["nominal_cost"]) > 1e-8 and
            float(r["solved_cost"]) > float(r["nominal_cost"]) + 1e-4 for r in valid),
        absolute_wz=stats([abs(float(r["wz"])) for r in valid]),
        predicted_overlap_samples=sum(float(r["minimum_clearance"]) < 0 for r in valid))


def flatten(value):
    return [n for row in value for n in row]


folder = Path(sys.argv[1])
before = json.loads((folder/"fixed_before.json").read_text()) if (folder/"fixed_before.json").exists() else []
after = json.loads((folder/"fixed_after.json").read_text()) if (folder/"fixed_after.json").exists() else []
fixed = []
for a, b in zip(before, after, strict=True):
    controls = max(abs(x-y) for x,y in zip(flatten(a["controls"]), flatten(b["controls"]), strict=True))
    stages = max(abs(x-y) for x,y in zip(flatten(a["stages"]), flatten(b["stages"]), strict=True))
    assert controls < 2e-5 and stages < 2e-5
    fixed.append(dict(scenario=a["scenario"], max_control_difference=controls, max_stage_difference=stages))
replay = read(folder/"replay.csv") if (folder/"replay.csv").exists() else []
probe = read(folder/"probe.csv") if (folder/"probe.csv").exists() else []
result = dict(stage="Research", scope="offline recorded values and synthetic stationary probes; no actual output",
              fixed_baseline_comparison=fixed, recorded={}, probes={})
for scene in ("S0", "S1", "S2"):
    selected = [r for r in replay if r["scene"] == scene]
    if not selected:
        continue
    value = summary(selected)
    value["recorded_fixed_baseline_valid"] = sum(r["baseline_valid"] == "1" for r in selected)
    value["absolute_horizon_yaw_delta"] = stats([abs(float(r["yaw_delta"])) for r in selected if r["valid"] == "1"])
    result["recorded"][scene] = value
for case in sorted(set(r["case"] for r in probe)):
    selected = [r for r in probe if r["case"] == case]
    value = summary(selected)
    value["rows"] = selected if len(selected) == 1 else [selected[0], selected[29], selected[30], selected[-1]]
    if case in ("hold_clear_sequence", "locked_future_hold_clear"):
        value["hold"] = summary(selected[:30])
        value["clear"] = summary(selected[30:])
        if case == "locked_future_hold_clear":
            for phase, phase_rows in (("hold", selected[:30]), ("clear", selected[30:])):
                value[phase]["virtual_world_forward_mps"] = stats([
                    float(r["vx"])*math.cos(float(r["yaw_end"])) -
                    float(r["vy"])*math.sin(float(r["yaw_end"])) for r in phase_rows if r["valid"] == "1"])
        value["max_virtual_seed_delta"] = {
            axis: max((abs(float(b[axis])-float(a[axis])) for a,b in zip(selected, selected[1:])
                      if a["valid"] == b["valid"] == "1"), default=None) for axis in ("vx", "vy", "wz")}
    result["probes"][case] = value
(folder/"summary.json").write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps({"recorded":result["recorded"], "probe_valid":{
    k:(v["valid"],v["samples"]) for k,v in result["probes"].items()}}, indent=2))
