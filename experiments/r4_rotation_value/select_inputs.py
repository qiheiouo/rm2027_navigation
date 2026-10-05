"""Select only the existing A16 native-navigation window; no new scene run."""
import csv
import json
import sys
from pathlib import Path

root, output = map(Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
for scene in ("S0", "S1", "S2"):
    evidence = root / "experiments/r4_corrected_runtime_shadow/evidence"
    events = json.loads((evidence / f"{scene}_events.json").read_text())["events"]
    begin = next(e["ROS_ns"] for e in events if e["kind"] == "goal_accepted")
    end = next(e["ROS_ns"] for e in events if e["kind"] == "native_goal_result")
    with (evidence / f"{scene}_cycles.csv").open() as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        selected = [r for r in reader if begin <= int(r["acquire_ros_ns"]) < end]
    with (output / f"{scene}_native.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(selected)
    print(scene, len(selected))
