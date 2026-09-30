#!/usr/bin/env python3
"""Keep the Gazebo pose recorder alive for a fixed simulated-time tail."""
import argparse
import json
from pathlib import Path
import time


def last_complete_stamp(path):
    with path.open("rb") as stream:
        stream.seek(0, 2)
        size = stream.tell()
        if not size:
            raise ValueError("physical pose stream is empty")
        stream.seek(max(0, size - 131072))
        lines = stream.read().splitlines()
    complete = lines if lines and lines[-1].rstrip().endswith(b"}") else lines[:-1]
    for line in reversed(complete):
        try:
            stamp = json.loads(line)["header"]["stamp"]
            return float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
        except (ValueError, KeyError, TypeError):
            continue
    raise ValueError("no complete physical pose sample in file tail")


def wait_for_tail(path, duration_s, timeout_s):
    start = last_complete_stamp(path)
    target = start + duration_s
    deadline = time.monotonic() + timeout_s
    current = start
    while current < target and time.monotonic() < deadline:
        time.sleep(.1)
        current = last_complete_stamp(path)
    result = {"schema": "rm_dynamic_prediction_physical_tail/v1",
              "start_sim_s": start, "target_sim_s": target,
              "final_sim_s": current, "tail_complete": current >= target}
    print(json.dumps(result))
    return result["tail_complete"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poses", type=Path)
    parser.add_argument("duration_s", type=float)
    parser.add_argument("--timeout-s", type=float, default=180.)
    args = parser.parse_args()
    if not 3.1 <= args.duration_s <= 10 or args.timeout_s <= 0:
        parser.error("tail must cover the 3 s MPPI horizon with margin")
    raise SystemExit(0 if wait_for_tail(args.poses, args.duration_s,
                                        args.timeout_s) else 2)
