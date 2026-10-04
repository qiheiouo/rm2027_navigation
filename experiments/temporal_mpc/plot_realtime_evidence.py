#!/usr/bin/env python3
"""Render the recorded QP shadow evidence; no solver or parameter changes."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def plot(directory):
    reports = json.loads((directory / "summary.json").read_text())["reports"]
    records = [json.loads(line) for p in directory.glob("*.jsonl") for line in p.read_text().splitlines()]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), layout="constrained")
    fig.suptitle("T-DT frontend + temporal QP: offline fixed-yaw shadow evidence", fontsize=16)
    ax = axes[0, 0]
    values = np.array([r["control_elapsed_s"] * 1000 for r in records])
    ax.hist(values, bins=40, color="#3275a8")
    ax.axvline(40, color="#dd9132", label="core budget 40 ms")
    ax.axvline(50, color="#c63c36", label="20 Hz period 50 ms")
    ax.set(xlabel="Window + QP + validation latency (ms)", ylabel="Control cycles", xlim=(0, 53),
           title=f"{len(records):,} cycles; P95 {np.percentile(values, 95):.2f} ms; max {values.max():.2f} ms")
    ax.legend()
    ax = axes[0, 1]
    names = list(dict.fromkeys(r["scenario"] for r in reports if r["geometry_mode"] != "static_map"))
    by = {(r["scenario"], r["geometry_mode"]): r for r in reports}
    pos = np.arange(len(names))
    for offset, mode, color, label in [(-.18, "observed_polygon", "#3275a8", "Observed"),
                                      (.18, "nominal_diameter", "#dd9132", "Nominal full D")]:
        ax.bar(pos + offset, [by[name, mode]["minimum_physical_clearance_lower_m"] for name in names],
               width=.36, color=color, label=label)
    ax.axhline(.05, color="#c63c36", label="Physical gate 0.05 m")
    ax.set_xticks(pos, names, rotation=65, ha="right", fontsize=8)
    ax.set(ylabel="Minimum swept physical clearance lower bound (m)",
           title="Two head-on contacts; arrival alone does not pass")
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    wait = [json.loads(line) for line in (directory / "wait_then_pass__observed_polygon.jsonl").read_text().splitlines()]
    ax.plot([r["epoch_ns"] * 1e-9 for r in wait], [r["initial"][0] for r in wait], color="#3275a8", label="Robot x")
    ax.axhline(1.3, color="#dd9132", label="Crossing x")
    ax.axvspan((.7 - .605) / .3, (.7 + .605) / .3, color="#dd9132", alpha=.15,
               label="Obstacle intersects path footprint band")
    ax.set(xlabel="Ideal simulated time (s)", ylabel="World x (m)",
           title="Wait then pass: MPC proposals; MPPI handoff not executed")
    ax.legend(fontsize=9)
    ax = axes[1, 1]
    motion = [json.loads(line) for line in (directory / "static_map_detour__static_map.jsonl").read_text().splitlines()]
    route = json.loads((directory / "static_map_detour__static_map__route.json").read_text())["tdt_output"]["path"]
    route, motion = np.array(route), np.array([r["initial"][:2] for r in motion])
    ax.plot(route[:, 0], route[:, 1], "--", color="#666666", label="Migrated T-DT reference")
    ax.plot(motion[:, 0], motion[:, 1], color="#3275a8", label="QP executed centre")
    ax.add_patch(Rectangle((2.5, -.35), .5, .7, facecolor="#dd9132", label="Raw static wall"))
    ax.set(xlabel="World x (m)", ylabel="World y (m)", title="Static topology comes from the planner")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=9)
    for ax in axes.flat:
        ax.grid(alpha=.15)
    fig.savefig(directory / "overview.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    plot(parser.parse_args().directory)
