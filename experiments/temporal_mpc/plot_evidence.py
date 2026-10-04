#!/usr/bin/env python3
"""Optional static research plot from the recorded matrix; no control dependencies."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot(directory, output):
    rows = json.loads((directory / "matrix/summary.json").read_text())["reports"]
    modes = ["observed_polygon", "nominal_diameter"]
    names = [r["scenario"] for r in rows if r["geometry_mode"] == modes[0]]
    colors = ["#247b8c", "#c26937"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    for mode, color, offset in zip(modes, colors, [-.15, .15]):
        subset = [next(r for r in rows if r["scenario"] == name and r["geometry_mode"] == mode) for name in names]
        x = np.arange(len(names)) + offset
        axes[0, 0].scatter(x, [r["minimum_physical_clearance_lower_m"] for r in subset], color=color, label=mode, s=38)
        axes[0, 1].bar(x, [r["target_20hz_deadline_miss_count"] / r["cycles"] for r in subset], width=.3, color=color, label=mode)
        axes[1, 0].bar(x, [int(r["offline_physical_task_gate"]) for r in subset], width=.3, color=color)
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.set_xticks(np.arange(len(names)), [name.replace("_", " ") for name in names], rotation=55, ha="right", fontsize=8)
        ax.grid(axis="y", alpha=.2)
    axes[0, 0].axhline(.05, color="#933333", linestyle="--", linewidth=1, label="0.05 m gate")
    axes[0, 0].set_title("Independent swept mechanical clearance (lower bound)")
    axes[0, 0].set_ylabel("metres")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set_title("Compute cycles exceeding the 50 ms target")
    axes[0, 1].set_ylabel("fraction of cycles")
    axes[0, 1].set_ylim(0, 1.05)
    axes[1, 0].set_title("Goal + no contact + physical clearance only")
    axes[1, 0].set_ylabel("physical task gate (not deployment acceptance)")
    axes[1, 0].set_yticks([0, 1])
    for mode, color in zip(modes, colors):
        path = directory / "matrix" / f"wait_then_pass__{mode}__temporal__h1.5.jsonl"
        trace = [json.loads(line) for line in path.read_text().splitlines()]
        axes[1, 1].plot([r["evaluation_ns"] * 1e-9 for r in trace], [r["initial"][0] for r in trace], color=color, label=mode)
    axes[1, 1].axhline(4.2, color="#666666", linestyle="--", label="goal x")
    axes[1, 1].set_title("Wait-then-pass: executed progress")
    axes[1, 1].set_xlabel("simulation time (s)")
    axes[1, 1].set_ylabel("x (m)")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(alpha=.2)
    fig.suptitle("Temporal MPC fixed offline matrix — ideal plant, no STVL/MPPI comparison", fontsize=14)
    fig.savefig(output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot(args.directory, args.output)
