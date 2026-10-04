#!/usr/bin/env python3
"""Run fixed offline scenarios; every cycle and failure is saved as reviewable evidence."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
import numpy as np
import scipy

from temporal_mpc.fixtures import SCENARIOS, reference
from temporal_mpc.oracle import step_and_audit
from temporal_mpc.solver import Config, TemporalMPC


def simulate(scenario, mode, consumption, cfg, output):
    controller = TemporalMPC(cfg, consumption)
    state = np.array(scenario.start)
    states, controls, commands, elapsed, cpu = [state.tolist()], [], [], [], []
    statuses, reasons = Counter(), Counter()
    stopped, distance, minimum, sampled_min, collision = 0., 0., float("inf"), float("inf"), False
    success_time, failures, deadline_misses, target_misses, invalid, feasible = None, 0, 0, 0, 0, 0
    trace = []
    name = f"{scenario.name}__{mode}__{consumption}__h{cfg.horizon:g}"
    trace_path = output / (name + ".jsonl")
    trace_path.touch(exist_ok=False)
    times = np.arange(cfg.steps * cfg.substeps + 1) * cfg.collision_dt
    for k in range(round(scenario.duration / cfg.dt)):
        t = k * cfg.dt
        snapshot = scenario.observe(t, mode)
        epoch = round(t * 1e9)
        result = controller.solve(state, epoch, epoch, snapshot, reference(state, scenario.goal, times), domain=scenario.domain)
        next_state, lower, sampled, contact = step_and_audit(state, result.acceleration, t, cfg.dt,
                                                          scenario.truth, scenario.domain, scenario.speed_bound)
        distance += float(np.linalg.norm(np.asarray(next_state[:2]) - state[:2]))
        if np.linalg.norm(np.asarray(next_state[3:5])) < 0.05:
            stopped += cfg.dt
        minimum, sampled_min = min(minimum, lower), min(sampled_min, sampled)
        collision = collision or contact
        statuses[result.status] += 1
        reasons[result.reason] += 1
        failures += result.optimizer_failures
        deadline_misses += int(result.deadline_miss)
        target_misses += int(result.elapsed_s > 0.05)
        invalid += int(result.status == "invalid_input_brake")
        feasible += int(result.model_feasible)
        elapsed.append(result.elapsed_s)
        cpu.append(result.cpu_s)
        controls.append(result.acceleration.tolist())
        commands.append(result.command.tolist())
        trace.append({"cycle": k, "evaluation_ns": epoch, "source_ns": snapshot.source_ns,
                      "initial": state.tolist(), "command": result.command.tolist(),
                      "acceleration": result.acceleration.tolist(), "status": result.status,
                      "reason": result.reason, "model_feasible": result.model_feasible,
                      "constraint_min": result.constraint_min, "objective": result.objective,
                      "elapsed_s": result.elapsed_s, "cpu_s": result.cpu_s,
                      "physical_clearance_lower": lower, "physical_clearance_sampled": sampled,
                      "physical_contact": contact})
        with trace_path.open("a") as stream:
            stream.write(json.dumps(trace[-1], allow_nan=False) + "\n")
        state = np.array(next_state)
        states.append(state.tolist())
        reached = (np.linalg.norm(state[:2] - scenario.goal[:2]) <= 0.12
                   and abs(np.arctan2(np.sin(state[2] - scenario.goal[2]), np.cos(state[2] - scenario.goal[2]))) <= 0.10
                   and np.max(np.abs(state[3:])) <= 0.05)
        if reached:
            success_time = (k + 1) * cfg.dt
            break
        if contact:
            break  # Preserve first contact; do not continue through a physical obstacle.
    duration = len(controls) * cfg.dt
    velocity = np.asarray(states)[:, 3:]
    acceleration = np.asarray(controls)
    delta = np.diff(np.asarray(commands), axis=0)
    report = {
        "scenario": scenario.name, "geometry_mode": mode, "consumption": consumption,
        "model_goal_reached": success_time is not None, "ros_action_success": None,
        "physical_collision": collision, "minimum_physical_clearance_lower_m": minimum,
        "minimum_physical_clearance_sampled_m": sampled_min, "completion_time_s": success_time,
        "observed_duration_s": duration, "average_speed_m_s": distance / duration,
        "travel_distance_m": distance, "detour_distance_m": max(0., distance - np.linalg.norm(state[:2] - np.asarray(scenario.start[:2]))),
        "stop_wait_duration_s": stopped, "recovery_count": None, "optimizer_failure_count": failures,
        "configured_deadline_miss_count": deadline_misses, "target_20hz_deadline_miss_count": target_misses,
        "cpu_process_s": sum(cpu), "cpu_compute_fraction_of_simulated_duration": sum(cpu) / duration,
        "solve_p50_s": float(np.percentile(elapsed, 50)), "solve_p95_s": float(np.percentile(elapsed, 95)), "solve_max_s": max(elapsed),
        "max_abs_velocity_xyz": np.max(np.abs(velocity), axis=0).tolist(),
        "rms_velocity_xyz": np.sqrt(np.mean(velocity**2, axis=0)).tolist(),
        "max_abs_acceleration_xyz": np.max(np.abs(acceleration), axis=0).tolist(),
        "max_abs_command_delta_xyz": np.max(np.abs(delta), axis=0).tolist() if len(delta) else [0., 0., 0.],
        "max_abs_jerk_xyz": (np.max(np.abs(np.diff(acceleration, axis=0)), axis=0) / cfg.dt).tolist() if len(acceleration) > 1 else [0., 0., 0.],
        "false_block_count": None, "false_block_duration_s": None,
        "false_block_reason": "No matched STVL + MPPI safe completion witness in this offline experiment",
        "statuses": dict(statuses), "reasons": dict(reasons), "invalid_input_cycles": invalid,
        "model_feasible_cycles": feasible, "cycles": len(controls), "terminal_state": state.tolist(),
        "offline_physical_task_gate": bool(success_time is not None and not collision and minimum >= 0.05),
        "deployment_accepted": False,
    }
    (output / (name + ".json")).write_text(json.dumps({"scenario": asdict(scenario), "config": asdict(cfg), "report": report}, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="+", choices=[s.name for s in SCENARIOS], default=[s.name for s in SCENARIOS])
    parser.add_argument("--modes", nargs="+", choices=["observed_polygon", "nominal_diameter"], default=["observed_polygon", "nominal_diameter"])
    parser.add_argument("--consumptions", nargs="+", choices=["temporal", "current_only", "future_union"], default=["temporal"])
    parser.add_argument("--horizons", nargs="+", type=float, default=[1.5])
    args = parser.parse_args()
    # A fresh directory is mandatory: a failed experiment cannot be overwritten.
    args.output.mkdir(parents=True, exist_ok=False)
    reports = []
    started = time.perf_counter()
    configs = [Config(horizon=horizon) for horizon in args.horizons]
    source_root = Path(__file__).parent
    sources = [source_root / "run_experiment.py", *sorted((source_root / "temporal_mpc").glob("*.py"))]
    manifest = {"schema": "temporal_mpc_offline_evidence/v1", "date": "2026-10-04 Asia/Shanghai",
                "python": sys.version, "numpy": np.__version__, "scipy": scipy.__version__, "platform": platform.platform(),
                "arguments": vars(args) | {"output": str(args.output)},
                "sources_sha256": {str(p.relative_to(source_root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                "scope": "Ideal acceleration plant and current known-motion measurements. No ROS, STVL, MPPI or Gazebo execution.",
                "physical_safety_gate_m": 0.05, "nominal_control_period_s": 0.05,
                "deterministic_without_deadline_effects": True,
                "deadline_branching_is_host_dependent": True, "timings_are_host_dependent": True,
                "thread_environment": {"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    for cfg in configs:
        for scenario in SCENARIOS:
            if scenario.name not in args.scenarios:
                continue
            for mode in args.modes:
                for consumption in args.consumptions:
                    try:
                        report = simulate(scenario, mode, consumption, cfg, args.output)
                    except BaseException as error:
                        (args.output / "run_failure.json").write_text(json.dumps({"exception": type(error).__name__,
                            "reason": str(error), "scenario": scenario.name, "mode": mode, "consumption": consumption,
                            "completed_reports": reports}, indent=2) + "\n")
                        raise
                    reports.append(report)
                    print(json.dumps({key: report[key] for key in ("scenario", "geometry_mode", "consumption", "model_goal_reached",
                          "physical_collision", "minimum_physical_clearance_lower_m", "solve_p95_s", "offline_physical_task_gate")}), flush=True)
    (args.output / "summary.json").write_text(json.dumps({"reports": reports, "elapsed_wall_s": time.perf_counter() - started,
                                                          "deployment_accepted": False}, indent=2) + "\n")


if __name__ == "__main__":
    main()
