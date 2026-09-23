#!/usr/bin/env python3
"""Fixture-only command scaler with next-cycle and odom-age stop reserve."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "dynamic_guard_pilot_20260923"))
from guard_node import Guard
from horizon_core import BISECTION_STEPS, scale_command


class HorizonGuard(Guard):
    def __init__(self, profile, output):
        super().__init__(profile, output)
        params = yaml.safe_load(profile.read_text())
        self.frequency_hz = params["velocity_smoother"]["ros__parameters"]["smoothing_frequency"]
        assert self.frequency_hz == 20.0
        self.stream.write(json.dumps({
            "kind": "policy", "name": "same_direction_scaling_with_next_cycle_reserve",
            "smoother_frequency_hz": self.frequency_hz,
            "next_interval_s": 1.0 / self.frequency_hz,
            "bisection_steps": BISECTION_STEPS,
            "fallback": "zero_when_odom_missing_or_stop_reserve_absent"}) + "\n")

    def command_cb(self, msg):
        now = self.sim_time()
        proposed = self.vector(msg)
        self.last_input_sim = now
        record = {"kind": "decision", "sim_s": now, "steady_ns": time.monotonic_ns(),
                  "proposed": [v if math.isfinite(v) else None for v in proposed]}
        scale = 0.0
        if self.odom is None or now <= 0:
            record["reason"] = "missing_odom"
        else:
            pose, speed, stamp = self.odom
            age = now - stamp
            record.update(pose=pose, odom_speed=speed, odom_stamp_s=stamp, odom_age_s=age)
            if age < -0.02 or age > 0.1:
                record["reason"] = "stale_odom"
            else:
                try:
                    scale, selected, full, reason, hold = scale_command(
                        pose, speed, proposed, self.body, self.sweep,
                        age, self.frequency_hz)
                    record.update(reason=reason, response_horizon_s=hold,
                                  certificate=selected, full_command_certificate=full)
                except ValueError as error:
                    record["reason"] = "invalid_input"
                    record["error"] = str(error)
        output = tuple(scale * v for v in proposed) if scale > 0 else (0.0, 0.0, 0.0)
        self.emit(output)
        record.update(scale=scale, emitted=output)
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")


def main():
    rclpy.init()
    node = HorizonGuard(Path(sys.argv[1]), Path(sys.argv[2]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
