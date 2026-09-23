#!/usr/bin/env python3
"""Sample only the SDF-derived sweep; keep continuous guard as final authority."""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim_stop_probe_20260923"))
from sweep_shadow import fixture_sweep

RESOLUTION_M = 0.05


def axis_samples(low, high, step):
    assert low < high and step > 0
    count = math.floor((high - low) / step)
    values = [low + i * step for i in range(count + 1)]
    if high - values[-1] > 1e-12:
        values.append(high)
    else:
        values[-1] = high
    return values


def sweep_points():
    sweep, provenance = fixture_sweep()
    xs = [p[0] for p in sweep]
    ys = [p[1] for p in sweep]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    step = RESOLUTION_M / 2
    points = [(x, y, 0.4) for x in axis_samples(x0, x1, step)
              for y in axis_samples(y0, y1, step)]
    return points, {"sweep_polygon_m": sweep, "source": provenance,
                    "resolution_m": RESOLUTION_M, "lattice_step_m": step,
                    "raster_margin_m": 0.0, "point_count": len(points)}
