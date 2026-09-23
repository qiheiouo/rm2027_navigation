#!/usr/bin/env python3
"""Experiment-only planner occupancy derived from the fixture's mechanical sweep."""
import copy
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim_stop_probe_20260923"))
from sweep_shadow import fixture_sweep

RESOLUTION_M = 0.05
SOURCE = "fixture_sweep"


def sweep_points():
    sweep, provenance = fixture_sweep()
    xs = [p[0] for p in sweep]
    ys = [p[1] for p in sweep]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    # A half-cell grid at half-cell spacing covers every costmap cell whose
    # closed area intersects the known sweep, despite rolling-grid alignment.
    step = RESOLUTION_M / 2
    margin = RESOLUTION_M
    nx = math.ceil((x1 - x0 + 2 * margin) / step) + 1
    ny = math.ceil((y1 - y0 + 2 * margin) / step) + 1
    points = [(x0 - margin + i * step, y0 - margin + j * step, 0.4)
              for i in range(nx) for j in range(ny)]
    return points, {"sweep_polygon_m": sweep, "source": provenance,
                    "resolution_m": RESOLUTION_M, "lattice_step_m": step,
                    "raster_margin_m": margin, "point_count": len(points)}


def patch_profile(original):
    result = copy.deepcopy(original)
    for name in ("local_costmap", "global_costmap"):
        p = result[name][name]["ros__parameters"]
        assert abs(p["resolution"] - RESOLUTION_M) < 1e-9
        assert p["plugins"] == ["obstacle_layer", "inflation_layer"]
        p["plugins"] = ["obstacle_layer", "fixture_sweep_layer", "inflation_layer"]
        p["fixture_sweep_layer"] = {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "enabled": True,
            "footprint_clearing_enabled": False,
            "observation_sources": SOURCE,
            SOURCE: {
                "topic": "/fixture_sweep/points",
                "data_type": "PointCloud2",
                "marking": True,
                "clearing": False,
                "obstacle_min_range": 0.0,
                "obstacle_max_range": 20.0,
                "min_obstacle_height": 0.0,
                "max_obstacle_height": 2.0,
                "observation_persistence": 1.0,
                "expected_update_rate": 0.0,
            },
        }
    return result
