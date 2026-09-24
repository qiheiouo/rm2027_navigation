"""Small independent trace fixture for geometry, score, and ranking gates."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
import yaml

import analyze
import rank_probe


class FrozenCycleTest(unittest.TestCase):
    def test_overlap_area_is_graded_for_partial_occupancy(self):
        square = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)]
        box = analyze.predicted_box((1., .5), (0., 0.), (0., 0.),
                                    (.5, .5), 0., 0., 0.)
        self.assertAlmostEqual(rank_probe.overlap_area(square, box), .5)

    def test_safe_candidate_and_collision_score(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            cycle = root / "cycle_0.json"
            cycle.write_text("{}")
            cycle.with_suffix(".bin").write_bytes(b"")
            profile = root / "profile.yaml"
            profile.write_text(yaml.safe_dump({
                "controller_server": {"ros__parameters": {"FollowPath": {
                    "model_dt": .1, "batch_size": 2, "time_steps": 2,
                    "iteration_count": 1,
                    "PredictionV1Critic": {"horizon": .2, "max_age": .4,
                        "object_width": .45, "object_height": .55,
                        "reference_acceleration": 0.0}}}},
                "local_costmap": {"local_costmap": {"ros__parameters": {
                    "footprint": "[[-0.2,-0.2],[0.2,-0.2],[0.2,0.2],[-0.2,0.2]]",
                    "footprint_padding": .03}}}}))
            truth = root / "truth.jsonl"
            truth.write_text("\n")
            collision_score = (3.81 / 254.) * 1_000_000 / 2
            prediction = {"status": "accepted", "schema": "rm_dynamic_obstacle_predictions/v1",
                          "authority": "shadow_only", "complete": True, "frame": "odom",
                          "source_age_s": 0., "consumer_sim_s": 0.,
                          "tracks": [{"state": 2, "xy": [2., 0.], "vxy": [0., 0.],
                                      "size_xy": [0., 0.]}]}
            meta = {"schema": "rm_dynamic_prediction_cycle/v1", "cycle_id": 0,
                    "pose": [0., 0., 0.],
                    "map": {"width": 1, "height": 1, "frame": "odom"},
                    "path": [[0., 0., 0.]], "padded_footprint":
                        [[-.23, -.23], [.23, -.23], [.23, .23], [-.23, .23]],
                    "events": [{"kind": "prediction.input", "value": prediction},
                               {"kind": "settings", "value": {"batch": 2, "steps": 2,
                                                                "dt": .1, "offset": 0}},
                               {"kind": "output", "value": [0., 0., 0.]}],
                    "request_steady_ns": 0, "finish_steady_ns": 1_000_000,
                    "observer_copy_ns": 100_000}
            arrays = [
                ("locked.raw_map", np.zeros((1, 1), dtype=np.uint8)),
                ("rollout.x", np.array([[0., 0.], [0., 2.]], dtype=np.float32)),
                ("rollout.y", np.zeros((2, 2), dtype=np.float32)),
                ("rollout.yaw", np.zeros((2, 2), dtype=np.float32)),
                ("critic.FollowPath.CostCritic", np.array([0., 0.], dtype=np.float32)),
                ("critic.FollowPath.PredictionV1Critic",
                 np.array([0., collision_score], dtype=np.float32)),
                ("scored.costs", np.array([0., collision_score], dtype=np.float32)),
                ("weighted.costs", np.array([0., collision_score], dtype=np.float32)),
                ("weighted.probability", np.array([1., 0.], dtype=np.float32)),
                ("cost_critic.collisions", np.array([0, 0], dtype=np.uint8)),
            ]
            for name in ("initial", "before_filter", "after_filter"):
                for key in ("vx", "vy", "wz"):
                    arrays.append((f"{name}.{key}", np.zeros(2, dtype=np.float32)))
            for key in ("cvx", "cvy", "cwz", "vx", "vy", "wz"):
                sample = np.zeros((2, 2), dtype=np.float32)
                if key == "vx":
                    sample[1, 1] = 20.
                arrays.append((f"sampled.{key}", sample))
            rows = [{"t": t, "obstacle": (2., 0., 0.)} for t in (0., .1, .2, .3)]
            with patch.object(analyze, "read_cycle", return_value=(meta, arrays)), \
                 patch.object(analyze, "rows_from_transport", return_value=rows):
                summary, records = analyze.analyze(cycle, profile, truth)
            self.assertEqual(summary["safe_truth_and_costmap_rollouts"], 1)
            self.assertEqual(summary["predicted_collision_rollouts"], 1)
            self.assertEqual(summary["best_safe_total_rank"], 1)
            self.assertEqual(records[1]["first_predicted_conflict_s"], .2)
            self.assertLess(summary["prediction_score_max_abs_error"], .01)


if __name__ == "__main__":
    unittest.main()
