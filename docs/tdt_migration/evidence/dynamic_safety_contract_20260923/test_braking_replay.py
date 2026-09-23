import math
import unittest

from braking_replay import braking_poses, slowing, obstacle_speed_bound, check_brake


class BrakeModelTests(unittest.TestCase):
    def test_component_reaches_zero_after_delay(self):
        self.assertAlmostEqual(slowing(0.5, 0.3, 0.1, 1.0), 0.3)
        self.assertEqual(slowing(-0.5, 0.7, 0.1, 1.0), 0.0)

    def test_constant_delay_then_bounded_stop(self):
        p = braking_poses((0, 0, 0), (0.5, 0, 0), 0.1, horizon=2, step=0.01)
        self.assertAlmostEqual(p[10][0], 0.05, places=6)
        self.assertAlmostEqual(p[-1][0], 0.175, places=4)
        self.assertEqual(p[-1], p[-2])

    def test_obstacle_speed_uses_actual_pose_intervals(self):
        rows = [{"t": 0, "obstacle": (0, 0, 0)},
                {"t": 0.1, "obstacle": (0.05, 0, 0)},
                {"t": 0.2, "obstacle": (0.15, 0, 0)}]
        self.assertAlmostEqual(obstacle_speed_bound(rows, [0, .1, .2], .05, .15, .4), 1.0)
        with self.assertRaises(ValueError):
            obstacle_speed_bound(rows, [0, .1, .2], .05, .25, .4)

    def test_yaw_is_integrated_and_stops(self):
        p = braking_poses((0, 0, 0), (0, 0, 0.4), 0, horizon=1, step=.01)
        self.assertAlmostEqual(p[-1][2], .04, places=4)
        self.assertEqual(math.dist(p[-1][:2], (0, 0)), 0)


    def test_future_sweep_hits_stationary_robot(self):
        body = [(-.1, -.1), (.1, -.1), (.1, .1), (-.1, .1)]
        box = body
        rows = [{"t": 0.0, "obstacle": (1.0, 0.0, 0.0)},
                {"t": 2.0, "obstacle": (-1.0, 0.0, 0.0)}]
        result = check_brake(rows, [0.0, 2.0], 0.0, (0, 0, 0),
                             (0, 0, 0), body, box, 0.0)
        self.assertFalse(result["assumed_braking_gate_pass"])
        self.assertEqual(result["body"]["sample_min_m"], 0.0)

    def test_response_delay_can_close_braking_window(self):
        body = [(-.1, -.1), (.1, -.1), (.1, .1), (-.1, .1)]
        box = body
        rows = [{"t": 0.0, "obstacle": (.43, 0.0, 0.0)},
                {"t": 2.0, "obstacle": (.43, 0.0, 0.0)}]
        now = check_brake(rows, [0.0, 2.0], 0.0, (0, 0, 0),
                          (.5, 0, 0), body, box, 0.0)
        late = check_brake(rows, [0.0, 2.0], 0.0, (0, 0, 0),
                           (.5, 0, 0), body, box, 0.2)
        self.assertTrue(now["assumed_braking_gate_pass"])
        self.assertFalse(late["assumed_braking_gate_pass"])


if __name__ == "__main__":
    unittest.main()
