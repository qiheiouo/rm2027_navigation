import unittest

from sweep_shadow import fixture_sweep, shadow_case


BODY = [[-0.28, -0.19], [0.28, -0.19], [0.28, 0.19], [-0.28, 0.19]]


class FixtureSweepShadowTest(unittest.TestCase):
    def test_sweep_comes_from_joint_limits_and_box(self):
        sweep, source = fixture_sweep()
        self.assertEqual(source["slider_limit_m"], [-0.95, 0.95])
        self.assertEqual(source["box_size_m"][:2], [0.45, 0.55])
        self.assertEqual(sweep, [[4.675000000000001, -1.225],
                                 [5.125, -1.225], [5.125, 1.225],
                                 [4.675000000000001, 1.225]])

    def test_stopped_inside_full_sweep_is_rejected(self):
        sweep, _ = fixture_sweep()
        cycle = {"pose": [4.9, -1.25, 0.0], "odom_speed": [0.0, 0.0, 0.0]}
        self.assertFalse(shadow_case(cycle, BODY, sweep, 0.0)["assumed_braking_gate_pass"])

    def test_stopped_outside_full_sweep_is_accepted_under_model(self):
        sweep, _ = fixture_sweep()
        cycle = {"pose": [4.9, -2.0, 0.0], "odom_speed": [0.0, 0.0, 0.0]}
        result = shadow_case(cycle, BODY, sweep, 0.0)
        self.assertTrue(result["assumed_braking_gate_pass"])
        self.assertGreater(result["body"]["continuous_bound_m"], 0.05)

    def test_finite_braking_distance_changes_feasibility(self):
        sweep, _ = fixture_sweep()
        cycle = {"pose": [4.9, -1.65, 0.0], "odom_speed": [0.0, 0.5, 0.0]}
        self.assertFalse(shadow_case(cycle, BODY, sweep, 0.2)["assumed_braking_gate_pass"])
        cycle["odom_speed"] = [0.0, 0.0, 0.0]
        self.assertTrue(shadow_case(cycle, BODY, sweep, 0.2)["assumed_braking_gate_pass"])


if __name__ == "__main__":
    unittest.main()
