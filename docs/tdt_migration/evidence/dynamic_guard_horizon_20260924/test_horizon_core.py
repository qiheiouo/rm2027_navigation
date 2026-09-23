import math
import unittest

from horizon_core import response_horizon, scale_command

BODY = [(-0.28, -0.19), (0.28, -0.19), (0.28, 0.19), (-0.28, 0.19)]
SWEEP = [(4.675, -1.225), (5.125, -1.225),
         (5.125, 1.225), (4.675, 1.225)]


class HorizonGuardTest(unittest.TestCase):
    def test_next_cycle_and_observation_age_are_explicit(self):
        self.assertAlmostEqual(response_horizon(0.02, 20), 0.22)

    def test_distant_command_passes(self):
        scale, selected, _, reason, hold = scale_command(
            (0, 0, 0), (0, 0, 0), (0.5, 0, 0), BODY, SWEEP, 0.02, 20)
        self.assertEqual((scale, reason), (1, "admit"))
        self.assertTrue(selected["safe_under_model"])
        self.assertAlmostEqual(hold, 0.22)

    def test_near_command_is_scaled_under_unchanged_clearance_gate(self):
        scale, selected, full, reason, _ = scale_command(
            (4.9, -1.7, 0), (0, 0, 0), (0, 0.5, 0), BODY, SWEEP, 0.02, 20)
        self.assertEqual(reason, "scale_sweep")
        self.assertTrue(0 < scale < 1)
        self.assertTrue(selected["safe_under_model"])
        self.assertFalse(full["safe_under_model"])

    def test_unstoppable_zero_fails_closed(self):
        scale, selected, _, reason, _ = scale_command(
            (4.9, -1.55, 0), (0, 0.5, 0), (0, 0.5, 0),
            BODY, SWEEP, 0.02, 20)
        self.assertEqual((scale, reason), (0, "reject_unstoppable"))
        self.assertFalse(selected["safe_under_model"])

    def test_stale_or_nonfinite_timing_rejected(self):
        for age in (0.101, math.nan):
            with self.assertRaises(ValueError):
                response_horizon(age, 20)


if __name__ == "__main__":
    unittest.main()
