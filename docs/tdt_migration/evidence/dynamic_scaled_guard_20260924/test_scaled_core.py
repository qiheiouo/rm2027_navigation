import math
import unittest

from scaled_core import scale_command

BODY = [(-0.28, -0.19), (0.28, -0.19), (0.28, 0.19), (-0.28, 0.19)]
SWEEP = [(4.675, -1.225), (5.125, -1.225),
         (5.125, 1.225), (4.675, 1.225)]


class ScaledGuardTest(unittest.TestCase):
    def test_unrestricted_command_is_unchanged(self):
        scale, chosen, full, reason = scale_command((0, 0, 0), (0, 0, 0),
                                                    (0.5, 0, 0), BODY, SWEEP)
        self.assertEqual((scale, reason), (1, "admit"))
        self.assertEqual(chosen, full)

    def test_fast_command_is_scaled_to_safe_boundary(self):
        pose = (4.9, -1.65, 0)
        scale, chosen, full, reason = scale_command(pose, (0, 0, 0),
                                                    (0, 0.5, 0), BODY, SWEEP)
        self.assertEqual(reason, "scale_sweep")
        self.assertGreater(scale, 0)
        self.assertLess(scale, 1)
        self.assertFalse(full["safe_under_model"])
        self.assertTrue(chosen["safe_under_model"])

    def test_existing_speed_can_make_zero_the_only_response(self):
        scale, chosen, full, reason = scale_command((4.9, -1.55, 0),
                                                    (0, 0.5, 0), (0, 0.5, 0),
                                                    BODY, SWEEP)
        self.assertEqual((scale, reason), (0, "reject_unstoppable"))
        self.assertFalse(chosen["safe_under_model"])

    def test_nonfinite_input_fails_closed(self):
        with self.assertRaises(ValueError):
            scale_command((0, 0, 0), (0, 0, 0), (math.nan, 0, 0), BODY, SWEEP)


if __name__ == "__main__":
    unittest.main()
