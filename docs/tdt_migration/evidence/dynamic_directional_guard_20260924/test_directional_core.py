import math
import unittest

from directional_core import certificate

BODY = [(-0.28, -0.19), (0.28, -0.19), (0.28, 0.19), (-0.28, 0.19)]
SWEEP = [(4.675, -1.225), (5.125, -1.225),
         (5.125, 1.225), (4.675, 1.225)]


class DirectionalCertificateTest(unittest.TestCase):
    def test_static_position_outside_sweep_passes(self):
        self.assertTrue(certificate((0, 0, 0), (0, 0, 0), (0, 0, 0), BODY, SWEEP)["safe_under_model"])

    def test_command_into_sweep_fails(self):
        result = certificate((4.9, -1.55, 0), (0, 0, 0), (0, 0.5, 0), BODY, SWEEP)
        self.assertFalse(result["safe_under_model"])

    def test_existing_motion_blocks_zero_command_if_stop_is_too_late(self):
        result = certificate((4.9, -1.55, 0), (0, 0.5, 0), (0, 0, 0), BODY, SWEEP)
        self.assertFalse(result["safe_under_model"])

    def test_direction_away_from_sweep_can_pass(self):
        result = certificate((5.5, -1.7, 0), (0, 0, 0), (0.5, 0, 0), BODY, SWEEP)
        self.assertTrue(result["safe_under_model"])

    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):
            certificate((0, 0, 0), (0, 0, 0), (math.nan, 0, 0), BODY, SWEEP)


if __name__ == "__main__":
    unittest.main()
