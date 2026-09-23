import math
import unittest

from guard_core import certificate


BODY = [(-0.28, -0.19), (0.28, -0.19), (0.28, 0.19), (-0.28, 0.19)]
SWEEP = [(4.675, -1.225), (5.125, -1.225),
         (5.125, 1.225), (4.675, 1.225)]


class GuardCoreTest(unittest.TestCase):
    def test_distant_pose_has_stop_reserve(self):
        result = certificate((0, 0, 0), (0.4, 0, 0), (0.4, 0, 0), BODY, SWEEP)
        self.assertTrue(result["safe_under_model"])

    def test_next_command_uses_up_stop_reserve(self):
        pose = (4.9, -1.65, 0)
        stopped = certificate(pose, (0, 0, 0), (0, 0, 0), BODY, SWEEP)
        moving = certificate(pose, (0, 0, 0), (0, 0.5, 0), BODY, SWEEP)
        self.assertTrue(stopped["safe_under_model"])
        self.assertFalse(moving["safe_under_model"])

    def test_odom_speed_matters_even_when_command_is_zero(self):
        pose = (4.9, -1.65, 0)
        result = certificate(pose, (0, 0.5, 0), (0, 0, 0), BODY, SWEEP)
        self.assertFalse(result["safe_under_model"])

    def test_nonfinite_input_fails_closed(self):
        with self.assertRaises(ValueError):
            certificate((0, 0, 0), (0, 0, 0), (math.nan, 0, 0), BODY, SWEEP)


if __name__ == "__main__":
    unittest.main()
