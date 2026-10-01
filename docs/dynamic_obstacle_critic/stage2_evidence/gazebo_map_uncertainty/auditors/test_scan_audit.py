#!/usr/bin/env python3
"""Independent analytic cases for offline range labels and source-time brackets."""
import math
import unittest
from audit_scan_geometry import interpolate, ray_box


class ScanAuditTest(unittest.TestCase):
    def test_axis_aligned_and_parallel_miss(self):
        self.assertAlmostEqual(ray_box((-3, 0, 0), 0, (0, 0, 0), (2, 4)), 2)
        self.assertIsNone(ray_box((-3, 3, 0), 0, (0, 0, 0), (2, 4)))
        self.assertIsNone(ray_box((-3, 0, 0), math.pi, (0, 0, 0), (2, 4)))

    def test_rotated_box(self):
        self.assertAlmostEqual(ray_box((0, -3, 0), math.pi / 2,
                                      (0, 0, math.pi / 2), (2, 4)), 2)

    def test_nearest_occluder(self):
        near = ray_box((0, 0, 0), 0, (2, 0, 0), (1, 1))
        far = ray_box((0, 0, 0), 0, (5, 0, 0), (2, 2))
        self.assertEqual(min(near, far), 1.5)

    def test_source_time_interpolation_across_yaw_wrap(self):
        p = interpolate([1, 1.02], [(0, 0, math.pi - .1),
                                    (2, 0, -math.pi + .1)], 1.01, .04)
        self.assertAlmostEqual(p[0], 1)
        self.assertAlmostEqual(abs(p[2]), math.pi)

    def test_missing_or_old_pose_never_uses_latest(self):
        times, poses = [1, 2], [(0, 0, 0), (10, 0, 0)]
        for stamp in (.9, 1.5, 2.1):
            self.assertIsNone(interpolate(times, poses, stamp, .04))
        self.assertEqual(interpolate(times, poses, 2, .04), (10, 0, 0))


if __name__ == '__main__':
    unittest.main()
