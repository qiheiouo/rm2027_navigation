"""Check the exact end-iterator condition used by the PathAlign audit."""
import unittest

import numpy as np

from guarded_path_align_probe import guarded_align_scores
from path_align_bound_audit import path_exposure


class PathAlignBoundAuditTest(unittest.TestCase):
    def test_integrated_distance_beyond_last_path_entry(self):
        path = [(float(x), 0.) for x in range(6)]
        trajectory = np.zeros((2, 9, 2), dtype=np.float32)
        trajectory[0, :, 0] = np.linspace(0., 5., 9)
        trajectory[1, :, 0] = np.linspace(0., 2., 9)
        furthest, limit, steps, first = path_exposure(path, trajectory, 4)
        self.assertEqual(furthest, 5)
        self.assertEqual(limit, 4.)
        self.assertEqual(steps, [4, 8])
        self.assertEqual(first.tolist(), [8, -1])

    def test_guarded_last_path_point_has_defined_score(self):
        path = [(float(x), 0.) for x in range(6)]
        trajectory = np.zeros((2, 9, 2), dtype=np.float32)
        trajectory[0, :, 0] = np.linspace(0., 5., 9)
        trajectory[1, :, 0] = np.linspace(0., 2., 9)
        raw = np.zeros((10, 10), dtype=np.uint8)
        map_meta = {"origin": (0., 0.), "resolution": 1.,
                    "width": 10, "height": 10}
        scores, first, furthest = guarded_align_scores(
            path, trajectory, 10., 4, raw, map_meta)
        self.assertEqual(furthest, 5)
        self.assertEqual(first.tolist(), [8, -1])
        np.testing.assert_allclose(scores, [7.5, 0.], rtol=0, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
