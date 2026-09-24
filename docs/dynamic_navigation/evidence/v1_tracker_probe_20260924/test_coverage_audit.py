import unittest
from coverage_audit import deficits, summarize


class CoverageAuditTests(unittest.TestCase):
    def test_exact_visible_box_contains_truth(self):
        row = {'estimated_xy': [1.0, 2.0], 'estimated_visible_cluster_size_xy': [0.4, 0.6], 'actual_obstacle_xy': [1.0, 2.0]}
        self.assertEqual(summarize([row], [0.4, 0.6])['fully_contained'], 1)
        self.assertEqual(deficits(row, [0.4, 0.6])['y']['positive_m'], 0)

    def test_visible_surface_loses_hidden_side(self):
        row = {'estimated_xy': [1.0, 1.8], 'estimated_visible_cluster_size_xy': [0.4, 0.2], 'actual_obstacle_xy': [1.0, 2.0]}
        self.assertAlmostEqual(deficits(row, [0.4, 0.6])['y']['positive_m'], 0.4)
        self.assertEqual(summarize([row], [0.4, 0.6])['fully_contained'], 0)


if __name__ == '__main__':
    unittest.main()
