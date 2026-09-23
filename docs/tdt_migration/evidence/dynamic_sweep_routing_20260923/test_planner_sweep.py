import copy
import unittest

from planner_sweep import RESOLUTION_M, patch_profile, sweep_points


class PlannerSweepTest(unittest.TestCase):
    def test_lattice_covers_sweep_and_one_cell_raster_margin(self):
        points, source = sweep_points()
        x0, y0 = source["sweep_polygon_m"][0]
        x1, y1 = source["sweep_polygon_m"][2]
        self.assertLessEqual(min(p[0] for p in points), x0 - RESOLUTION_M)
        self.assertGreaterEqual(max(p[0] for p in points), x1 + RESOLUTION_M - 1e-12)
        self.assertLessEqual(min(p[1] for p in points), y0 - RESOLUTION_M)
        self.assertGreaterEqual(max(p[1] for p in points), y1 + RESOLUTION_M - 1e-12)
        self.assertEqual(len(points), source["point_count"])

    def test_only_two_costmaps_gain_new_layer(self):
        original = {
            "local_costmap": {"local_costmap": {"ros__parameters": {
                "resolution": RESOLUTION_M, "plugins": ["obstacle_layer", "inflation_layer"]}}},
            "global_costmap": {"global_costmap": {"ros__parameters": {
                "resolution": RESOLUTION_M, "plugins": ["obstacle_layer", "inflation_layer"]}}},
            "controller_server": {"ros__parameters": {"odom_topic": "/odometry/lio"}},
        }
        old = copy.deepcopy(original)
        changed = patch_profile(original)
        self.assertEqual(original, old)
        self.assertEqual(changed["controller_server"], old["controller_server"])
        for name in ("local_costmap", "global_costmap"):
            p = changed[name][name]["ros__parameters"]
            self.assertEqual(p["plugins"], ["obstacle_layer", "fixture_sweep_layer", "inflation_layer"])
            self.assertFalse(p["fixture_sweep_layer"]["fixture_sweep"]["clearing"])


if __name__ == "__main__":
    unittest.main()
