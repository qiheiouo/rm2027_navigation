import math
import unittest

from planner_sweep_v2 import RESOLUTION_M, axis_samples, sweep_points


class PlannerSweepV2Test(unittest.TestCase):
    def test_points_have_no_extra_margin_and_include_boundary(self):
        points, source = sweep_points()
        x0, y0 = source["sweep_polygon_m"][0]
        x1, y1 = source["sweep_polygon_m"][2]
        self.assertEqual((min(p[0] for p in points), max(p[0] for p in points)), (x0, x1))
        self.assertEqual((min(p[1] for p in points), max(p[1] for p in points)), (y0, y1))
        self.assertEqual(source["raster_margin_m"], 0.0)

    def test_every_positive_area_intersecting_cell_contains_lattice_point(self):
        points, source = sweep_points()
        x0, y0 = source["sweep_polygon_m"][0]
        x1, y1 = source["sweep_polygon_m"][2]
        xs = sorted({p[0] for p in points})
        ys = sorted({p[1] for p in points})
        for samples, low, high in ((xs, x0, x1), (ys, y0, y1)):
            for phase in range(50):
                origin = low - 0.2 + phase * 0.001
                for cell in range(120):
                    a = origin + cell * RESOLUTION_M
                    b = a + RESOLUTION_M
                    if max(a, low) < min(b, high) - 1e-12:
                        self.assertTrue(any(a - 1e-12 <= v < b + 1e-12 for v in samples))

    def test_nominal_goal_has_room_beyond_true_sweep_and_one_cell(self):
        _, source = sweep_points()
        x1 = source["sweep_polygon_m"][1][0]
        self.assertGreater(5.6 - (x1 + RESOLUTION_M), 0.4007889076915766)


if __name__ == "__main__":
    unittest.main()
