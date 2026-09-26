"""Independent analytic checks for the diagnostic probability transform."""
import unittest

from center_marginal_calibration import accelerated_cdf, polygon_cdf


class CenterMarginalCalibrationTest(unittest.TestCase):
    def test_rectangle_projection_cdf(self):
        # C ~ U[0, 2], V ~ U[0, 1], so C + V has a triangular edge.
        polygon = [(0., 0.), (2., 0.), (2., 1.), (0., 1.)]
        self.assertAlmostEqual(polygon_cdf(polygon, 1., .5), 1. / 16.)
        self.assertAlmostEqual(polygon_cdf(polygon, 1., 1.5), .5)

    def test_uniform_acceleration_convolution(self):
        # With horizon 1 and |a| <= 2, U = a/2 ~ U[-1, 1].
        # Integrating the triangular edge gives P(C + V + U <= 0) = 1/24.
        polygon = [(0., 0.), (2., 0.), (2., 1.), (0., 1.)]
        self.assertAlmostEqual(accelerated_cdf(polygon, 1., 0., 2.), 1. / 24.)
        self.assertAlmostEqual(accelerated_cdf(polygon, 1., 1.5, 2.), .5)


if __name__ == "__main__":
    unittest.main()
