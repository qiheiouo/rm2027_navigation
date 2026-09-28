"""Check that the single-sample oracle uses Nav2's aggregate filter path."""
import unittest

import numpy as np

from native_critic_sensitivity import aggregate
from oracle_aggregation_probe import one_hot


class OracleAggregationProbeTest(unittest.TestCase):
    def test_one_hot_matches_aggregate_with_unit_probability(self):
        rng = np.random.default_rng(23)
        controls = rng.normal(0, .3, (3, 30, 3)).astype(np.float32)
        initial = rng.normal(0, .2, (30, 3)).astype(np.float32)
        history = rng.normal(0, .1, (4, 3)).astype(np.float32)
        settings = {"steps": 30, "offset": 1, "gamma": .015,
                    "temperature": .3, "vx_std": .2, "vy_std": .2,
                    "wz_std": .2, "vx_min": -.5, "vx_max": .8,
                    "vy_max": .5, "wz_max": 1.2}
        weights = np.array([0., 1., 0.], dtype=np.float32)
        expected = aggregate(np.array([3., 2., 1.], dtype=np.float32),
                             controls, initial, settings, history,
                             weights)["filtered_sequence"]
        np.testing.assert_allclose(one_hot(controls, 1, settings, history),
                                   expected, rtol=0, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
