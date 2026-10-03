#!/usr/bin/env python3
"""Observe missing coast duration with an independently constructed tight set."""
from copy import deepcopy
from fractions import Fraction
import math
import unittest

from conditional_observation_support import conditional_observation_radius
from conditional_cv_support import KEYS


def assumptions():
    return dict(schema=1, units='m_s', time_reference='last_observation_epoch',
        motion_semantics='complete_projection_directed_Hausdorff_from_observation',
        bounds=dict(zip(KEYS, [.125, .25, .5, 3.])),
        condition_sources=dict.fromkeys(KEYS, 'synthetic:test-only'))


class ObservationSupport(unittest.TestCase):
    def test_tight_support_needs_coast_and_source_age(self):
        document = assumptions()
        row = conditional_observation_radius(document, [-1.5, 0], .5, .25, 2.25)
        far_point = .125 + .5*3
        public_anchor = -.25 - 1.5*.5
        future_anchor = public_anchor - 1.5*(.25+2.25)
        self.assertEqual(row['radius_m'], far_point-future_anchor)
        self.assertGreater(far_point-future_anchor,
            conditional_observation_radius(document, [-1.5, 0], 0., .25, 2.25)['radius_m'])

    def test_validity_starts_at_last_observation_including_now(self):
        document = assumptions()
        self.assertEqual(conditional_observation_radius(document, [0, 0], 0, 0, 3)['status'],
                         'CONDITIONAL_SUPPORT')
        self.assertIn('outside_observation_validity_interval',
            conditional_observation_radius(document, [0, 0], .1, 0, 3)['reasons'])
        document['bounds']['valid_for_sec'] = .1
        self.assertEqual(conditional_observation_radius(document, [0, 0], .2, 0, 0)['status'],
                         'UNKNOWN')

    def test_unknown_geometry_motion_and_provenance(self):
        for key in KEYS:
            for slot in ('bounds', 'condition_sources'):
                document = assumptions(); document[slot][key] = None
                row = conditional_observation_radius(document, [0, 0], .1, 0, 0)
                self.assertEqual(row['status'], 'UNKNOWN')
                self.assertIsNone(row['radius_m'])

    def test_current_observation_only_needs_geometry_but_coasting_needs_motion(self):
        document = assumptions()
        for key in KEYS[2:]:
            document['bounds'][key] = document['condition_sources'][key] = None
        now = conditional_observation_radius(document, [1e308, 1e308], 0, 0, 0)
        self.assertEqual(now['radius_m'], .375)
        self.assertTrue(now['observation_time_only'])
        self.assertEqual(conditional_observation_radius(document, [0, 0], .01, 0, 0)['status'],
                         'UNKNOWN')

    def test_distinct_epoch_contract_and_invalid_time(self):
        document = assumptions()
        for field, value in [('time_reference', 'message_source_epoch'),
                             ('motion_semantics', 'chassis_translation_only')]:
            bad = deepcopy(document); bad[field] = value
            with self.assertRaises(ValueError):
                conditional_observation_radius(bad, [0, 0], 0, 0, 0)
        for elapsed in (-.1, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                conditional_observation_radius(document, [0, 0], elapsed, 0, 0)

    def test_three_duration_terms_keep_exact_validity_and_outward_radius(self):
        document = assumptions(); document['bounds']['valid_for_sec'] = .3
        self.assertEqual(conditional_observation_radius(document, [0, 0], .1, .1, .1)['status'],
                         'UNKNOWN')  # exact float sum is greater than float .3
        document['bounds']['valid_for_sec'] = 4.
        for velocity in ([.1, .2], [1e-300, 1e-300], [3., 4.], [-1.23, 2.34]):
            row = conditional_observation_radius(document, velocity, .35, .087, 3.)
            elapsed = Fraction(.35)+Fraction(.087)+Fraction(3.)
            available = (Fraction(row['radius_m'])-Fraction(.125)-Fraction(.25))/elapsed-Fraction(.5)
            self.assertGreaterEqual(available, 0)
            self.assertGreaterEqual(available**2, sum(Fraction(v)**2 for v in velocity))


if __name__ == '__main__':
    unittest.main()
