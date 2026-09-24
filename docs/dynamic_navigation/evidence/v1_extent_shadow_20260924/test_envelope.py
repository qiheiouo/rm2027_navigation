import math
import unittest
from envelope import predicted_box, fixture_target_acceleration


class EnvelopeTests(unittest.TestCase):
    def test_hidden_side_of_known_box_is_covered(self):
        # A tiny visible return lies at the bottom/right corner of a full box.
        box = predicted_box((.225, -.275), (0, 0), (0, 0), (.45, .55), 0, 0, 0)
        self.assertTrue(box.contains((0, 0), (.45, .55)))
        # The half-extent interpretation would miss the opposite corner.
        self.assertLessEqual(box.min_x, -.225)
        self.assertGreaterEqual(box.max_y, .275)

    def test_age_and_future_step_share_one_source_clock(self):
        box = predicted_box((1, 2), (0, 1), (.1, .1), (.45, .55), .15, .4, 0)
        self.assertAlmostEqual((box.min_y + box.max_y) / 2, 2.55)

    def test_rejects_nonfinite_or_negative_extent(self):
        with self.assertRaises(ValueError):
            predicted_box((math.nan, 0), (0, 0), (0, 0), (.45, .55), 0, .2, 0)
        with self.assertRaises(ValueError):
            predicted_box((0, 0), (0, 0), (0, 0), (-.45, .55), 0, .2, 0)

    def test_reference_acceleration_is_target_only(self):
        self.assertAlmostEqual(fixture_target_acceleration(), .5551652475612764)


if __name__ == '__main__':
    unittest.main()
