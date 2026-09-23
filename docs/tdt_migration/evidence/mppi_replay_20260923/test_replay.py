import math
import unittest
import numpy as np
from replay import closed_cells, first_step_from_twist, min_raw_gap, replay_controls

class ReplayTests(unittest.TestCase):
    def test_shifted_first_command_is_index_one(self):
        path = replay_controls((0, 0, 0), np.array([9., 1., 2.]),
                               np.zeros(3), np.zeros(3), .1)
        self.assertAlmostEqual(path[0][0], .1)
        self.assertAlmostEqual(path[1][0], .3)
        self.assertAlmostEqual(path[2][0], .5)

    def test_body_twist_rotates_into_world(self):
        x, y, yaw = first_step_from_twist((1, 2, math.pi / 2), (.5, .1, 1.), .1)
        self.assertAlmostEqual(x, .99)
        self.assertAlmostEqual(y, 2.05)
        self.assertAlmostEqual(yaw, math.pi / 2 + .1)

    def test_closed_cell_geometry_and_contact(self):
        cells = closed_cells(np.array([[0, 254], [255, 253]], dtype=np.uint8),
                             {'origin': [0, 0], 'resolution': .1})
        self.assertEqual(len(cells), 2)
        self.assertIn((.1, 0., .2, .1), cells)
        body = [(-.02, -.02), (.02, -.02), (.02, .02), (-.02, .02)]
        self.assertAlmostEqual(min_raw_gap(body, [(0, 0, 0)], cells), .08)
        self.assertEqual(min_raw_gap(body, [(.1, 0, 0)], cells), 0.)

if __name__ == '__main__': unittest.main()
