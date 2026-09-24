import unittest
from probe import static_map
from rm_dynamic_obstacle_tracking.core import Point2D


class FixtureMapTest(unittest.TestCase):
    def test_static_subtraction_keeps_moving_box_out_of_map(self):
        grid,rectangles=static_map()
        self.assertEqual(len(rectangles),3)
        self.assertEqual(grid.value_at_world(Point2D(1.4,0)),100)
        self.assertEqual(grid.value_at_world(Point2D(3.0,.525)),100)
        self.assertEqual(grid.value_at_world(Point2D(3.0,-.525)),100)
        self.assertEqual(grid.value_at_world(Point2D(4.9,0)),0)

if __name__=='__main__':unittest.main()
