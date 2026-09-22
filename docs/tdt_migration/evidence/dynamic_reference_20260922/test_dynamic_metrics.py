import math
import unittest
from dynamic_metrics import compose, stamp, geometry_metrics, polygon_distance, placed

BOX=[(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)]
class MovingGeometry(unittest.TestCase):
    def test_model_link_composition(self):
        p=compose((4.9,1.,math.pi/2),(0.,.5,math.pi/2))
        self.assertAlmostEqual(p[0],4.4);self.assertAlmostEqual(p[1],1.)
        self.assertAlmostEqual(abs(p[2]),math.pi)
    def test_protobuf_int64_and_default_stamp(self):
        self.assertAlmostEqual(stamp({'header':{'stamp':{'sec':'9','nsec':500000000}}}),9.5)
        self.assertEqual(stamp({'header':{'stamp':{}}}),0.)
    def test_two_body_motion_cannot_be_ignored(self):
        # Equal safe endpoint distances hide an obstacle passing through the robot.
        rows=[{'t':0.,'robot':(10.,0.,0.),'obstacle':(9.5,0.,0.)},
              {'t':1.,'robot':(10.,0.,0.),'obstacle':(10.5,0.,0.)}]
        m=geometry_metrics(rows,BOX,0.,BOX)
        self.assertAlmostEqual(m['body']['moving_sample_min_m'],.3)
        self.assertLess(m['body']['moving_interpolation_bound_m'],0.)
        self.assertFalse(m['body_clearance_at_least_005m'])
    def test_translating_together_bound_and_padding(self):
        rows=[{'t':0.,'robot':(10.,0.,0.),'obstacle':(10.3,0.,0.)},
              {'t':1.,'robot':(10.01,0.,0.),'obstacle':(10.31,0.,0.)}]
        m=geometry_metrics(rows,BOX,.03,BOX)
        self.assertAlmostEqual(m['body']['moving_interpolation_bound_m'],.09)
        self.assertAlmostEqual(m['padded']['moving_interpolation_bound_m'],.06)
        self.assertTrue(m['body_clearance_at_least_005m'])
    def test_rotated_contact(self):
        self.assertEqual(polygon_distance(placed(BOX,(0.,0.,0.)),placed(BOX,(.1,0.,math.pi/4))),0.)
if __name__=='__main__':unittest.main()
