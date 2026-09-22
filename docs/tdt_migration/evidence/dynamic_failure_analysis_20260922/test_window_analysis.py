import math
import unittest
from analyze_window import ray_box, interpolate, scan_review, map_review

class WindowTests(unittest.TestCase):
    def test_ray_parallel_hit_and_miss(self):
        self.assertEqual(ray_box((0,0),(1,0),(2,-1,3,1)),2)
        self.assertIsNone(ray_box((0,2),(1,0),(2,-1,3,1)))
        self.assertIsNone(ray_box((0,0),(-1,0),(2,-1,3,1)))

    def test_ray_corner_and_inside(self):
        self.assertAlmostEqual(ray_box((0,0),(2**-.5,2**-.5),(1,1,2,2)),2**.5)
        self.assertEqual(ray_box((1.5,1.5),(1,0),(1,1,2,2)),0)

    def test_interpolation_wrap_and_coverage(self):
        rows=[{'t':0,'robot':(0,0,math.radians(179)),'obstacle':(2,0,0)},
              {'t':2,'robot':(2,0,math.radians(-179)),'obstacle':(2,2,0)}]
        r=interpolate(rows,1)
        self.assertAlmostEqual(r['robot'][2],math.pi)
        self.assertEqual(r['obstacle'],[2,1,0])
        with self.assertRaises(ValueError):interpolate(rows,3)

    def test_nearer_scan_return_is_not_box_detection(self):
        rows=[{'t':i,'robot':(0,0,0),'obstacle':(2,0,0)} for i in (0,1)]
        scan={'t':.5,'frame':'sim_lidar_link','angle_min':0,'angle_increment':.01,
              'range_min':.1,'range_max':8,'ranges':[1.]}
        r=scan_review(scan,rows,.04)
        self.assertEqual((r['matching_rays'],r['nearer_returns']),(0,1))
        scan['ranges']=[1.775]
        self.assertEqual(scan_review(scan,rows,.04)['matching_rays'],1)

    def test_inscribed_is_not_lethal(self):
        state={'t':1,'robot':(0,0,0),'obstacle':(1.5,0,0)}
        polygon=[(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]
        grid={'t':.9,'resolution':1,'width':1,'origin':[1,-.5],'data':[99],'frame':'odom'}
        self.assertIsNone(map_review(grid,state,polygon)['body_to_published_lethal_cells_m'])
        self.assertEqual(map_review(grid,state,polygon)['box_centre_counts']['99_inscribed'],1)
        grid['data']=[100]
        self.assertAlmostEqual(map_review(grid,state,polygon)['body_to_published_lethal_cells_m'],.5)
        self.assertEqual(map_review(grid,state,polygon)['box_centre_counts']['100_lethal'],1)

if __name__=='__main__':unittest.main()
