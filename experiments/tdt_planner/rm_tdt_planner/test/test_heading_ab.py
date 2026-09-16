import sys
from pathlib import Path
import math
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from heading_ab import rates
class AngularMetrics(unittest.TestCase):
    def test_time_weighting_and_zero_dt_accounting(self):
        r=rates([{'t':0.,'wz':0.},{'t':1.,'wz':2.},{'t':1.,'wz':0.},{'t':3.,'wz':0.}])
        self.assertAlmostEqual(r['time_weighted_rms_wz_rad_s'], math.sqrt(2/3))
        self.assertEqual(r['max_abs_dwz_dt_rad_s2'],2)
        self.assertEqual(r['total_variation_wz_rad_s'],4)
        self.assertEqual(r['zero_dt_intervals_excluded_from_derivative'],1)
    def test_invalid_timing_rejected(self):
        for rows in ([],[{'t':2.,'wz':0.},{'t':1.,'wz':0.}],
                     [{'t':0.,'wz':float('nan')},{'t':1.,'wz':0.}]):
            with self.assertRaises(ValueError): rates(rows)
if __name__=='__main__': unittest.main(verbosity=2)
