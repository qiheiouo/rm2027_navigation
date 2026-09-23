import math
import unittest
from viability_core import NEXT_DECISION_S, NEXT_RESPONSE_S, scale_command, viability

BODY=[(-.28,-.19),(.28,-.19),(.28,.19),(-.28,.19)]
SWEEP=[(4.675,-1.225),(5.125,-1.225),(5.125,1.225),(4.675,1.225)]

class ViabilityTest(unittest.TestCase):
    def test_uses_existing_watchdog_and_max_age(self):
        self.assertEqual(NEXT_DECISION_S,.12)
        self.assertAlmostEqual(NEXT_RESPONSE_S,.30)

    def test_far_command_admitted(self):
        scale,chosen,_,reason=scale_command((4.9,-2.3,0),(0,0,0),(0,.5,0),BODY,SWEEP,.02)
        self.assertEqual((scale,reason),(1,'admit'))
        self.assertTrue(chosen['safe_under_model'])

    def test_near_command_scaled_before_zero_loses_certificate(self):
        scale,chosen,full,reason=scale_command((4.9,-1.7,0),(0,0,0),(0,.5,0),BODY,SWEEP,.02)
        self.assertEqual(reason,'scale_viability')
        self.assertTrue(0<scale<1)
        self.assertTrue(chosen['safe_under_model'])
        self.assertFalse(full['safe_under_model'])
        self.assertGreaterEqual(chosen['next_body_lower_m'],.05)

    def test_unstoppable_zero_fails_closed(self):
        scale,chosen,_,reason=scale_command((4.9,-1.45,0),(0,0,0),(0,.5,0),BODY,SWEEP,.02)
        self.assertEqual((scale,reason),(0,'reject_next_unstoppable'))
        self.assertFalse(chosen['safe_under_model'])

    def test_rejects_invalid_age(self):
        with self.assertRaises(ValueError):
            viability((0,0,0),(0,0,0),(0,0,0),BODY,SWEEP,math.nan)
        with self.assertRaises(ValueError):
            viability((0,0,0),(0,0,0),(0,0,0),BODY,SWEEP,.101)

if __name__=='__main__':
    unittest.main()
