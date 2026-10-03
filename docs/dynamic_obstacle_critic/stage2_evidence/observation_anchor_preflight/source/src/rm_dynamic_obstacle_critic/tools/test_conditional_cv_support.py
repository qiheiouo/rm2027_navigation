#!/usr/bin/env python3
"""Independent conditional-set geometry and source-age/unknown regression."""
from fractions import Fraction
import json
import math
from pathlib import Path
import unittest
import numpy as np
import yaml
from conditional_cv_support import conditional_radius, KEYS
from robot_extent_prior import class_diameters


def assumptions(diameter=2., error=.1, speed=1., validity=4.):
    return {'schema':1,'units':'m_s','time_reference':'message_source_epoch',
            'motion_semantics':'complete_projection_directed_Hausdorff_from_source',
            'bounds':dict(zip(KEYS,[diameter,error,speed,validity])),
            'condition_sources':dict.fromkeys(KEYS,'synthetic:test-only/no-physical-certificate')}


class ConditionalSupport(unittest.TestCase):
    def test_constructive_tight_bound_and_source_age(self):
        a=assumptions(.125,.25,.5,3.)
        row=conditional_radius(a,[-1.5,0],.5,2.5)
        # Source body endpoints 0,D; q=-e; true translation B*h and CV
        # translation in the opposite direction attain the bound geometrically.
        far_point=.125+.5*3
        cv_anchor=-.25-1.5*3
        self.assertEqual(row['radius_m'],far_point-cv_anchor)
        wrong_age=conditional_radius(a,[-1.5,0],0,2.5)['radius_m']
        self.assertGreater(far_point-cv_anchor,wrong_age)
        received=conditional_radius(a,[-1.5,0],.5,0)
        self.assertEqual(received['radius_m'],1.375)
        self.assertFalse(received['source_time_only'])

    def test_8000_irregular_rotating_and_deforming_sets(self):
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/ground_robot_extent_prior_offline.yaml').read_text())
        diameters=class_diameters(config)
        rng=np.random.default_rng(20261002)
        largest_gap=-math.inf
        counts=dict.fromkeys(diameters,0)
        for trial in range(8000):
            name=list(diameters)[trial%4];D=math.nextafter(diameters[name],math.inf)
            count=int(rng.integers(3,13));points=rng.normal(size=(count,2))
            width=float(np.max(np.linalg.norm(points[:,None,:]-points[None,:,:],axis=2)))
            points*=D*.95/width
            theta=float(rng.uniform(-math.pi,math.pi))
            rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
            points=points@rotation.T+rng.uniform(-20,20,size=2)
            weights=rng.uniform(.1,1,size=count);weights/=weights.sum()
            error=float(rng.uniform(0,.25));direction=rng.normal(size=2);direction/=np.linalg.norm(direction)
            anchor=weights@points+direction*error*.99
            B=float(rng.uniform(0,3));age=float(rng.uniform(0,.2));future=float(rng.uniform(0,3));h=age+future
            displacement=rng.normal(size=(count,2));displacement/=np.linalg.norm(displacement,axis=1)[:,None]
            displacement*=rng.uniform(0,.99,size=(count,1))*B*h
            truth=points+displacement  # Each future point has a source antecedent within B*h.
            velocity=rng.uniform(-3,3,size=2)
            row=conditional_radius(assumptions(D,error,B,4.),velocity.tolist(),age,future)
            self.assertEqual(row['status'],'CONDITIONAL_SUPPORT')
            distance=float(np.max(np.linalg.norm(truth-(anchor+velocity*h),axis=1)))
            largest_gap=max(largest_gap,distance-row['radius_m'])
            self.assertLessEqual(distance,row['radius_m']+2e-12)
            counts[name]+=1
        print(json.dumps({'seed':20261002,'independent_irregular_cases':8000,'class_counts':counts,
                          'maximum_point_distance_minus_conditional_radius_m':largest_gap,
                          'numerical_comparison_only':2e-12}),flush=True)

    def test_unknown_conditions_and_missing_provenance(self):
        for key in KEYS:
            for slot,reason in [('bounds','missing_bound:'),('condition_sources','missing_source:')]:
                a=assumptions();a[slot][key]=None
                row=conditional_radius(a,[.6,-.2],.1,3)
                self.assertEqual(row['status'],'UNKNOWN');self.assertIsNone(row['radius_m'])
                self.assertIn(reason+key,row['reasons'])

    def test_exact_source_time_requires_only_current_geometry(self):
        a=assumptions();a['bounds'][KEYS[2]]=None;a['bounds'][KEYS[3]]=None
        a['condition_sources'][KEYS[2]]=None;a['condition_sources'][KEYS[3]]=None
        row=conditional_radius(a,[1e308,1e308],0,0)
        self.assertEqual(row['status'],'CONDITIONAL_SUPPORT');self.assertTrue(row['source_time_only'])
        self.assertEqual(conditional_radius(a,[0,0],.01,0)['status'],'UNKNOWN')

    def test_source_validity_interval_and_exact_clock_addition(self):
        a=assumptions(validity=3)
        self.assertEqual(conditional_radius(a,[0,0],0,3)['status'],'CONDITIONAL_SUPPORT')
        self.assertIn('outside_source_validity_interval',conditional_radius(a,[0,0],.087,3)['reasons'])
        a['bounds'][KEYS[3]]=.3
        self.assertEqual(conditional_radius(a,[0,0],0,.3)['status'],'CONDITIONAL_SUPPORT')
        # Float 0.1+0.2 exceeds float 0.3 even in exact rational arithmetic.
        self.assertEqual(conditional_radius(a,[0,0],.1,.2)['status'],'UNKNOWN')

    def test_outward_rounding_against_exact_squared_norm(self):
        for velocity in [[.1,.2],[1e-300,1e-300],[3.,4.],[-1.23,2.34]]:
            a=assumptions(.17,.03,.31,4)
            age,future=.087,3.
            row=conditional_radius(a,velocity,age,future)
            h=Fraction(age)+Fraction(future)
            remainder=Fraction(row['radius_m'])-Fraction(.17)-Fraction(.03)-Fraction(.31)*h
            self.assertGreaterEqual(remainder,0)
            self.assertGreaterEqual(remainder**2,(Fraction(velocity[0])**2+Fraction(velocity[1])**2)*h**2)

    def test_nonfinite_overflow_and_malformed_conditions(self):
        for bad in [-1,math.inf,math.nan,True,'1',10**400]:
            a=assumptions();a['bounds'][KEYS[1]]=bad
            with self.assertRaises(ValueError):conditional_radius(a,[0,0],0,1)
        for velocity,age,future in [([math.nan,0],0,1),([0],0,1),([0,0],-1,1),([0,0],0,math.inf)]:
            with self.assertRaises(ValueError):conditional_radius(assumptions(),velocity,age,future)
        a=assumptions();a['motion_semantics']='chassis_translation_only'
        with self.assertRaises(ValueError):conditional_radius(a,[0,0],0,1)
        a=assumptions();a['condition_sources'][KEYS[2]]=' '
        with self.assertRaises(ValueError):conditional_radius(a,[0,0],0,1)
        a=assumptions();a['bounds'][KEYS[0]]=1e308;a['bounds'][KEYS[1]]=1e308
        self.assertIn('numeric_radius_overflow',conditional_radius(a,[0,0],0,0)['reasons'])
        self.assertIn('numeric_velocity_norm_overflow',conditional_radius(assumptions(),[1.7e308,1.7e308],0,1)['reasons'])

    def test_default_file_has_no_implicit_physical_bounds(self):
        path=Path(__file__).resolve().parents[1]/'config/cv_support_assumptions_offline.yaml'
        a=yaml.safe_load(path.read_text())
        self.assertTrue(all(v is None for v in a['bounds'].values()))
        for t in [0,1,2,3]:
            self.assertEqual(conditional_radius(a,[.593,0],.087,t)['status'],'UNKNOWN')


if __name__=='__main__':unittest.main(verbosity=2)
