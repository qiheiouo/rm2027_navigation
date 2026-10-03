"""Profile/goal gate regressions independent of Gazebo and ROS processes."""
import copy
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import yaml
from observation_profile_preflight import (NOMINAL_DIAMETER, SCHEMA, registered_profiles,
    receipt_evidence, typed_value)
from dynamic_radius_io import profile_radius, consumed_radius

CONFIG = Path(__file__).resolve().parents[1]/'config'
def load(name): return yaml.safe_load((CONFIG/name).read_text())


class Selection(unittest.TestCase):
    def test_auditors_use_full_nominal_diameter_and_keep_old_visible_radius(self):
        visible=dict(minimum_obstacle_radius=.36)
        self.assertEqual(profile_radius(visible,[.2,.2]),.36)
        nominal=load('guard_observation_nominal.yaml')['dynamic_safety_guard']['ros__parameters']
        self.assertEqual(profile_radius(nominal,[.2,.2]),NOMINAL_DIAMETER)
        limits=dict(minimum_radius=.36,**{key:nominal[key] for key in
                    ('input_schema','obstacle_radius_mode','nominal_obstacle_diameter','geometry_prior_source')})
        self.assertEqual(consumed_radius(limits,[.2,.2],2),NOMINAL_DIAMETER)
        limits['input_schema']='rm_dynamic_obstacle_predictions/v1'
        with self.assertRaises(ValueError):consumed_radius(limits,[.2,.2],2)

    def test_nominal_float_covers_exact_manual_upright_square_diagonal(self):
        squared = Fraction.from_float(NOMINAL_DIAMETER)**2
        self.assertGreaterEqual(squared, 2*Fraction(6,5)**2)

    def test_two_complete_profiles_and_cross_component_tampering(self):
        baselines = dict(controller=load('nav2_cv_cost_accumulator_evidence.yaml'),
                         tracker=load('tracker_cv.yaml'), guard=load('guard_mechanical_footprint.yaml'))
        for label, mode in [('visible','visible_extent'),('nominal','nominal_diameter')]:
            profiles = dict(controller=load('nav2_cv_observation_'+label+'.yaml'),
                            tracker=load('tracker_observation_cv.yaml'), guard=load('guard_observation_'+label+'.yaml'))
            registered_profiles(**profiles, baselines=baselines, mode=mode)
            probes = [
                ('controller', lambda p:p['controller_server']['ros__parameters']['FollowPath'].update(batch_size=301)),
                ('controller', lambda p:p['controller_server']['ros__parameters']['FollowPath']['DynamicObstacleCritic'].pop('geometry_prior_source')),
                ('controller', lambda p:p['controller_server']['ros__parameters'].update(controller_frequency=10)),
                ('tracker', lambda p:p['dynamic_obstacle_tracker_shadow']['ros__parameters']['prediction'].update(max_speed=3.0)),
                ('tracker', lambda p:p['dynamic_obstacle_tracker_shadow']['ros__parameters']['tracker'].update(association_gate=.7)),
                ('guard', lambda p:p['dynamic_safety_guard']['ros__parameters'].update(nominal_obstacle_diameter=.8)),
                ('guard', lambda p:p['dynamic_safety_guard']['ros__parameters'].update(horizon=.6))]
            for role, mutate in probes:
                changed=copy.deepcopy(profiles);mutate(changed[role])
                with self.assertRaises(ValueError): registered_profiles(**changed, baselines=baselines, mode=mode)

    def test_parameter_types_preserve_bool_int_double_and_array(self):
        self.assertEqual([typed_value(v)[0] for v in [True,1,1.,'1',[.1,.2],['a']]], [1,2,3,4,8,9])
        for value in [float('nan'),[1,2],[True],None]:
            with self.assertRaises(ValueError): typed_value(value)

    def test_goal_receipt_requires_selected_schema_freshness_grid_and_completeness(self):
        message = S(header=S(stamp=S(sec=1700000000,nanosec=123456789),frame_id='map'), schema=SCHEMA,
                    authority='shadow_only', AUTHORITY_SHADOW_ONLY='shadow_only', complete=True,
                    prediction_steps=30,prediction_dt=.1,total_track_count=0,tracks=[])
        now=1700000000123456789
        self.assertEqual(receipt_evidence(message,now)['source_stamp_ns'],now)
        for field, value in [('schema','rm_dynamic_obstacle_predictions/v1'),('authority','control'),
                             ('complete',False),('prediction_steps',29),('prediction_dt',.2),('total_track_count',1)]:
            changed=copy.deepcopy(message);setattr(changed,field,value)
            with self.assertRaises(RuntimeError): receipt_evidence(changed,now)
        for delta in [400000001,-50000001]:
            with self.assertRaises(RuntimeError): receipt_evidence(message,now+delta)


if __name__=='__main__': unittest.main()
