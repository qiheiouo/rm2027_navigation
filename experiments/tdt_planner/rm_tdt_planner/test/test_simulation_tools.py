#!/usr/bin/env python3
"""Synthetic fixtures for handoff tooling; not Gazebo or physical acceptance."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0, str(TOOLS))
from audit_endpoint_witness import audit_message, audit_file
from simulation_geometry import clearance, point_segment, polygon_distance
from inspect_simulation_endpoints import endpoint
from simulation_evidence import analyze
from summarize_simulation import summarize


class WitnessAuditTest(unittest.TestCase):
    def setUp(self):
        self.payload = dict(origin=[-2.,-4.], size=[200,160], resolution=.05,
                            radius=.43, clearance=.02, guard=1e-7,
                            start=dict(status='free', local=[7.3,3.], collision=None),
                            goal=dict(status='blocked', local=[6.3,4.], collision=dict(
                                cell=[120,87], cost=254, boundary=False,
                                distance=math.hypot(.25,.35), required=.43+.02)))
        self.context = (' [input=nav2_master start=(5.3,-1) goal=(4.3,0) origin=(-2,-4) '
                        'size=200x160 resolution=0.05 radius=0.43 clearance=0.02 '
                        'start_yaw=0 goal_yaw=0]')

    def message(self):
        return ('start or goal outside conservative free space endpoint_witness_v1=' +
                json.dumps(self.payload) + self.context)

    def test_exact_square_and_raw_cost_not_endpoint_cost(self):
        result = audit_message(self.message())
        self.assertAlmostEqual(result['goal']['shortfall_m'], .45-math.hypot(.25,.35))
        self.assertEqual(result['goal']['cost'],254)
        self.assertFalse(result['start']['free_claim_independently_verified'])
        self.assertAlmostEqual(result['goal']['world_box'][0],4.)

    def test_corrupt_evidence_is_rejected(self):
        original = copy.deepcopy(self.payload)
        for key, value in [('cost', 252), ('distance', 0.), ('distance', float('nan')),
                           ('required', .1), ('boundary', True), ('cell', [200,87])]:
            self.payload = copy.deepcopy(original)
            self.payload['goal']['collision'][key] = value
            with self.assertRaises(ValueError): audit_message(self.message())
        self.payload = original
        with self.assertRaises(ValueError):
            audit_message(self.message().replace('goal=(4.3,0)','goal=(4.4,0)'))

    def test_centre_only_and_outside_reasons(self):
        self.payload['goal']['local'] = [6.3,4.0]
        self.payload['goal']['collision'] = dict(cell=[126,80],cost=253,boundary=False,
                                               distance=0.,required=0.)
        audit_message(self.message())
        self.payload['goal'] = dict(status='outside_or_nonfinite',local=[-1.,4.],collision=None)
        audit_message(self.message().replace('goal=(4.3,0)','goal=(-3,0)'))

    def test_missing_witnesses_never_claim_an_audit(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'events.jsonl'
            p.write_text('')
            self.assertFalse(audit_file(p)['audited'])
            p.write_text(json.dumps(dict(node='planner_server',t=1.,
                message='start or goal outside conservative free space'))+'\n')
            self.assertEqual(audit_file(p)['missing_witnesses'],1)
            self.assertFalse(audit_file(p)['audited'])


class EndpointInspectionTest(unittest.TestCase):
    def test_point_distance_distinguishes_cell_overconservatism_and_real_collision(self):
        grid = dict(width=200, height=160, resolution=.05, origin=[-2.0,-4.0], data=[0]*32000)
        grid['data'][87*200+119] = 100
        radius = math.hypot(.33,.28)
        point = endpoint(grid,4.3,0,radius,.02)
        self.assertFalse(point['nav2_master_seed_model_free'])
        self.assertTrue(point['point_model_free'])
        self.assertAlmostEqual(point['point_to_hard_or_border_square_m'],math.hypot(.3,.35))
        grid['data'][87*200+120] = -1
        point = endpoint(grid,4.3,0,radius,.02)
        self.assertFalse(point['point_model_free'])
        self.assertAlmostEqual(point['point_to_hard_or_border_square_m'],math.hypot(.25,.35))
        grid['data'] = [0]*32000
        grid['data'][80*200+126] = 99
        self.assertFalse(endpoint(grid,4.3,0,radius,.02)['point_model_free'])


class GeometryTest(unittest.TestCase):
    def test_segment_projection_and_zero_length(self):
        self.assertEqual(point_segment((1,1),(0,0),(2,0)), 1)
        self.assertEqual(point_segment((3,0),(0,0),(2,0)), 1)
        self.assertAlmostEqual(point_segment((1,1),(0,0),(0,0)), math.sqrt(2))

    def test_containment_contact_and_separation(self):
        a = [(0,0),(2,0),(2,2),(0,2)]
        self.assertEqual(polygon_distance(a,[(.5,.5),(1,.5),(1,1),(.5,1)]), 0)
        self.assertEqual(polygon_distance(a,[(2,0),(3,0),(3,1),(2,1)]), 0)
        self.assertEqual(polygon_distance(a,[(3,0),(4,0),(4,1),(3,1)]), 1)

    def test_fixture_padding_and_rotation(self):
        self.assertAlmostEqual(clearance(0,0,0), .925)
        self.assertAlmostEqual(clearance(0,0,0,.03), .895)
        self.assertAlmostEqual(clearance(0,0,math.pi/2), .975)
        self.assertEqual(clearance(1.4,0,0), 0)

    def test_invalid_pose_is_not_free(self):
        with self.assertRaises(ValueError): clearance(float('nan'),0,0)
        with self.assertRaises(ValueError): clearance(0,0,0,-.1)


class EvidenceTest(unittest.TestCase):
    def setUp(self):
        self.samples = [dict(t=t,x=4.3,y=0,yaw=0,body_clearance_m=.15,padded_clearance_m=.12,
                             cross_track_m=.01,vx=0,vy=0,wz=0) for t in (1.0,1.04,1.08)]
        self.commands = [dict(t=1.08,vx=0,vy=0,wz=0)]

    def report(self, **overrides):
        args = dict(samples=self.samples,commands=self.commands,events=[],goal_x=4.3,goal_y=0,
                    status=4,timed_out=False,recoveries=0)
        args.update(overrides)
        return analyze(**args)

    def test_complete_static_fixture(self):
        self.assertTrue(self.report()['static_geometry_and_goal_pass'])

    def test_action_success_does_not_override_clearance(self):
        self.samples[1]['body_clearance_m'] = 0
        self.assertFalse(self.report()['static_geometry_and_goal_pass'])

    def test_rotation_interval_bound(self):
        self.samples[1]['yaw'] = math.pi
        self.assertLess(self.report()['linear_pose_interpolation_body_bound_m'], 0)

    def test_insufficient_or_nonmonotonic_evidence(self):
        with self.assertRaises(ValueError): self.report(samples=[])
        self.samples[1]['t'] = 1.0
        with self.assertRaises(ValueError): self.report()

    def test_missing_plan_or_large_gap_is_incomplete(self):
        for sample in self.samples: sample['cross_track_m'] = None
        self.assertFalse(self.report()['evidence_valid'])
        for sample in self.samples: sample['cross_track_m'] = .01
        self.samples[-1]['t'] = 1.5
        self.assertFalse(self.report()['evidence_valid'])

    def test_stop_timeout_and_recovery(self):
        self.assertFalse(self.report(timed_out=True)['static_geometry_and_goal_pass'])
        self.assertFalse(self.report(recoveries=1)['static_geometry_and_goal_pass'])
        self.commands[-1]['vx'] = .2
        self.assertFalse(self.report()['static_geometry_and_goal_pass'])

    def test_resource_warning_never_rejects_algorithm(self):
        report = self.report(events=[{'message':'Control loop missed desired rate'}])
        self.assertTrue(report['static_geometry_and_goal_pass'])
        self.assertFalse(report['algorithm_rejected'])
        self.assertEqual(report['warning_counts']['Control loop missed'],1)

    def test_empty_matrix_is_not_a_pass(self):
        with tempfile.TemporaryDirectory() as root:
            report = summarize(Path(root))
            self.assertFalse(report['all_recorded_static_checks_pass'])
            self.assertEqual(len(report['missing_trials']),20)


class ProfileTest(unittest.TestCase):
    def test_only_planner_changes_and_mutation_is_rejected(self):
        import yaml
        root = TOOLS.parents[3]
        baseline = root/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml'
        original = yaml.safe_load(baseline.read_text())
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'profiles'
            cmd = [sys.executable,str(TOOLS/'make_sim_profiles.py'),str(baseline),str(output)]
            subprocess.run(cmd,check=True,capture_output=True)
            subprocess.run(cmd+['--verify'],check=True,capture_output=True)
            for file in output.glob('*.yaml'):
                data = yaml.safe_load(file.read_text())
                data['planner_server']['ros__parameters']['GridBased'] = copy.deepcopy(original['planner_server']['ros__parameters']['GridBased'])
                self.assertEqual(data,original)
            self.assertNotEqual(subprocess.run(cmd,capture_output=True).returncode,0)
            file = output/'tdt_qp.yaml'
            file.write_text(file.read_text().replace('controller_frequency: 10.0','controller_frequency: 9.0'))
            self.assertNotEqual(subprocess.run(cmd+['--verify'],capture_output=True).returncode,0)


if __name__ == '__main__':
    unittest.main()
