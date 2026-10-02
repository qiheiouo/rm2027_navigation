"""Independent rotated/irregular geometry and invalid-bound counterexamples."""
import copy
import math
from pathlib import Path
import random
import unittest
import yaml
from robot_extent_prior import class_diameters, anchor_envelope_radius, alternative_diameter

CONFIG = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/ground_robot_extent_prior_offline.yaml').read_text())


class ExtentPriorTest(unittest.TestCase):
    def test_rules_preserve_engineering_alternatives_and_unknown_scope(self):
        values = class_diameters(CONFIG)
        self.assertAlmostEqual(values['hero'], 1.2*math.sqrt(2))
        self.assertEqual(values['engineer'], values['hero'])
        self.assertAlmostEqual(values['infantry'], .8*math.sqrt(2))
        self.assertEqual(anchor_envelope_radius(CONFIG, 'unknown_within_listed_ground_classes', 0), max(values.values()))
        self.assertEqual(alternative_diameter({'sphere_diameter': 1.5}), 1.5)

    def test_half_diagonal_at_surface_anchor_misses_opposite_corner(self):
        anchor = (-.4, -.4); opposite = (.4, .4)
        half = .5*math.hypot(.8, .8)
        self.assertGreater(math.dist(anchor, opposite), half)
        self.assertLessEqual(math.dist(anchor, opposite), anchor_envelope_radius(CONFIG, 'infantry', 0))

    def test_irregular_rotated_sets_and_convex_hull_anchor_5000_cases(self):
        rng = random.Random(20261002)
        for _ in range(5000):
            length, width = rng.uniform(.01, 1.2), rng.uniform(.01, 1.2)
            # Independent irregular point set; an asymmetric convex combination
            # need not be any bounding-box center or even inside the object.
            points = [(rng.uniform(0, length), rng.uniform(0, width)) for _ in range(rng.randrange(3, 20))]
            coefficients = [rng.uniform(.001, 1) for _ in points]; denominator = sum(coefficients)
            q = [sum(p[i]*a for p, a in zip(points, coefficients))/denominator for i in (0, 1)]
            angle = rng.uniform(-math.pi, math.pi); c, s = math.cos(angle), math.sin(angle)
            offset = (rng.uniform(-20, 20), rng.uniform(-20, 20))
            def world(p): return offset[0]+c*p[0]-s*p[1], offset[1]+s*p[0]+c*p[1]
            world_q = world(q); error = rng.uniform(0, .2); direction = rng.uniform(-math.pi, math.pi)
            measured = [world_q[0]+error*math.cos(direction), world_q[1]+error*math.sin(direction)]
            radius = anchor_envelope_radius(CONFIG, 'unknown_within_listed_ground_classes', error)
            self.assertTrue(all(math.dist(measured, world(p)) <= radius+1e-13 for p in points))
            # Tighter independent scalar diameter inequality for this rectangle.
            self.assertTrue(all(math.dist(measured, world(p)) <= math.sqrt(length*length+width*width)+error+1e-13 for p in points))

    def test_nonzero_anchor_offset_cannot_be_silently_ignored(self):
        error = .15; anchor = (-.4-error, -.4); far = (.4, .4)
        true_distance = math.dist(anchor, far)
        self.assertGreater(true_distance, anchor_envelope_radius(CONFIG, 'sentry', 0))
        self.assertLessEqual(true_distance, anchor_envelope_radius(CONFIG, 'sentry', error))

    def test_reject_nonfinite_negative_and_boolean_bounds(self):
        for value in [float('nan'), float('inf'), -.001, True, None]:
            with self.assertRaises(ValueError): anchor_envelope_radius(CONFIG, 'hero', value)

    def test_out_of_scope_target_rejected(self):
        for name in ['airborne', 'field_equipment', 'robot', 'unknown']:
            with self.assertRaises(ValueError): anchor_envelope_radius(CONFIG, name, .1)

    def test_ambiguous_alternative_or_invalid_extent_rejected(self):
        for rule in [{'box_xy': [0, .8]}, {'box_xy': [1, 1, 1]}, {'box_xy': [1.7e308, 1.7e308]}, {'box_xy': [.8, .8], 'sphere_diameter': 1.5}, {'sphere_diameter': -1}, {'sphere_diameter': float('nan')}]:
            with self.assertRaises(ValueError): alternative_diameter(rule)

    def test_scope_must_match_its_rule_table(self):
        conf = copy.deepcopy(CONFIG); conf['scope']['target_classes'].remove('hero')
        with self.assertRaises(ValueError): class_diameters(conf)


if __name__ == '__main__': unittest.main()
