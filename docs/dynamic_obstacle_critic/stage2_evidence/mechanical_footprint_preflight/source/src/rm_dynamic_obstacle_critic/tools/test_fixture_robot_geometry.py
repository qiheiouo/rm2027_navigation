"""Independent checks for physical plant versus configured planning geometry."""
import math
from pathlib import Path
import tempfile
import unittest
import yaml
from fixture_robot_geometry import base_body_polygon, physical_base_body
from audit_mechanical_footprint import components


PACKAGE = Path(__file__).resolve().parents[1]
SCENE = PACKAGE.parent / 'rm_simulation' / 'worlds'


class FixtureGeometryTest(unittest.TestCase):
    def write_fixture(self, root, pose='0 0 0 0 0 0', size='2 4 1', frame=''):
        (root / 'phase1_omni.sdf').write_text(
            '<sdf><world><model name="rm_sentry_2027"><link name="base_link">'
            '<collision name="base_collision"><pose' + frame + '>' + pose + '</pose>'
            '<geometry><box><size>' + size + '</size></box></geometry>'
            '</collision></link></model></world></sdf>')

    def test_translated_rotated_physical_box(self):
        with tempfile.TemporaryDirectory() as path:
            root = Path(path)
            self.write_fixture(root, '3 5 0 0 0 ' + str(math.pi/2))
            actual = base_body_polygon(root)
            for p, expected in zip(actual, [(5,4),(1,4),(1,6),(5,6)]):
                for a, b in zip(p, expected): self.assertAlmostEqual(a,b)

    def test_unsupported_frames_nonplanar_and_invalid_sizes_rejected(self):
        with tempfile.TemporaryDirectory() as path:
            root = Path(path)
            for pose, size, frame in [('0 0 0 .1 0 0','2 4 1',''),
                    ('0 0 0 0 0 nan','2 4 1',''),('0 0 0 0 0 0','2 -4 1',''),
                    ('0 0 0 0 0 0','2 4 1',' relative_to="map"')]:
                self.write_fixture(root, pose, size, frame)
                with self.assertRaises(ValueError): base_body_polygon(root)

    def test_actual_base_does_not_expand_with_planning_envelope(self):
        configured = [(-.325,-.3),(.325,-.3),(.325,.3),(-.325,.3)]
        with tempfile.TemporaryDirectory() as path:
            root=Path(path); scene=root/'scene_inputs'; scene.mkdir()
            (scene/'phase1_omni.sdf').write_bytes((SCENE/'phase1_omni.sdf').read_bytes())
            actual=physical_base_body(root,configured)
            self.assertEqual(actual,[(-.3,-.25),(-.3,.25),(.3,.25),(.3,-.25)])
            self.assertNotEqual(actual,configured)
            (scene/'phase1_omni.sdf').unlink()
            self.assertEqual(physical_base_body(root,configured),configured)

    def test_complete_wheel_circles_fit_new_envelope_old_one_excludes_edges(self):
        shapes=components(SCENE)
        self.assertEqual(len(shapes),5)
        cfg=yaml.safe_load((PACKAGE/'config/nav2_cv_mechanical_footprint.yaml').read_text())
        cm=cfg['local_costmap']['local_costmap']['ros__parameters']
        fp=yaml.safe_load(cm['footprint']); hx=max(p[0] for p in fp);hy=max(p[1] for p in fp)
        circles=[s for s in shapes if s['kind']=='circle']
        self.assertEqual(len(circles),4)
        for s in circles:
            # Axis support of an entire circle, including its radius.
            self.assertLessEqual(abs(s['center'][0])+s['radius'],hx+1e-12)
            self.assertLessEqual(abs(s['center'][1])+s['radius'],hy+1e-12)
        self.assertTrue(any(abs(s['center'][1])+s['radius']>.28 for s in circles))
        padded={(math.copysign(abs(x)+cm['footprint_padding'],x),
                 math.copysign(abs(y)+cm['footprint_padding'],y)) for x,y in fp}
        guard=yaml.safe_load((PACKAGE/'config/guard_mechanical_footprint.yaml').read_text())
        flat=guard['dynamic_safety_guard']['ros__parameters']['footprint']
        guard_points=set(zip(flat[::2],flat[1::2]))
        self.assertEqual(len(padded),len(guard_points))
        self.assertLessEqual(max(min(math.dist(p,g) for g in guard_points) for p in padded),1e-12)

    def test_unknown_link_frame_and_sphere_radius_rejected(self):
        with tempfile.TemporaryDirectory() as path:
            root=Path(path)
            text=(SCENE/'phase1_omni.sdf').read_text()
            for bad in [text.replace('<pose>0.25 0.225', '<pose relative_to="map">0.25 0.225'),
                        text.replace('<radius>0.075</radius>','<radius>nan</radius>')]:
                (root/'phase1_omni.sdf').write_text(bad)
                with self.assertRaises(ValueError): components(root)


if __name__=='__main__': unittest.main()
