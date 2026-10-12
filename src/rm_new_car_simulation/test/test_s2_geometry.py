"""Independent enclosure and occupancy invariants for the S2 research domain."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('scene', ROOT/'scripts/baseline_scene.py')
scene = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scene)


class GeometryEvidence(unittest.TestCase):
    def test_continuous_attitude_and_joint_enclosure(self):
        cad, cfg = scene.load_config(ROOT)
        rng = np.random.default_rng(20261007)
        minimum = float('inf')
        for selection in ['A', 'B']:
            for profile in ['deformed', 'undeformed']:
                polygon = np.asarray(scene.navigation_footprint(cad, cfg, selection, profile))
                equations = ConvexHull(polygon).equations
                center, offset = scene.sensor_geometry(cad, cfg, selection)
                # Independent full box corners, actual arbitrary gimbal yaw, and
                # sphere support functions; do not reuse the sampled hull points.
                body = np.array([[*p, z] for p in cad['footprint_m'] for z in
                    [cfg['lower_collision_z_m'], cfg['upper_envelope_top_m'][profile]]])
                lidar = np.array([[x,y,z] for x in [-.04,.04] for y in [-.04,.04] for z in [-.03,.03]])
                for i in range(800):
                    roll, pitch = rng.uniform([-3,-18], [3,18])
                    if i < 4: roll,pitch = [(-3,-18),(-3,18),(3,-18),(3,18)][i]
                    rot = Rotation.from_euler('xyz', [roll,pitch,0], degrees=True)
                    yaw = Rotation.from_euler('z', rng.uniform(-np.pi,np.pi))
                    points = np.vstack([rot.apply(body),rot.apply(center+yaw.apply(offset+lidar))])
                    gaps = -(points[:,:2]@equations[:,:2].T+equations[:,2])
                    minimum = min(minimum, float(gaps.min()))
                    wheels = np.array([[cad['wheel']['wheelbase_m']/2*x,
                        cad['wheel']['track_m']/2*y,cad['wheel']['radius_m']+q]
                        for x,y in [(1,1),(1,-1),(-1,1),(-1,-1)] for q in [-.03,.03]])
                    wheel_gaps = -(rot.apply(wheels)[:,:2]@equations[:,:2].T+equations[:,2])-cad['wheel']['radius_m']
                    minimum = min(minimum,float(wheel_gaps.min()))
        self.assertGreaterEqual(minimum, cfg['navigation_safety_margin_m']-1e-8)
        print(json.dumps({'enclosure_samples':3200,'minimum_navigation_margin_m':minimum}))

    def test_tessellation_independent_occupancy_and_opening(self):
        _,cfg = scene.load_config(ROOT)
        # Same square surface split on opposite diagonals and into a 4x4 grid.
        def square(x0,y0,x1,y1,flip=False):
            p=np.array([[x0,y0,.2],[x1,y0,.2],[x1,y1,.2],[x0,y1,.2]])
            return p[[[0,1,3],[1,2,3]]] if flip else p[[[0,1,2],[0,2,3]]]
        a=square(-.25,-.25,.25,.25); b=square(-.25,-.25,.25,.25,True)
        grid=np.linspace(-.25,.25,5)
        c=np.concatenate([square(x,y,u,v,(i+j)%2) for i,(x,u) in enumerate(zip(grid,grid[1:]))
            for j,(y,v) in enumerate(zip(grid,grid[1:]))])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)
            images=[]
            for triangles in [a,b,c]:
                scene.export_maps(path,triangles,cfg,'test');images.append((path/'field.pgm').read_bytes())
            self.assertEqual(images[0],images[1]);self.assertEqual(images[0],images[2])
            # Independently known closed-cell rectangle intersection, including
            # a boundary exactly aligned with cell edges.
            image=np.frombuffer(images[0].split(b'\n',3)[3],np.uint8).reshape(340,600)[::-1]
            xs=-15+(np.arange(600)+.5)*.05;ys=-8.5+(np.arange(340)+.5)*.05
            expected=((abs(xs)[None,:]<=.275+1e-10)&(abs(ys)[:,None]<=.275+1e-10))
            np.testing.assert_array_equal(image==0,expected)
            ring=np.concatenate([square(-.5,-.5,.5,-.3),square(-.5,.3,.5,.5),
                square(-.5,-.3,-.3,.3),square(.3,-.3,.5,.3)])
            scene.export_maps(path,ring,cfg,'ring')
            image=np.frombuffer((path/'field.pgm').read_bytes().split(b'\n',3)[3],np.uint8).reshape(340,600)[::-1]
            self.assertEqual(image[170,300],254)
            self.assertEqual(image[170,308],0)
            # A vertical diagonal wall projects to a degenerate triangle.
            # Independent interval intersection along x=y gives the exact
            # closed-cell supercover, including corner-only intersections.
            wall=np.array([[[-.22,-.22,.1],[.22,.22,.1],[.22,.22,.3]]])
            scene.export_maps(path,wall,cfg,'wall')
            image=np.frombuffer((path/'field.pgm').read_bytes().split(b'\n',3)[3],np.uint8).reshape(340,600)[::-1]
            lo=np.maximum(np.maximum(xs[None,:]-.025,ys[:,None]-.025),-.22)
            hi=np.minimum(np.minimum(xs[None,:]+.025,ys[:,None]+.025),.22)
            np.testing.assert_array_equal(image==0,lo<=hi+1e-10)

    def test_rules_clearance_profiles(self):
        import xml.etree.ElementTree as ET
        for section,width,height in [('A',.7,.26),('B',.8,.25)]:
            for tolerance,scale in [('nominal',1.),('lower_clearance',.95)]:
                root=ET.fromstring(scene.fixture_scene(section=section,tolerance=tolerance))
                roof=root.find('.//collision[@name="tunnel_roof_collision"]')
                z=float(roof.findtext('pose').split()[2]);thickness=float(roof.findtext('geometry/box/size').split()[2])
                self.assertAlmostEqual(z-thickness/2,height*scale)
                sides=[root.find(f'.//collision[@name="tunnel_{s}_collision"]') for s in ['left','right']]
                positions=[float(s.findtext('pose').split()[1]) for s in sides]
                self.assertAlmostEqual(positions[1]-positions[0]-.1,width*scale)


if __name__ == '__main__':
    unittest.main()
