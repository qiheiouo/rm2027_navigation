"""Vector results checked against independent scalar audits and analytic contacts."""
import math
import numpy as np
from analyze_trial import distance,rotation
from audit_mechanical_footprint import circle_box_gap as scalar_circle
from audit_guard_rejections import current_map_clear
from witness_geometry import rotate,polygon_distance,circle_box_gap,interpolation_lower,raw_clearance_lower


def box(x,y):return np.array([[-x/2,-y/2],[x/2,-y/2],[x/2,y/2],[-x/2,y/2]])


def test_5000_polygon_and_circle_cases():
    rng=np.random.default_rng(61002);a=rng.uniform(-3,3,(5000,3));b=rng.uniform(-3,3,(5000,3))
    pa=rotate(a,box(.6,.5));pb=rotate(b,box(.45,.55));actual=polygon_distance(pa,pb)
    expected=np.array([distance(x,y) for x,y in zip(pa.tolist(),pb.tolist())])
    np.testing.assert_allclose(actual,expected,rtol=0,atol=2e-12)
    centers=rng.uniform(-3,3,(5000,2));circles=circle_box_gap(centers,.075,b,(.45,.55))
    reference=[scalar_circle(c,.075,p,(.45,.55)) for c,p in zip(centers,b)]
    np.testing.assert_allclose(circles,reference,rtol=0,atol=2e-12)


def test_analytic_contact_and_broadcast():
    a=box(1,1);b=box(1,1)
    shifts=np.array([[1,0,0],[1.25,0,0],[1.25,1.25,0],[0,0,0]])
    actual=polygon_distance(a,rotate(shifts,b))
    np.testing.assert_allclose(actual,[0,.25,math.sqrt(.125),0],rtol=0,atol=1e-15)
    np.testing.assert_allclose(circle_box_gap(np.array([[0,0],[.6,0]]),.1,np.zeros((2,3)),(1,1)),[0,0],atol=1e-15)


def test_raw_gate_matches_scalar_and_reserve_is_conservative():
    rng=np.random.default_rng(1927);fp=box(.6,.5);poses=rng.uniform([.7,.7,-3],[3.3,3.3,3],(160,3))
    grid=np.zeros((20,20),dtype=np.uint8)
    grid[3:7,5:8]=203;grid[10:13,12:14]=255;grid[15,3]=202
    record={'resolution':.2,'size':[20,20],'origin':[0.,0.],'data':grid.ravel().tolist()}
    lower=raw_clearance_lower(poses,fp,record,np.zeros(160))
    byte_record={**record,'data':grid.tobytes()}
    np.testing.assert_array_equal(lower,raw_clearance_lower(poses,fp,byte_record,np.zeros(160)))
    scalar={**record,'origin':[0,0,0,0,0,1]}
    assert (lower>1e-9).tolist()==[current_map_clear(scalar,p,fp,203) for p in poses]
    reserves=np.full(160,.08);reserved=raw_clearance_lower(poses,fp,record,reserves)
    ys,xs=np.nonzero(grid>=203)
    cells=[box(.2,.2)+np.array([x*.2+.1,y*.2+.1]) for y,x in zip(ys,xs)]
    for index,pose in enumerate(poses):
        poly=rotation(pose,fp)
        boundary=min(min(p[0] for p in poly),min(p[1] for p in poly),4-max(p[0] for p in poly),4-max(p[1] for p in poly))
        exact=min([max(0,boundary)]+[distance(poly,c.tolist()) for c in cells])
        assert reserved[index]<=exact+2e-12
    outside=raw_clearance_lower(np.array([[-1.,0,0]]),fp,record,np.zeros(1))
    assert outside[0]==0


def test_interval_lower_bound_dense_independent_samples():
    rng=np.random.default_rng(381);fp=box(.6,.5);actor=box(.45,.55)
    for _ in range(80):
        robot=rng.uniform(-2,2,(2,3));obstacle=rng.uniform(-2,2,(2,3))
        robot[1,2]=robot[0,2]+rng.uniform(-.5,.5);obstacle[1,2]=obstacle[0,2]+rng.uniform(-.5,.5)
        gaps=polygon_distance(rotate(robot,fp),rotate(obstacle,actor))
        bound=interpolation_lower(gaps,robot,np.linalg.norm(fp,axis=1).max(),obstacle,np.linalg.norm(actor,axis=1).max())
        dense=[]
        for alpha in np.linspace(0,1,101):
            r=robot[0]+alpha*(robot[1]-robot[0]);o=obstacle[0]+alpha*(obstacle[1]-obstacle[0])
            dense.append(distance(rotation(r,fp),rotation(o,actor)))
        assert bound<=min(dense)+2e-12
