import numpy as np
import pytest
from temporal_mpc.contracts import Snapshot, Track, Geometry, predict, NOMINAL_DIAMETER
from temporal_mpc.frontend import Window
from temporal_mpc.local_reference import LateralReference
from temporal_mpc.realtime_qp import RealtimeMPC
from temporal_mpc.fixtures import reference
from temporal_mpc.dynamics import rollout_zoh


def fixture(m,x,ns=0,center=(3.,0.),radius=.2,bounds=(-.6,7.,-2.4,2.4),plan='one'):
    route_state=x.copy();route_state[1]=0.
    w=Window(ns,reference(route_state,(5.6,0.,x[2]),m.times),bounds,plan)
    s=Snapshot(ns,(Track(1,center,(0.,0.),ns,'confirmed',Geometry('circle',radius=radius,source='test')),))
    return w,s


def test_early_side_reference_is_bounded_and_latches_without_oracle():
    m=RealtimeMPC();x=np.zeros(6);w,s=fixture(m,x,radius=NOMINAL_DIAMETER,center=(4.7,0.))
    local=LateralReference();r=local.apply(x,w,predict(s,0,m.times))
    assert local.mode=='shift' and local.side==1
    assert np.max(r[:,0])==0. and np.min(r[:,1])>2.3
    x[1]=-.01
    w,s=fixture(m,x,50_000_000,radius=NOMINAL_DIAMETER,center=(4.7,.01))
    r=local.apply(x,w,predict(s,50_000_000,m.times))
    assert local.side==1 and np.min(r[:,1])>2.3
    # A new planner route resets the preference, not a feasible verdict.
    x[1]=-.1;w,s=fixture(m,x,100_000_000,radius=NOMINAL_DIAMETER,plan='new')
    local.apply(x,w,predict(s,100_000_000,m.times))
    assert local.side==-1


def test_insufficient_corridor_keeps_original_reference_and_hard_collision_veto():
    m=RealtimeMPC();x=np.zeros(6);w,s=fixture(m,x,bounds=(-.6,7.,-.5,.5),radius=1.)
    local=LateralReference();r=local.apply(x,w,predict(s,0,m.times))
    assert local.mode=='corridor_insufficient'
    np.testing.assert_allclose(r,w.reference[:,:2])
    w,s=fixture(m,x,center=(0.,0.))
    result=m.solve(x,0,s,w)
    assert not result.model_feasible and result.fallback_id=='FollowPathMPPI'


@pytest.mark.parametrize('yaw',[0.,.4])
def test_closed_loop_detour_and_return_keep_hard_checks(monkeypatch,yaw):
    # Timing is exercised separately in ROS; this fixture tests geometry and
    # progress under the explicitly held-target kinematic model.
    monkeypatch.setattr('temporal_mpc.realtime_qp.time.perf_counter',lambda:0.)
    m=RealtimeMPC();x=np.array([0.,0.,yaw,0.,0.,0.]);history=[]
    for k in range(500):
        ns=k*50_000_000;w,s=fixture(m,x,ns,center=(2.5,0.),radius=.3)
        result=m.solve(x,ns,s,w)
        assert result.model_feasible, (k,result.reason,result.constraint_min)
        assert result.constraint_min>=-1e-6 and np.max(np.abs(result.states[-1,3:]))<1e-5
        x=rollout_zoh(x,[(result.command-x[3:])/.05],.05)[1]
        history.append(x.copy())
        if np.linalg.norm(x[:2]-[5.6,0.])<.15:break
    history=np.array(history)
    assert np.max(history[:,1])>.8 and x[0]>5.45 and abs(x[1])<.15
    assert m.lateral.mode=='route'
