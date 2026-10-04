from dataclasses import replace
from pathlib import Path
import subprocess
import numpy as np
import pytest
pytest.importorskip("osqp")
from temporal_mpc.contracts import ContractError, Snapshot, Track, Geometry
from temporal_mpc.frontend import Window, StaticRoute, prepare_route
from temporal_mpc.fixtures import reference, box
from temporal_mpc.realtime_qp import RealtimeMPC, QPConfig
from temporal_mpc.selection import Selection, MPPI, MPC


def window(m, x, epoch=0, plan="one"):
    return Window(epoch,reference(x,(3.,0.,x[2]),m.times),(-.8,4.,-.7,.7),plan)


def fail_solver(monkeypatch):
    import temporal_mpc.realtime_qp as module
    class Failure:
        def setup(self, **kw): raise RuntimeError("injected unavailable solver")
    monkeypatch.setattr(module.osqp,"OSQP",Failure)


def test_bounded_qp_omni_and_command():
    m=RealtimeMPC();x=np.zeros(6);w=window(m,x)
    w=replace(w,reference=w.reference+np.array([0.,.15,0.]))
    r=m.solve(x,0,Snapshot(0,()),w)
    assert r.model_feasible and r.status in ("optimized","feasible_iterate")
    assert r.command[0]>0 and r.command[1]>0 and r.command[2]==0
    np.testing.assert_allclose(r.command,x[3:]+.05*r.acceleration)
    assert np.max(np.abs(r.states[-1,3:]))<1e-5
    assert r.iterations<=400


def test_previous_feasible_reanchors_and_rechecks(monkeypatch):
    m=RealtimeMPC();x=np.zeros(6);r=m.solve(x,0,Snapshot(0,()),window(m,x))
    assert r.model_feasible
    x=r.states[1].copy();fail_solver(monkeypatch);epoch=50_000_000
    r=m.solve(x,epoch,Snapshot(epoch,()),window(m,x,epoch))
    assert r.status=="previous_feasible" and r.fallback_id==MPPI
    np.testing.assert_allclose(r.states[0],x)
    # A new obstacle invalidates old safety; stale inputs cannot reuse a plan.
    obstacle=Track(1,tuple(x[:2]),(0.,0.),100_000_000,"confirmed",Geometry("polygon",box(.2,.2),source="test"))
    x=r.states[1];r=m.solve(x,100_000_000,Snapshot(100_000_000,(obstacle,)),window(m,x,100_000_000))
    assert r.status=="uncertified_brake" and not r.model_feasible and m.previous is None


@pytest.mark.parametrize("failure",["stale","plan","wz","yaw","epoch"])
def test_reuse_rejected_on_invalid_contract(monkeypatch,failure):
    m=RealtimeMPC();x=np.zeros(6);r=m.solve(x,0,Snapshot(0,()),window(m,x));x=r.states[1].copy()
    fail_solver(monkeypatch);epoch=50_000_000;w=window(m,x,epoch);snap=Snapshot(epoch,())
    if failure=="stale": epoch=500_000_000;w=window(m,x,epoch);snap=Snapshot(0,())
    if failure=="plan": w=replace(w,plan_id="new")
    if failure=="wz": x[5]=.2
    if failure=="yaw": w=replace(w,reference=w.reference+np.array([0.,0.,.1]))
    if failure=="epoch": w=replace(w,epoch_ns=0)
    r=m.solve(x,epoch,snap,w)
    assert r.status!="previous_feasible" and r.fallback_id==MPPI
    assert np.isfinite(r.command).all()
    assert np.max(np.abs(r.acceleration[:2]))<=1 and abs(r.acceleration[2])<=2


def test_active_obstacle_overload_is_explicit():
    m=RealtimeMPC();x=np.zeros(6)
    tracks=tuple(Track(i,(.8,.8),(0.,0.),0,"confirmed",Geometry("circle",radius=.1,source="test")) for i in range(5))
    r=m.solve(x,0,Snapshot(0,tracks),window(m,x))
    assert r.reason=="active obstacle budget exceeded" and r.fallback_id==MPPI


def test_static_certificate_cannot_ignore_unknown():
    doc={"path":[[1.,1.],[2.,1.]],"anchors":[{"position":[1.,1.],"centre_bounds":[.8,1.5,.8,1.2]},
       {"position":[2.,1.],"centre_bounds":[1.5,2.5,.8,1.2]}]}
    grid=np.zeros((40,60),np.uint8);grid[20,20]=255
    with pytest.raises(ContractError,match="complete padded footprint"):
        StaticRoute(doc,grid,.05,(0.,0.))


def test_selector_both_directions_and_failure_latch():
    p=Selection();assert p.request(MPC,1.)==MPPI
    p.health(1.,True);assert p.request(MPC,1.01)==MPC
    assert p.request(MPPI,1.02)==MPPI
    assert p.request(MPC,1.03)==MPC
    assert p.health(1.04,True,True)==MPPI
    p.health(1.05,True);assert p.tick(1.06)==MPPI # never automatically bounce back
    assert p.request(MPC,1.06)==MPC
    assert p.tick(1.16)==MPPI
    with pytest.raises(ValueError):p.request("bad",1.2)


def test_solver_limits_cannot_remove_safety_geometry():
    for kw in [dict(max_iterations=501),dict(max_active_obstacles=5),dict(cycle_budget=.5),
               dict(physical_half_extents=(.2,.2)),dict(padding=0),dict(horizon=.5)]:
        with pytest.raises(ValueError):QPConfig(**kw)


def test_solver_timeout_keeps_a_bounded_output(monkeypatch):
    import temporal_mpc.realtime_qp as module
    from types import SimpleNamespace
    class Timeout:
        def setup(self, **kw):
            assert kw['time_limit']==.015 and kw['max_iter']==400
        def solve(self, **kw):
            return SimpleNamespace(x=None,info=SimpleNamespace(status='run time limit reached',iter=10))
    monkeypatch.setattr(module.osqp,'OSQP',Timeout)
    m=RealtimeMPC();x=np.array([0.,0.,0.,.031,0.,0.])
    r=m.solve(x,0,Snapshot(0,()),window(m,x))
    assert r.status=='brake' and r.model_feasible and r.fallback_id==MPPI
    assert r.command[0]==pytest.approx(0.)
    assert r.states[1,3]>=-1e-12 # no substep reversal during held control tick


def test_actual_tdt_frontend_routes_around_static_wall(tmp_path):
    root=Path(__file__).resolve().parents[1]
    binary=tmp_path/'frontend'
    subprocess.run(['bash',str(root/'frontend/build.sh'),str(binary)],check=True,capture_output=True)
    grid=np.zeros((80,160),np.uint8);grid[33:47,70:80]=254
    p=prepare_route(binary,grid,.05,(-1.,-2.),(0.,0.),(5.6,0.))
    assert len(p.path)>2 and np.max(np.abs(p.path[:,1]))>.8
    # Consume externally planned topology unchanged. A colliding supplied path
    # must be rejected rather than silently replaced with a new search route.
    external=prepare_route(binary,grid,.05,(-1.,-2.),p.path[0],p.path[-1],supplied_path=p.path)
    np.testing.assert_allclose(external.path,p.path,atol=1e-6)
    with pytest.raises(ContractError,match='frontend failed'):
        prepare_route(binary,grid,.05,(-1.,-2.),(0.,0.),(5.6,0.),supplied_path=[[0.,0.],[5.6,0.]])
    w=p.window(np.zeros(6),0,RealtimeMPC().times)
    assert w.plan_id==p.plan_id
    for bad in [np.full((8,8),300),np.full((8,8),-1),np.full((8,8),1.5)]:
        with pytest.raises(ContractError,match='no wrapping/truncation'):
            prepare_route(binary,bad,.05,(0.,0.),(1.,1.),(2.,1.))
    for wire in ['1000000 1000000 .05 0 0 0 0 1 1\n',
                 '3 3 .05 nan 0 0 0 1 1\n']:
        rejected=subprocess.run([str(binary)],input=wire,text=True,capture_output=True)
        assert rejected.returncode==2
    # Block both possible static routes: unknown must not become a passage.
    grid[:,70:80]=255
    with pytest.raises(ContractError,match='frontend failed'):
        prepare_route(binary,grid,.05,(-1.,-2.),(0.,0.),(5.6,0.))


def test_ros_jitter_warm_inputs_do_not_reuse_old_verdict():
    m=RealtimeMPC();x=np.zeros(6)
    r=m.solve(x,0,Snapshot(0,()),window(m,x))
    assert r.model_feasible
    shifted=m._shift_previous(63_000_000,'one')
    assert shifted.shape==(30,3)
    # Integral of each fractional old interval is preserved, including zero tail.
    expected=.037*r.controls[1]+.013*r.controls[2]
    np.testing.assert_allclose(shifted[0]*.05,expected,atol=1e-12)
    assert m._shift_previous(0,'one') is None
    assert m._shift_previous(151_000_000,'one') is None
    assert m._shift_previous(63_000_000,'changed') is None


def test_numeric_projection_preserves_limits_and_requires_geometry_recheck():
    m=RealtimeMPC();x=np.zeros(6)
    z=np.zeros(30);z[0]=1.000002;z[2]=-.99999
    repaired=m._repair_iterate(x,z)
    from temporal_mpc.dynamics import rollout
    states=rollout(x,repaired,.05)
    assert np.max(np.abs(repaired))<=1
    assert np.max(np.abs(states[-1,3:]))<1e-12
    # A bad obstacle trajectory cannot be rescued merely by actuator projection.
    obstacle=Track(1,(0.,0.),(0.,0.),0,'confirmed',Geometry('circle',radius=.2,source='test'))
    r=m.solve(x,0,Snapshot(0,(obstacle,)),window(m,x))
    assert not r.model_feasible and r.fallback_id==MPPI
