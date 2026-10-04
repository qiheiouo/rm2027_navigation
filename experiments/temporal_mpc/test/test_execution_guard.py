from pathlib import Path
import json
from dataclasses import asdict
import math
import subprocess
import numpy as np
import pytest
from temporal_mpc.contracts import Snapshot, Track, Geometry
from temporal_mpc.dynamics import rollout_zoh, rollout
from temporal_mpc.execution_guard import ExecutionGuard, StaticCells
from temporal_mpc.realtime_qp import RealtimeMPC


def cells():
    return StaticCells(np.zeros((120,160),np.uint8),.05,(-1.,-3.))


def test_condensed_qp_matches_actual_held_commands_at_all_nodes():
    rng=np.random.default_rng(281)
    m=RealtimeMPC()
    for yaw in [0.,.6,math.pi/2]:
        x=np.array([1.,-.4,yaw,.1,-.2,0.])
        z=rng.uniform(-1,1,30)
        states=rollout_zoh(x,m._expand(z),.05)
        c,s=math.cos(yaw),math.sin(yaw); rot=np.array([[c,-s],[s,c]])
        expected=x[:2]+m.times[:,None]*(rot@x[3:5])+m.pmap@z.reshape(-1,2)@rot.T
        np.testing.assert_allclose(states[:,:2],expected,atol=1e-12)
        np.testing.assert_allclose(states[:,3:5],x[3:5]+m.vmap@z.reshape(-1,2),atol=1e-12)
    # Explicitly exposes the old ramp/ZOH displacement mismatch.
    a=np.tile([1.,0.,0.],(10,1));zero=np.zeros(6)
    assert rollout_zoh(zero,a,.05)[-1,0]-rollout(zero,a,.05)[-1,0]==pytest.approx(.0125)


def test_common_guard_command_slew_stop_and_invalid_input():
    g=ExecutionGuard();x=np.zeros(6)
    for k in range(10):
        previous=g.previous.copy()
        r=g.step(x,[.8,.5,0.],k*50_000_000,Snapshot(k*50_000_000,()),cells())
        assert r.model_certified and r.status=='pass'
        assert type(r.model_certified) is bool
        document=asdict(r);document['command']=r.command.tolist()
        json.dumps(document,allow_nan=False)
        assert np.max(np.abs(r.command-previous))<=.05+1e-12
        x=rollout_zoh(x,[(r.command-x[3:])/.05],.05)[1]
    for k in range(10):
        previous=g.previous.copy()
        r=g.step(x,[np.nan,0.,0.],(10+k)*50_000_000,None,cells())
        assert not r.model_certified and r.status=='uncertified_brake'
        assert np.isfinite(r.command).all()
        assert np.max(np.abs(r.command-previous))<=.05+1e-12
        x=rollout_zoh(x,[(r.command-x[3:])/.05],.05)[1]
    np.testing.assert_allclose(g.previous,0.,atol=1e-12)


def test_guard_rejects_future_collision_and_does_not_claim_uncertified_stop_safe():
    g=ExecutionGuard();g.previous=np.array([.4,0.,0.]);x=np.array([0.,0.,0.,.4,0.,0.])
    obstacle=Track(1,(.8,0.),(-.7,0.),0,'confirmed',Geometry('circle',radius=.1,source='test'))
    r=g.step(x,[.8,0.,0.],0,Snapshot(0,(obstacle,)),cells())
    assert r.status=='uncertified_brake' and not r.model_certified
    assert 'dynamic braking sweep' in r.reason and r.command[0]==pytest.approx(.35)
    # Current enclosing geometry remains conservative, including a track omitted
    # by any MPC active-set culling (guard independently checks all 64).
    tracks=tuple(Track(i,(0.,0.),(0.,0.),0,'confirmed',Geometry('circle',radius=.1,source='test')) for i in range(64))
    r=g.step(np.zeros(6),[0.,0.,0.],0,Snapshot(0,tracks),cells())
    assert not r.model_certified


def test_guard_unknown_static_cell_stale_state_and_prediction_deadline(monkeypatch):
    g=ExecutionGuard();grid=np.zeros((120,160),np.uint8);grid[60,20]=255
    r=g.step(np.zeros(6),[0.,0.,0.],0,Snapshot(0,()),StaticCells(grid,.05,(-1.,-3.)))
    assert not r.model_certified and 'static braking sweep' in r.reason
    r=g.step(np.zeros(6),[.8,0.,0.],500_000_000,Snapshot(0,()),cells())
    assert not r.model_certified
    r=g.step(np.zeros(6),[.8,0.,0.],0,Snapshot(0,()),cells(),.151)
    assert not r.model_certified
    import temporal_mpc.execution_guard as module
    times=iter([0.,.011]);monkeypatch.setattr(module.time,'perf_counter',lambda:next(times))
    r=g.step(np.zeros(6),[.8,0.,0.],0,Snapshot(0,()),cells())
    assert not r.model_certified and r.reason=='guard deadline'


def test_expanded_tdt_corridor_keeps_raw_static_certificate(tmp_path):
    from temporal_mpc.frontend import prepare_route
    root=Path(__file__).resolve().parents[1];binary=tmp_path/'front'
    subprocess.run(['bash',str(root/'frontend/build.sh'),str(binary)],check=True,capture_output=True)
    grid=np.zeros((120,160),np.uint8)
    route=prepare_route(binary,grid,.05,(-1.,-3.),(0.,0.),(5.6,0.))
    window=route.window(np.zeros(6),0,RealtimeMPC().times)
    assert window.centre_bounds[2]<-2.3 and window.centre_bounds[3]>2.3
    assert len(route.path)==2


def test_invalid_command_can_have_certified_brake_without_being_normal_pass():
    g=ExecutionGuard();g.previous=np.array([.4,0.,0.])
    r=g.step([0.,0.,0.,.4,0.,0.],[np.nan,0.,0.],0,Snapshot(0,()),cells())
    assert r.model_certified and r.status=='certified_brake'
    assert r.command[0]==pytest.approx(.35) and r.reason.startswith('invalid requested velocity')
