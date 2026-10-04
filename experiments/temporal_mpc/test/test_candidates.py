from dataclasses import replace
import json
import numpy as np
import pytest
from temporal_mpc.candidates import CandidateMPC
from temporal_mpc.contracts import Snapshot, Track, Geometry, ContractError
from temporal_mpc.frontend import Window
from temporal_mpc.fixtures import reference
from temporal_mpc.diagnostics import input_identity


def fixture(monkeypatch):
    monkeypatch.setattr('time.perf_counter',lambda:0.)
    m=CandidateMPC();x=np.zeros(6)
    w=Window(0,reference(x,(8.5,0.,0.),m.times),(-.6,10.,-4.4,4.4),'one')
    r=m.engines[0].solve(x,0,Snapshot(0,()),w)
    assert r.model_feasible
    return m,x,w,r


def test_first_feasible_stops_and_preserves_limits(monkeypatch):
    m,x,w,_=fixture(monkeypatch)
    r=m.solve(x,50_000_000,Snapshot(50_000_000,()),replace(w,epoch_ns=50_000_000))
    assert r.model_feasible and r.fallback_id is None and r.command[0]>0.
    assert sum(t['attempted'] for t in r.candidate_trace)==1 and len(r.candidate_trace)==3
    assert r.iterations<=130 and r.solver_s<=.015
    assert np.max(np.abs(r.controls[:,:2]))<=1.
    assert np.max(np.abs(r.states[-1,3:]))<1e-5


@pytest.mark.parametrize('first_status',['uncertified_brake','previous_feasible'])
def test_another_side_can_replace_failure_or_checked_fallback(monkeypatch,first_status):
    m,x,w,r=fixture(monkeypatch);calls=[]
    def solve_first(*args):
        calls.append(0)
        return replace(r,status=first_status,model_feasible=first_status=='previous_feasible',
                       fallback_id='FollowPathMPPI',iterations=130,solver_s=.004)
    def solve_second(*args):
        calls.append(1);return replace(r,iterations=100,solver_s=.004)
    monkeypatch.setattr(m.engines[0],'solve',solve_first)
    monkeypatch.setattr(m.engines[1],'solve',solve_second)
    result=m.solve(x,0,Snapshot(0,()),w)
    assert calls==[0,1] and result.chosen_candidate=='right'
    assert result.fallback_id is None and result.iterations==230
    assert result.solver_s==.008 and not result.candidate_trace[2]['attempted']


def test_waiting_and_all_failure_keep_bounded_outputs(monkeypatch):
    m,x,w,r=fixture(monkeypatch)
    for engine in m.engines[:2]:
        monkeypatch.setattr(engine,'solve',lambda *a:replace(r,status='uncertified_brake',model_feasible=False,
                         fallback_id='FollowPathMPPI',iterations=130,solver_s=.004))
    result=m.solve(x,0,Snapshot(0,()),w)
    assert result.chosen_candidate=='wait' and result.model_feasible and result.fallback_id is None
    assert result.iterations<=390 and np.max(np.abs(result.command))<1e-5
    monkeypatch.setattr(m.engines[2],'solve',lambda *a:replace(r,status='uncertified_brake',model_feasible=False,
                        fallback_id='FollowPathMPPI',iterations=130,solver_s=.004))
    result=m.solve(x,0,Snapshot(0,()),w)
    assert not result.model_feasible and result.fallback_id=='FollowPathMPPI'
    assert result.iterations==390 and all(t['attempted'] for t in result.candidate_trace)


@pytest.mark.parametrize('deadline',['cycle','solver'])
def test_late_feasible_is_discarded_and_deceleration_is_bounded(monkeypatch,deadline):
    m,x,w,r=fixture(monkeypatch);clock=[0.];x[3]=.31
    monkeypatch.setattr('time.perf_counter',lambda:clock[0])
    def delayed(*args):
        clock[0]=.041 if deadline=='cycle' else 0.
        return replace(r,solver_s=.016 if deadline=='solver' else .004)
    monkeypatch.setattr(m.engines[0],'solve',delayed)
    result=m.solve(x,0,Snapshot(0,()),w)
    assert not result.model_feasible and result.fallback_id=='FollowPathMPPI'
    assert result.status=='uncertified_brake' and result.command[0]==pytest.approx(.26)
    assert len(result.candidate_trace)==3 and sum(t['attempted'] for t in result.candidate_trace)==1
    assert all(e.previous is None for e in m.engines)


@pytest.mark.parametrize('failure',['stale','yaw','epoch','collision','overload'])
def test_portfolio_cannot_bypass_input_or_all_track_hard_checks(monkeypatch,failure):
    m,x,w,r=fixture(monkeypatch);ns=50_000_000;s=Snapshot(ns,());w=replace(w,epoch_ns=ns)
    if failure=='stale':s=Snapshot(0,());ns=500_000_000;w=replace(w,epoch_ns=ns)
    if failure=='yaw':w=replace(w,reference=w.reference+np.array([0.,0.,.1]))
    if failure=='epoch':w=replace(w,epoch_ns=0)
    if failure in ('collision','overload'):
        count=1 if failure=='collision' else 5
        pos=(0.,0.) if failure=='collision' else (.8,.8)
        s=Snapshot(ns,tuple(Track(i,pos,(0.,0.),ns,'confirmed',Geometry('circle',radius=.2,source='test')) for i in range(count)))
    result=m.solve(x,ns,s,w)
    if failure=='overload':
        # The waiting reference can cull these distant tracks from the QP;
        # all five still participate in independent whole-trajectory checks.
        assert result.chosen_candidate=='wait' and result.model_feasible
    else:
        assert result.fallback_id=='FollowPathMPPI'
        if failure!='yaw':assert not result.model_feasible
    assert np.isfinite(result.command).all() and np.max(np.abs(result.acceleration[:2]))<=1.
    if failure=='yaw':assert not result.candidate_trace[-1]['attempted']


def test_input_identity_is_exact_serializable_and_does_not_mutate():
    s=Snapshot(1_100_000_000,(Track(8,(2.05,0.),(.5,0.),1_000_000_000,'coasting',
                               Geometry('circle',radius=1.7,source='test')),))
    x=np.zeros(6);w=Window(1_200_000_000,np.zeros((31,3)),(-.6,10.,-4.4,4.4),'one')
    d=input_identity(w.epoch_ns,s,x,w,2,100_000_000,True)
    assert d['input_source_ns']==s.source_ns and d['track_observation_ns']==[[8,1_000_000_000]]
    assert d['source_age_s']==.1 and s.tracks[0].position==(2.05,0.)
    json.dumps(d,allow_nan=False)
    x[0]=float('nan');d=input_identity(w.epoch_ns,s,x,w,2,None,False)
    assert d['initial_state'] is None and d['initial_state_invalid']
    json.dumps(d,allow_nan=False)
