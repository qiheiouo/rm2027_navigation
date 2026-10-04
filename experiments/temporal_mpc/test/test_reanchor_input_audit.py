from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from audit_reanchor_inputs import factorial, dynamic_margins
from temporal_mpc.contracts import Snapshot, Track, Geometry


def fixture():
    old=Snapshot(1_180_000_000,(Track(1,(.8,0.),(0.,0.),1_180_000_000,'confirmed',Geometry('circle',radius=1.7,source='test')),))
    new=Snapshot(1_190_000_000,(Track(1,(.8,.2),(0.,0.),1_190_000_000,'confirmed',Geometry('circle',radius=1.7,source='test')),))
    h=dict(evaluation_ns=1_200_000_000,proposal_ns=1_180_000_000,generation=2,initial_state=[0.,2.18,0.,0.,0.,0.],
           reanchor_projection='velocity_and_stop/v2',constraint='dynamic_clearance',step=0,track_id=1,slack=0.)
    h['slack']=float(dynamic_margins(np.tile(h['initial_state'],(31,1)),new,h['evaluation_ns'])[0][0,0])
    p=dict(accelerations=[dict(x=0.,y=0.,z=0.) for _ in range(30)])
    w=dict(epoch_ns=h['proposal_ns'],generation=2,input_source_ns=old.source_ns,input_validated=True,initial_state=[0.,2.2,0.,0.,0.,0.])
    return h,p,w,old,new


def test_factorial_separates_recorded_state_and_prediction_substitutions():
    h,p,w,old,new=fixture();r=factorial(h,p,w,old,new);c=r['cases']
    assert c['native_state__native_prediction']['at_recorded_track_step_m']==pytest.approx(h['slack'])
    assert c['native_state__worker_prediction']['at_recorded_track_step_m']>0.
    assert c['worker_state__native_prediction']['at_recorded_track_step_m']<0.
    assert c['worker_state__worker_prediction']['at_recorded_track_step_m']>0.
    assert r['measured_state_delta'][1]==pytest.approx(-.02)
    with pytest.raises(ValueError,match='identity'):
        factorial(h,p,dict(w,input_source_ns=new.source_ns),old,new)


def test_old_prediction_cannot_be_forged_fresh_in_counterfactual():
    h,p,w,old,new=fixture();h=dict(h,evaluation_ns=1_600_000_000)
    new=Snapshot(1_590_000_000,(Track(1,(.8,.2),(0.,0.),1_590_000_000,'confirmed',Geometry('circle',radius=1.7,source='test')),))
    c=factorial(h,p,w,old,new)['cases']
    assert 'unavailable' in c['native_state__worker_prediction']
    assert 'unavailable' in c['worker_state__worker_prediction']
    assert 'at_recorded_track_step_m' in c['native_state__native_prediction']
