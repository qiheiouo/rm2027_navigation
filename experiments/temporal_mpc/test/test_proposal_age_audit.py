from pathlib import Path
import sys
import json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from audit_proposal_age import cases
from audit_execution import native_states
from audit_reanchor_inputs import dynamic_margins
from temporal_mpc.contracts import Snapshot, Track, Geometry


def fixture(age):
    s=Snapshot(1_000_000_000,(Track(1,(3.,0.),(-.2,0.),1_000_000_000,'confirmed',Geometry('circle',radius=1.7,source='test')),))
    h=dict(evaluation_ns=s.source_ns+age,proposal_ns=s.source_ns,generation=2,initial_state=[0.,0.,0.,0.,0.,0.],executed=True,
           reanchor_projection='velocity_and_stop/v2',constraint='dynamic_clearance',step=30,track_id=1)
    p=dict(accelerations=[dict(x=.4 if k<15 else -.4,y=0.,z=0.) for k in range(30)])
    h['slack']=float(dynamic_margins(native_states(h,p),s,h['evaluation_ns'])[0][0,30])
    w=dict(epoch_ns=s.source_ns,generation=2,input_source_ns=s.source_ns,input_validated=True,initial_state=h['initial_state'].copy())
    return h,p,w,s


def test_age_projection_is_checked_separately_from_raw_shift_and_inputs_unchanged():
    h,p,w,s=fixture(19_000_000);original=json.dumps([h,p,w]);r=cases(h,p,w,s,s);c=r['cases']
    assert r['actual_replay_match'] and r['age_ns']==19_000_000
    assert c['evaluation_shifted_without_terminal_projection']['terminal_speed_max']>.001
    assert c['evaluation_native_projection_old_inputs']['terminal_speed_max']<1e-5
    assert c['evaluation_original_relative_controls']['full_dynamic_min_m']<c['worker_epoch_original']['full_dynamic_min_m']
    assert json.dumps([h,p,w])==original


def test_zero_age_has_no_relative_time_change():
    h,p,w,s=fixture(0);r=cases(h,p,w,s,s)['cases']
    assert r['worker_epoch_original']==r['evaluation_original_relative_controls']
    assert r['worker_epoch_native_projection']==r['evaluation_native_projection_old_inputs']
