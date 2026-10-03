"""Stop labeling must survive interruption, missing receipts and speed spikes."""
from audit_brake_response import zero_episodes,sustained_stop


def cmd(t,v):return {'receive_sim':t,'velocity':[v,0,0,0,0,0]}

def odom(t,v=0,w=0):return {'stamp':t,'velocity':[v,0,w]}


def test_receipt_gap_is_not_a_sustained_zero_command():
    e=zero_episodes([cmd(0,1),cmd(.02,0),cmd(.04,0),cmd(.3,0)])
    assert len(e)==2 and e[0]['end_reason']=='command_receipt_gap'
    assert e[1]['previous']['velocity']==[0]*6


def test_nonzero_resume_ends_braking_window():
    e=zero_episodes([cmd(0,1),cmd(.02,0),cmd(.04,1)])
    assert len(e)==1 and e[0]['last']['receive_sim']==.02


def test_one_zero_speed_sample_does_not_establish_stop():
    assert sustained_stop([odom(0),odom(.02,.2),odom(.04)],0,.1) is None


def test_motion_gap_resets_stop_confirmation():
    assert sustained_stop([odom(0),odom(.3),odom(.32)],0,.32) is None


def test_rotation_is_not_a_stopped_footprint():
    assert sustained_stop([odom(i*.02,0,.02) for i in range(20)],0,1) is None


def test_stop_requires_continuous_quiet_samples():
    result=sustained_stop([odom(i*.02) for i in range(15)],0,1)
    assert result['stop_source_stamp']==0 and result['confirmation_source_stamp']>=.2
