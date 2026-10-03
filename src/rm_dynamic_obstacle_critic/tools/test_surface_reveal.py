"""Counterexamples for source-ray identity, confirmation and stopping labels."""
import hashlib,json,math
from pathlib import Path
import pytest
from audit_surface_reveal import face_bin,material_segment,reliable_keys,closing_sensitivity,linear_values,validate
from audit_scan_geometry import first_box


def test_occluded_surface_has_no_actor_return_label():
    geometry={'static_pose':(2,0,0),'static_dimensions':(.4,1),'actor_dimensions':(.4,.6)}
    assert first_box((0,0,0),0,(4,0,0),geometry,.1,8)[1]=='static'


def test_corners_are_not_forced_onto_one_surface():
    assert face_bin((.225,.275),(.45,.55),.03) is None
    assert face_bin((-.225,.26),(.45,.55),.03) is None
    assert face_bin((-.225,0),(.45,.55),.03)==('x_minus',5)


def test_material_bins_do_not_extend_box_endpoints():
    assert material_segment('x_minus',10,(.45,.55))==[(-.225,.22499999999999998),(-.225,.27499999999999997)]
    a,b=material_segment('y_plus',8,(.45,.55))
    assert a[0]>=-.225 and b[0]<=.225


def test_sparse_face_cannot_be_declared_reliable():
    key=('x_minus',0)
    assert not reliable_keys({key:1},{key:1},1.1,1.)


def test_two_frames_require_the_same_material_bin():
    a=('x_minus',0);b=('x_minus',1)
    assert not reliable_keys({a:3},{b:3},1.1,1.)
    assert reliable_keys({a:3},{a:3},1.1,1.)=={a}


def test_gap_or_duplicate_clock_breaks_confirmation():
    counts={('x_minus',0):3}
    assert not reliable_keys(counts,counts,1.2,1.)
    assert not reliable_keys(counts,counts,1.,1.)
    assert not reliable_keys(counts,counts,1.,None)


def test_twist_rate_is_not_wrapped_as_pose_yaw():
    assert linear_values([0.,1.],[(0,0,4.),(0,0,-4.)],.5,1.)==(0,0,0)
    assert linear_values([0.,1.],[(0,0,4.),(0,0,-4.)],.5,.1) is None
    assert linear_values([0.,1.],[(0,0,4.),(0,0,-4.)],0.,.1)==(0,0,4.)


def test_stopped_robot_still_has_target_approach_risk():
    model=next(r for r in closing_sensitivity(.1,0.,.7) if r['tau_s']==.2 and r['assumed_deceleration_mps2']==1.)
    assert model['static_target_reveal_margin_m']>0
    assert model['instant_actor_speed_sensitivity_margin_m']<0


@pytest.mark.parametrize('values',[(math.nan,1,1),(1,-1,1),(1,1,math.inf)])
def test_invalid_stopping_values_do_not_gain_pass_label(values):
    with pytest.raises(ValueError):closing_sensitivity(*values)


def test_manifest_detects_changed_or_extra_input(tmp_path,monkeypatch):
    p=tmp_path/'input.json';p.write_text('{}')
    manifest=tmp_path/'manifest.json';manifest.write_text(json.dumps({'files':{'input.json':hashlib.sha256(p.read_bytes()).hexdigest()}}))
    assert len(validate(tmp_path))==64
    monkeypatch.chdir(tmp_path)
    assert validate(Path('.'))==validate(tmp_path)
    p.write_text('{"changed":true}')
    with pytest.raises(ValueError,match='identity'):validate(tmp_path)
    p.write_text('{}');(tmp_path/'unregistered').write_text('')
    with pytest.raises(ValueError,match='file set'):validate(tmp_path)
