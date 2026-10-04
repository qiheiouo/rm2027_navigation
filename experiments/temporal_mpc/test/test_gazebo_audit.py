import copy
import json
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from audit_run import physical_projection, epoch, hull


def pose(child,parent,x=0.,y=0.,z=0.,ns=20_000_000,q=None):
    return dict(child_frame_id=child,header=dict(frame_id=parent,stamp=dict(sec=0,nanosec=ns)),
                transform=dict(translation=dict(x=x,y=y,z=z),rotation=q or dict(x=0.,y=0.,z=0.,w=1.)))


def robot():
    model='rm_sentry_2027'
    return dict(transforms=[pose(model,'phase1_omni',3.,1.),pose(model+'/base_link',model),
        *[pose(model+'/'+n,model,x,y,.075) for n,x,y in (
            ('front_left_wheel',.25,.225),('front_right_wheel',.25,-.225),
            ('rear_left_wheel',-.25,.225),('rear_right_wheel',-.25,-.225))]])


def test_actual_relative_oracle_composes_world_and_covers_wheels():
    ns,p,center,r=physical_projection(robot(),'rm_sentry_2027','base_link')
    assert ns==20_000_000
    assert np.allclose(center,[3.,1.,0.])
    assert np.allclose(np.min(p,axis=0),[2.675,.700])
    assert np.allclose(np.max(p,axis=0),[3.325,1.300])


def test_actual_relative_actor_oracle_composes_world_rotation():
    q=dict(x=0.,y=0.,z=np.sqrt(.5),w=np.sqrt(.5))
    message=dict(transforms=[pose('moving_obstacle','phase1_omni',4.9,q=q),
                            pose('moving_obstacle/obstacle_link','moving_obstacle',y=.7,z=.4)])
    _,p,center,_=physical_projection(message,'moving_obstacle','obstacle_link')
    assert np.allclose(center,[4.2,0.,.4])
    assert np.allclose(np.ptp(p,axis=0),[.55,.45])


@pytest.mark.parametrize('fault',['missing_world','wrong_parent','mixed_epoch','wheel_offset','quaternion'])
def test_oracle_cannot_invent_missing_world_pose_or_mix_sources(fault):
    message=robot()
    if fault=='missing_world': message['transforms'].pop(0)
    if fault=='wrong_parent': message['transforms'][0]['header']['frame_id']='odom'
    if fault=='mixed_epoch': message['transforms'][1]['header']['stamp']['nanosec']+=1
    if fault=='wheel_offset': message['transforms'][2]['transform']['translation']['x']+=.1
    if fault=='quaternion': message['transforms'][0]['transform']['rotation']['w']=.5
    with pytest.raises((KeyError,ValueError)):physical_projection(message,'rm_sentry_2027','base_link')


def test_integer_source_contract_rejects_rounding():
    with pytest.raises(ValueError):epoch(dict(sec=1.,nanosec=0))
    with pytest.raises(ValueError):epoch(dict(sec=1,nanosec=10**9))


def test_generated_scene_preserves_literal_gazebo_friction_frame_extension(tmp_path):
    from prepare_scene import prepare
    import xml.etree.ElementTree as ET
    prepare(tmp_path)
    text=(tmp_path/'world.sdf').read_text()
    assert 'ns0:expressed_in' not in text
    assert text.count('fdir1 ignition:expressed_in="base_link"')==4
    source=Path(__file__).resolve().parents[3]/'src/rm_simulation/worlds/phase1_omni.sdf'
    before=ET.parse(source).getroot().find("world/model[@name='rm_sentry_2027']")
    after=ET.parse(tmp_path/'world.sdf').getroot().find("world/model[@name='rm_sentry_2027']")
    after.remove(after.find("plugin[@filename='ignition-gazebo-pose-publisher-system']"))
    # Same joint/collision/friction/drive/sensor tree; ignore only formatting.
    def value(node):return node.tag,node.attrib,(node.text or '').strip(),[value(c) for c in node]
    assert value(before)==value(after)
    sensor=ET.parse(tmp_path/'world.sdf').getroot().find("world/model[@name='moving_obstacle']/link[@name='obstacle_link']/sensor")
    assert sensor.find('contact/topic').text=='/simulation/oracle/contacts'
