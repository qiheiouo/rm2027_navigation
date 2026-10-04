from pathlib import Path
import sys
import subprocess
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from prepare_scene import prepare
from fixture_profile import fixture_profile
from static_grid import grid_for
from temporal_mpc.frontend import prepare_route
from temporal_mpc.contracts import NOMINAL_DIAMETER


def test_new_profile_changes_map_goal_and_keeps_authored_plant(tmp_path):
    prepare(tmp_path/'legacy');prepare(tmp_path/'long',profile='open_long')
    for filename in ('world.sdf','nav2.yaml','tracker.yaml','bridge.yaml'):
        assert (tmp_path/'legacy'/filename).read_bytes()==(tmp_path/'long'/filename).read_bytes()
    p=fixture_profile('open_long');grid=grid_for('crossing',p['width'],p['height'],p['resolution'],p['origin'])
    assert grid.shape==(200,240) and grid.size<=100000 and p['goal']==[8.5,0.]
    assert np.all(grid[[0,-1]]==100) and np.all(grid[:,[0,-1]]==100)
    with pytest.raises(ValueError):fixture_profile('unknown')


def test_tdt_certifies_long_goal_and_full_D_lateral_space(tmp_path):
    root=Path(__file__).resolve().parents[1];binary=tmp_path/'frontend'
    subprocess.run(['bash',str(root/'frontend/build.sh'),str(binary)],check=True,capture_output=True)
    p=fixture_profile('open_long');grid=grid_for('crossing',p['width'],p['height'],p['resolution'],p['origin'])
    route=prepare_route(binary,np.where(grid>=65,254,0).astype(np.uint8),.05,tuple(p['origin']),(0.,0.),p['goal'],corridor_range=p['corridor_range'])
    w=route.window(np.zeros(6),0,np.arange(31)*.05)
    assert w.centre_bounds[1]>8.5 and w.centre_bounds[2]<-3.6 and w.centre_bounds[3]>3.6
    # Authored fixture design only: largest head-on x joint limit + body face.
    # This bound is never sent to control or used as perception confidence.
    anchor_x=4.9+.95+.225
    source_goal_slack=8.5-anchor_x-.355-NOMINAL_DIAMETER-.02-.025*(np.hypot(.8,.5)+3.)
    assert source_goal_slack-.15>0.
    for value in (float('nan'),0.,13.):
        with pytest.raises(Exception,match='corridor range'):
            prepare_route(binary,np.zeros((3,3),np.uint8),.05,(0.,0.),(0.,0.),(1.,1.),corridor_range=value)
    for value in ('nan','13','1suffix'):
        rejected=subprocess.run([str(binary),'--range',value],input='',text=True,capture_output=True)
        assert rejected.returncode==2
