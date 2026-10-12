"""Native launch substitution evaluation only: never executes nodes or hardware."""
import importlib.util
import json
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import evaluate_parameters

ROOT = Path.cwd()
PREFIX = ROOT / 'build/aud001/profile_ament'
for p in (ROOT / 'src').iterdir():
    if not (p / 'package.xml').exists():
        continue
    marker = PREFIX / 'share/ament_index/resource_index/packages' / p.name
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()
    share = PREFIX / 'share' / p.name
    if not share.exists():
        share.symlink_to(p, target_is_directory=True)
os.environ['AMENT_PREFIX_PATH'] = str(PREFIX) + ':' + os.environ.get('AMENT_PREFIX_PATH', '')


def load(package, name):
    path = ROOT / 'src' / package / 'launch' / name
    spec = importlib.util.spec_from_file_location(package+name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def enter(module, inherited):
    c = LaunchContext()
    c.launch_configurations.update(inherited)
    d = module.generate_launch_description()
    for action in d.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(c)
    return c, d


def resolve(value, c):
    return perform_substitutions(c, normalize_to_list_of_substitutions(value))


def includes(d, c):
    found = {}
    for a in d.entities:
        if isinstance(a, IncludeLaunchDescription) and (not a.condition or a.condition.evaluate(c)):
            filename = Path(resolve(a.launch_description_source._LaunchDescriptionSource__location, c)).name
            found[filename] = {k: resolve(v, c) for k, v in a.launch_arguments}
    return found

rows = []
for backend, topic in [('amcl_2d', '/localization/amcl_backend_valid'),
                       ('gicp_3d', '/localization/gicp_backend_valid')]:
    m = load('rm_navigation_bringup', 'navigation.launch.py')
    c, d = enter(m, {'relocalization_backend': backend,
                     'global_localization_mode': 'external_pose', 'use_map_server': 'true'})
    m._validate_localization_backend(c)
    args = includes(d, c)['phase2c_relocalization_bringup.launch.py']
    phase = load('rm_navigation_bringup', 'phase2c_relocalization_bringup.launch.py')
    pc, pd = enter(phase, args)
    args = includes(pd, pc)['global_pose_bridge.launch.py']
    bridge = load('rm_relocalization_bridge', 'global_pose_bridge.launch.py')
    bc, bd = enter(bridge, args)
    node = bridge._launch_bridge(bc)[0]
    params = evaluate_parameters(bc, node._Node__parameters)
    assert params[-1]['upstream_valid_topic'] == topic
    rows.append({'entry': 'generic', 'backend': backend, 'bridge_status_topic': topic+'/status'})
    comp = load('rm_navigation_bringup', 'old_car_2026_competition.launch.py')
    cc, cd = enter(comp, {'enable_competition_stack': 'true', 'relocalization_backend': backend})
    args = includes(cd, cc)['global_pose_bridge.launch.py']
    assert args['upstream_valid_topic'] == topic
    rows.append({'entry': 'old_car_competition', 'backend': backend, 'bridge_status_topic': topic+'/status'})
    standalone = load('rm_navigation_bringup', 'old_car_2026_'+('amcl' if backend=='amcl_2d' else 'gicp')+'_relocalization.launch.py')
    sc, sd = enter(standalone, {'enable_relocalization': 'true'})
    args = includes(sd, sc)['global_pose_bridge.launch.py']
    assert args['upstream_valid_topic'] == topic
    rows.append({'entry': 'old_car_standalone', 'backend': backend, 'bridge_status_topic': topic+'/status'})
result = {'passed': True, 'scope': 'Native launch parameters only; no node launch or map/hardware validation.', 'cases': rows}
OUT = Path(os.environ['PROBE_OUTPUT'])
OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
