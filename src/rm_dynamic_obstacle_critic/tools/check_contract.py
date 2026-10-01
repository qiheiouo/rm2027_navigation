#!/usr/bin/env python3
"""Check experiment horizon, padded geometry, and baseline MPPI parameter preservation."""
from pathlib import Path
import argparse
import copy
import yaml
ROOT=Path(__file__).resolve().parents[3]
PKG=Path(__file__).resolve().parents[1]
base=yaml.safe_load((ROOT/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml').read_text())
parser=argparse.ArgumentParser();selection=parser.add_mutually_exclusive_group()
selection.add_argument('--static-stopping',action='store_true')
selection.add_argument('--map-uncertainty',action='store_true');args=parser.parse_args()
stopping=args.static_stopping or args.map_uncertainty
profile_name='nav2_cv_map_uncertainty.yaml' if args.map_uncertainty else 'nav2_cv_static_stopping.yaml' if stopping else 'nav2_cv_experiment.yaml'
profile=yaml.safe_load((PKG/'config'/profile_name).read_text())
original=base['controller_server']['ros__parameters']['FollowPath'];current=profile['controller_server']['ros__parameters']['FollowPath']
for key,value in original.items():
    if key=='critics':assert current[key]==value+['DynamicObstacleCritic']+(['StaticStoppingCritic'] if stopping else [])
    else:assert current[key]==value,(key,'baseline MPPI changed')
critic=current['DynamicObstacleCritic']
tracker=yaml.safe_load((PKG/'config/tracker_cv.yaml').read_text())['dynamic_obstacle_tracker_shadow']['ros__parameters']
guard=yaml.safe_load((PKG/'config/guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
assert abs(critic['prediction_horizon']-current['model_dt']*current['time_steps'])<1e-9
assert tracker['prediction']['steps']==current['time_steps'] and tracker['prediction']['dt']==current['model_dt']
assert tracker['prediction']['velocity_decay_tau']==0.0
for key in ['max_age','max_observation_age','max_obstacle_speed','max_obstacle_extent','minimum_obstacle_radius','jump_tolerance','max_tf_age','max_tracks','input_frame','safety_margin']:
    assert critic[key]==guard[key],key
cm=profile['local_costmap']['local_costmap']['ros__parameters'];pad=cm['footprint_padding'];footprint=yaml.safe_load(cm['footprint'])
assert guard['footprint']==[v+(pad if v>0 else -pad if v<0 else 0) for xy in footprint for v in xy]
assert guard['collision_threshold']==203
assert profile['local_costmap']['local_costmap']['ros__parameters']['always_send_full_costmap']
if stopping:
    for key in ['horizon','simulation_dt','response_delay','linear_deceleration','angular_deceleration','max_simulation_steps','collision_threshold']:
        assert current['StaticStoppingCritic'][key]==guard[key],key
    assert current['StaticStoppingCritic']['rejection_cost']==10000
    assert abs(1/profile['controller_server']['ros__parameters']['controller_frequency']-current['model_dt'])<1e-6
if args.map_uncertainty:
    unbuffered=yaml.safe_load((PKG/'config/nav2_cv_static_stopping.yaml').read_text())
    comparison=copy.deepcopy(profile)
    margin=comparison['controller_server']['ros__parameters']['FollowPath']['StaticStoppingCritic'].pop('map_uncertainty_margin')
    assert margin==.11 and comparison==unbuffered,'uncertainty profile must change only its planning allowance'
print('PASS: unchanged native MPPI, matched 3s CV grid, shared boundary parameters and padded footprint')
