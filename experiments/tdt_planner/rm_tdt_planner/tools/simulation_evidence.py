"""Summarize static simulation evidence; performance never rejects an algorithm."""
import collections
import math
import statistics


def analyze(samples, commands, events, goal_x, goal_y, status, timed_out, recoveries):
    keys = ('t', 'x', 'y', 'yaw', 'body_clearance_m', 'padded_clearance_m', 'vx', 'vy', 'wz')
    if len(samples) < 2 or not all(math.isfinite(s[k]) for s in samples for k in keys):
        raise ValueError('missing or non-finite trajectory evidence')
    gaps = [b['t'] - a['t'] for a, b in zip(samples, samples[1:])]
    if min(gaps) <= 0:
        raise ValueError('nonmonotonic trajectory timestamps')
    body = min(s['body_clearance_m'] for s in samples)
    padded = min(s['padded_clearance_m'] for s in samples)
    body_bound, padded_bound, traveled = body, padded, 0.0
    for a, b in zip(samples, samples[1:]):
        translation = math.hypot(b['x']-a['x'], b['y']-a['y'])
        angle = abs(math.remainder(b['yaw']-a['yaw'], 2*math.pi))
        traveled += translation
        # Conservative bound for linear translation/yaw interpolation only.
        # It is not a proof about unobserved Gazebo motion between messages.
        body_bound = min(body_bound, min(a['body_clearance_m'], b['body_clearance_m']) -
                         .5*(translation + math.hypot(.30, .25)*angle))
        padded_bound = min(padded_bound, min(a['padded_clearance_m'], b['padded_clearance_m']) -
                           .5*(translation + math.hypot(.33, .28)*angle))
    track = [s['cross_track_m'] for s in samples if s['cross_track_m'] is not None]
    if not all(math.isfinite(v) and v >= 0 for v in track):
        raise ValueError('invalid cross-track evidence')
    last = samples[-1]
    xy_error = math.hypot(last['x']-goal_x, last['y']-goal_y)
    yaw_error = abs(math.remainder(last['yaw'], 2*math.pi))
    commands = [c for c in commands if samples[0]['t'] <= c['t'] <= last['t'] + .1]
    if not all(math.isfinite(c[k]) for c in commands for k in ('t', 'vx', 'vy', 'wz')):
        raise ValueError('non-finite command evidence')
    stopped = bool(commands and -.1 <= last['t']-commands[-1]['t'] <= 1.0 and
                   max(abs(commands[-1][k]) for k in ('vx', 'vy', 'wz')) <= .01)
    command_acceleration = []
    for a, b in zip(commands, commands[1:]):
        dt = b['t'] - a['t']
        if dt > 1e-6:
            command_acceleration.append(math.hypot(b['vx']-a['vx'], b['vy']-a['vy'])/dt)
    warning_counts = collections.Counter()
    for event in events:
        for key in ('costmap or footprint changed', 'planning deadline exceeded',
                    'start or goal outside', 'Control loop missed', 'Failed to create'):
            if key in event.get('message', ''):
                warning_counts[key] += 1
    evidence_valid = bool(track and commands and max(gaps) <= .20)
    checks = {'action_succeeded': status == 4 and not timed_out,
              'body_clearance_at_least_005m': body_bound >= .05,
              'padded_footprint_no_contact': padded_bound > 0,
              'final_position_within_015m': xy_error <= .15,
              'final_yaw_within_020rad': yaw_error <= .20,
              'no_recovery': recoveries == 0, 'fresh_stopped_command': stopped}
    return {
        'schema': 'rm_tdt_planner/static_simulation/v1',
        'evidence_valid': evidence_valid, 'checks': checks,
        'static_geometry_and_goal_pass': bool(evidence_valid and all(checks.values())),
        'action_status': status, 'timed_out': timed_out, 'recoveries': recoveries,
        'goal': [goal_x, goal_y, 0.0], 'final_pose': [last['x'], last['y'], last['yaw']],
        'final_xy_error_m': xy_error, 'final_yaw_error_rad': yaw_error,
        'trajectory_samples': len(samples), 'max_sample_gap_sim_s': max(gaps),
        'min_body_clearance_m': body, 'min_padded_clearance_m': padded,
        'linear_pose_interpolation_body_bound_m': body_bound,
        'linear_pose_interpolation_padded_bound_m': padded_bound,
        'cross_track_rms_m': math.sqrt(statistics.mean(v*v for v in track)) if track else None,
        'cross_track_max_m': max(track) if track else None, 'traveled_m': traveled,
        'max_command_acceleration_m_s2': max(command_acceleration) if command_acceleration else None,
        'warning_counts': dict(warning_counts),
        'snapshot_rejection_observed': warning_counts['costmap or footprint changed'] > 0,
        'algorithm_rejected': False,
        'scope': 'Static fixture, sampled poses and linear pose interpolation; manual review still required.',
        'performance_policy': 'Record current-device limits; no performance patches or algorithm rejection.',
    }
