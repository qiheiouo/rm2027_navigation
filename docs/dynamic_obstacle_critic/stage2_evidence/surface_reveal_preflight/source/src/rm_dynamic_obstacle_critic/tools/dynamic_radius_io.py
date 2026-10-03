"""Offline interpretation of explicitly selected runtime radius semantics."""
import math

V1 = 'rm_dynamic_obstacle_predictions/v1'
V2 = 'rm_dynamic_obstacle_predictions/v2_observation_anchor'


def obstacle_radius(minimum, size, input_schema=V1, mode='visible_extent', diameter=0., prior=''):
    if input_schema not in (V1,V2):raise ValueError('unsupported radius input schema')
    if mode=='visible_extent':
        if diameter!=0. or prior!='':raise ValueError('unused nominal radius metadata')
        return max(minimum,.5*math.hypot(*size))
    if (mode!='nominal_diameter' or input_schema!=V2 or not math.isfinite(diameter) or
            not 0.<diameter<=3. or not isinstance(prior,str) or
            not prior.strip(' \t\n\r\f\v') or len(prior.encode())>4096):
        raise ValueError('invalid nominal radius metadata')
    return max(minimum,diameter)


def profile_radius(parameters, size):
    return obstacle_radius(parameters['minimum_obstacle_radius'],size,
        parameters.get('input_schema',V1),parameters.get('obstacle_radius_mode','visible_extent'),
        parameters.get('nominal_obstacle_diameter',0.),parameters.get('geometry_prior_source',''))


def consumed_radius(limits, size, version):
    if version==1:return obstacle_radius(limits['minimum_radius'],size)
    if version!=2:raise ValueError('unsupported consumed radius version')
    return obstacle_radius(limits['minimum_radius'],size,limits['input_schema'],
        limits['obstacle_radius_mode'],limits['nominal_obstacle_diameter'],limits['geometry_prior_source'])
