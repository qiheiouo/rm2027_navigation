"""Offline observation-chain prototype; only endpoints are measured evidence.

Nominal chords assume local continuity. Their interiors, hidden geometry and
future occupancy are not certified. No ROS, TF, truth or tracker state here.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

XY=tuple[float,float]


@dataclass(frozen=True)
class SurfaceConfig:
    max_neighbor_distance:float=.20
    max_range_step:float=.20
    max_points:int=4096
    max_elements:int=1024
    nominal_prediction_lifetime:float=.5
    max_nominal_speed:float=3.

    def __post_init__(self):
        for value in (self.max_neighbor_distance,self.max_range_step,self.nominal_prediction_lifetime,self.max_nominal_speed):
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError('invalid finite positive surface policy')
        if any(type(v) is not int or not 1<=v<=4096 for v in (self.max_points,self.max_elements)):raise ValueError('invalid surface budget')


@dataclass(frozen=True)
class SurfaceElement:
    start:XY
    end:XY
    source_indices:tuple[int,...]
    kind:str
    interior_evidence:str


@dataclass(frozen=True)
class ObservedSurface:
    source_stamp_ns:int
    elements:tuple[SurfaceElement,...]
    observed_source_indices:tuple[int,...]
    breaks:tuple[tuple[int,int,tuple[str,...]],...]
    support_is_certified:bool=False
    hidden_space_status:str='UNKNOWN'


@dataclass(frozen=True)
class SurfacePrediction:
    elements:tuple[SurfaceElement,...]
    age_plus_future_seconds:float
    status:str
    support_is_certified:bool=False
    rotation_modeled:bool=False
    hidden_space_status:str='UNKNOWN'


def extract_observed_surface(points:Sequence[XY],beam_indices:Sequence[int],*,ranges:Sequence[float],
                             source_stamp_ns:int,range_min:float,range_max:float,
                             config:SurfaceConfig=SurfaceConfig())->ObservedSurface:
    if type(source_stamp_ns) is not int or source_stamp_ns<=0:raise ValueError('positive integer source epoch required')
    if (type(range_min) not in (int,float) or type(range_max) not in (int,float)
            or not math.isfinite(range_min) or not math.isfinite(range_max) or not 0<=range_min<range_max):raise ValueError('invalid sensor range interval')
    if len(points)!=len(beam_indices) or len(points)>config.max_points or len(ranges)>100_000:raise ValueError('source shape or surface point budget')
    if any(type(i) is not int or not 0<=i<len(ranges) for i in beam_indices) or len(set(beam_indices))!=len(beam_indices):raise ValueError('original beam identity required')
    if any(len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) for p in points):raise ValueError('finite XY endpoints required')
    if any(type(ranges[i]) not in (int,float) or not math.isfinite(ranges[i]) or not range_min<=ranges[i]<=range_max for i in beam_indices):raise ValueError('invalid member return')
    ordered=sorted((i,(float(p[0]),float(p[1]))) for i,p in zip(beam_indices,points))
    elements=[];connected=set();breaks=[]
    for (a,p),(b,q) in zip(ordered,ordered[1:]):
        reasons=[]
        if b!=a+1:reasons.append('source_index_gap')
        if abs(ranges[b]-ranges[a])>config.max_range_step:reasons.append('range_discontinuity')
        if math.dist(p,q)>config.max_neighbor_distance:reasons.append('spatial_discontinuity')
        if reasons:breaks.append((a,b,tuple(reasons)))
        else:
            elements.append(SurfaceElement(p,q,(a,b),'nominal_chord','LOCAL_CONTINUITY_ASSUMPTION'))
            connected.update((a,b))
    for index,p in ordered:
        if index not in connected:elements.append(SurfaceElement(p,p,(index,),'observed_point','OBSERVED_ENDPOINT_ONLY'))
    elements.sort(key=lambda e:e.source_indices)
    if len(elements)>config.max_elements:raise ValueError('complete surface element budget exceeded')
    return ObservedSurface(source_stamp_ns,tuple(elements),tuple(i for i,p in ordered),tuple(breaks))


def propagate_nominal(surface:ObservedSurface,velocity:XY,*,source_age_seconds:float,future_seconds:float,
                      config:SurfaceConfig=SurfaceConfig())->SurfacePrediction:
    if (len(velocity)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in velocity)
            or math.hypot(*velocity)>config.max_nominal_speed):raise ValueError('invalid bounded nominal translation')
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in (source_age_seconds,future_seconds)):raise ValueError('nonnegative finite prediction epochs required')
    horizon=source_age_seconds+future_seconds
    if not math.isfinite(horizon):raise ValueError('prediction epoch overflow')
    def shifted(p):
        point=(p[0]+velocity[0]*horizon,p[1]+velocity[1]*horizon)
        if not all(math.isfinite(v) for v in point):raise ValueError('prediction coordinate overflow')
        return point
    predicted=tuple(SurfaceElement(shifted(e.start),shifted(e.end),e.source_indices,e.kind,e.interior_evidence) for e in surface.elements)
    status='NO_OBSERVATION_UNKNOWN' if not surface.elements else ('NOMINAL_SHORT_CV_UNCERTIFIED' if horizon<=config.nominal_prediction_lifetime else 'EXPIRED_GEOMETRY_UNKNOWN_TREND_ONLY')
    return SurfacePrediction(predicted,horizon,status)
