#!/usr/bin/env python3
"""Offline manual-derived radius hypotheses on frozen actual consumed CV fields."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import yaml
from audit_consumed_cv_support import audit
from robot_extent_prior import anchor_envelope_radius, class_diameters


def analyze(source, reference):
    manifest=json.loads((source/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        p=(source/name).resolve()
        if not p.is_relative_to(source.resolve()) or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
            raise ValueError('frozen source identity mismatch')
    config=yaml.safe_load(reference.read_text());diameters=class_diameters(config)
    if set(diameters)!={'hero','engineer','infantry','sentry'}:raise ValueError('user requires all four ground robot classes')
    geometry=audit(source,include_geometry_requirements=True)
    radius=anchor_envelope_radius(config,'unknown_within_listed_ground_classes',0.)
    hypotheses={'original_consumed_radius':None,'false_800mm_center_half_diagonal':.5*math.hypot(.8,.8),
        'all_ground_convex_hull_anchor_zero_error_hypothesis':radius}
    usable=[r for r in geometry['records'] if r['usable_actor_track']];summaries={};records=[]
    for name, fixed in hypotheses.items():
        summaries[name]={}
        for horizon in ('0.0','1.0','2.0','3.0'):
            required=[r['horizons'][horizon]['maximum_corner_distance'] for r in usable]
            deficits=[max(0.,d-(r['used_radius'] if fixed is None else fixed)) for r,d in zip(usable,required)]
            summaries[name][horizon]={'scores':len(usable),'full_physical_box_covered':sum(d==0 for d in deficits),
                'support_deficit_max':max(deficits,default=None),'required_radius_median':statistics.median(required) if required else None,
                'required_radius_max':max(required,default=None)}
    for r in usable:
        records.append({'ordinal':r['ordinal'],'score_stamp':r['score_stamp'],'used_radius':r['used_radius'],
            'anchor_to_current_actor_projection_distance':r['current_anchor_to_actor_projection_distance'],
            'required_radius_by_horizon':{h:v['maximum_corner_distance'] for h,v in r['horizons'].items()}})
    errors=[r['anchor_to_current_actor_projection_distance'] for r in records]
    return {'schema':1,'source_manifest_sha256':hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
        'reference_yaml_sha256':hashlib.sha256(reference.read_bytes()).hexdigest(),'manual_reference':config['reference'],
        'target_scope':config['scope'],'class_diameter_upper_bounds':diameters,'zero_error_radius_hypothesis':radius,
        'scores':geometry['scores'],'usable_actor_scores':len(usable),'no_usable_actor_scores':geometry['scores']-len(usable),
        'filtered_CV_anchor_outside_current_actor_projection_scores':sum(e>0 for e in errors),
        'empirical_anchor_projection_distance_max':max(errors,default=None),'hypotheses':summaries,'records':records,
        'scope':'Offline reference/hypothesis support labels on exact consumed CV states and unchanged score clock; no online radius change.',
        'limits':['2026 compliance/classification/attachment and 2027 applicability remain conditions, not certified facts.',
            'Zero anchor error is an explicit hypothesis and may fail membership, even when a large disk empirically covers this actor.',
            'Current diameter bound provides no future CV turn/rotation/deformation or source/measurement error certificate.',
            'No truth selected control/parameter, no CA/sampler/ranking change, no deployment acceptance.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for name in ['source','reference','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();report=analyze(args.source.resolve(),args.reference)
    with args.output.open('x') as stream:stream.write(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'}))
