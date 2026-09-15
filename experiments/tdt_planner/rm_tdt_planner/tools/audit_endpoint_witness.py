#!/usr/bin/env python3
"""Audit exact core rejection witnesses, independently of published costmap frames.

A witness certifies one sufficient rejecting square, not the nearest square, the
whole snapshot, actual physical contact, or rectangle/path/rotation feasibility.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

MARKER = 'endpoint_witness_v1='
CONTEXT = re.compile(r'\[input=nav2_master start=\(([^,]+),([^\)]+)\) goal=\(([^,]+),([^\)]+)\) '
                     r'origin=\(([^,]+),([^\)]+)\) size=(\d+)x(\d+) resolution=(\S+) '
                     r'radius=(\S+) clearance=(\S+) start_yaw=(\S+) goal_yaw=([^\]]+)\]')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'nonfinite/nonnumeric value')
    return value


def pair(value):
    require(isinstance(value, list) and len(value) == 2, 'expected coordinate pair')
    return tuple(number(x) for x in value)


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key')
        result[key] = value
    return result


def audit_message(message):
    require(message.count(MARKER) == 1, 'missing/duplicate witness marker')
    payload, _ = json.JSONDecoder(object_pairs_hook=unique).raw_decode(message.split(MARKER)[1])
    require(isinstance(payload, dict), 'expected witness object')
    ox, oy = pair(payload['origin'])
    width, height = pair(payload['size'])
    require(type(width) is int and type(height) is int and 3 <= width <= 4096 and
            3 <= height <= 4096 and width*height <= 1000000, 'invalid grid size')
    r, radius, clearance, guard = (number(payload[k]) for k in ('resolution', 'radius', 'clearance', 'guard'))
    require(.001 <= r <= 1 and 0 <= radius <= 5 and 0 <= clearance <= 2 and guard == 1e-7,
            'invalid geometry parameters')
    match = CONTEXT.search(message)
    require(match is not None, 'missing exact plugin request context')
    context = [float(x) for x in match.groups()]
    require(all(math.isfinite(x) for x in context), 'nonfinite plugin context')
    sx, sy, gx, gy, cx, cy, cw, ch, cr, ra, cl, syaw, gyaw = context
    require((cx, cy, cw, ch, cr, ra, cl) == (ox, oy, width, height, r, radius, clearance),
            'core/plugin snapshot geometry mismatch')
    result = {}
    for name, world, yaw in (('start', (sx, sy), syaw), ('goal', (gx, gy), gyaw)):
        item = payload[name]
        p = pair(item['local'])  # Plugin has already required finite poses.
        require(all(abs(v - q - origin) <= 1e-10 for v, q, origin in zip(world, p, (ox, oy))),
                'core/plugin endpoint mismatch')
        inside = 0 <= p[0] < width*r and 0 <= p[1] < height*r
        w = item['collision']
        status = item['status']
        checked = {'status': status, 'world': list(world), 'yaw': yaw}
        if status == 'blocked':
            require(inside and isinstance(w, dict), 'blocked endpoint without witness')
            x, y = pair(w['cell'])
            require(type(x) is int and type(y) is int and 0 <= x < width and 0 <= y < height,
                    'invalid witness cell')
            cost = w['cost']
            boundary = x in (0, width-1) or y in (0, height-1)
            require(type(cost) is int and 0 <= cost <= 255 and type(w['boundary']) is bool and
                    w['boundary'] == boundary, 'invalid raw cost/boundary flag')
            seed = cost >= 254 or boundary
            require(seed or cost == 253, 'nonblocking witness')
            expected = radius + clearance if seed else 0.0
            required, measured = number(w['required']), number(w['distance'])
            require(required == expected, 'wrong collision threshold')
            # Independent endpoint-to-closed-AABB clamp; core uses segment clipping.
            near = (min(max(p[0], x*r), (x+1)*r), min(max(p[1], y*r), (y+1)*r))
            distance = math.dist(p, near)
            require(abs(distance-measured) <= 1e-10 and distance <= expected+guard+1e-10,
                    'distance mismatch or nonrejecting square')
            checked.update(cell=[x,y], cost=cost, boundary=boundary,
                           world_box=[ox+x*r, oy+y*r, ox+(x+1)*r, oy+(y+1)*r],
                           distance_m=distance, required_m=required,
                           shortfall_m=required-distance)
        elif status == 'free':
            require(inside and w is None, 'invalid free endpoint')
            checked['free_claim_independently_verified'] = False  # No entire snapshot here.
        else:
            require(status == 'outside_or_nonfinite' and not inside and w is None,
                    'invalid outside endpoint')
        result[name] = checked
    require(any(result[n]['status'] != 'free' for n in ('start', 'goal')), 'no rejecting endpoint')
    return result


def audit_file(path):
    entries = []
    missing = 0
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        event = json.loads(line)
        message = event.get('message', '')
        if event.get('node') != 'planner_server':
            continue
        if 'start or goal outside conservative free space' not in message:
            continue
        if MARKER not in message:
            missing += 1
            continue
        entries.append({'line': line_number, 't': number(event['t']), **audit_message(message)})
    return {'schema': 'rm_tdt_planner/endpoint_witness_audit/v1',
            'source': str(path.resolve()), 'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'audited': bool(entries) and missing == 0, 'missing_witnesses': missing,
            'rejections': len(entries),
            'start_blocked': sum(e['start']['status'] == 'blocked' for e in entries),
            'goal_blocked': sum(e['goal']['status'] == 'blocked' for e in entries),
            'entries': entries,
            'scope': 'One sufficient raw snapshot cell per blocked endpoint. Free claims, nearest cell, '
                     'physical collision and whole-footprint feasibility are not certified.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        report = audit_file(args.events)
        with args.output.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(2, f'Witness audit failed: {error}\n')
    print(f"audited={report['audited']} rejections={report['rejections']} "
          f"start_blocked={report['start_blocked']} goal_blocked={report['goal_blocked']}")
    return 0 if report['audited'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
