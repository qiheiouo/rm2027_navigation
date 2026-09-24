#!/usr/bin/env python3
"""Audit visible-cluster occupancy against the known fixture box, offline only."""
import hashlib
import json
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PROBE = HERE / 'probe_v2.json'
BOX = ROOT / 'src/rm_simulation/models/moving_obstacle.sdf'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def box_size():
    root = ET.parse(BOX)
    size = root.find('.//collision/geometry/box/size')
    assert size is not None
    values = [float(x) for x in size.text.split()]
    assert len(values) == 3 and all(x > 0 for x in values)
    return values[:2]


def deficits(row, size):
    """Per-axis missing reach beyond the observed, axis-aligned cluster box."""
    out = {}
    for axis, label in enumerate(('x', 'y')):
        center = row['estimated_xy'][axis]
        half_visible = row['estimated_visible_cluster_size_xy'][axis] / 2
        actual = row['actual_obstacle_xy'][axis]
        half_box = size[axis] / 2
        out[label] = {
            'negative_m': max(0.0, center - half_visible - (actual - half_box)),
            'positive_m': max(0.0, (actual + half_box) - (center + half_visible)),
        }
    return out


def summarize(rows, size):
    ds = [deficits(row, size) for row in rows]
    result = {'samples': len(rows), 'fully_contained': sum(
        all(missing[side] <= 1e-9 for missing in d.values() for side in ('negative_m', 'positive_m'))
        for d in ds)}
    for axis in ('x', 'y'):
        for side in ('negative_m', 'positive_m'):
            values = [d[axis][side] for d in ds]
            result[f'{axis}_{side}_median'] = statistics.median(values) if values else None
            result[f'{axis}_{side}_max'] = max(values) if values else None
    return result


def main():
    probe = json.loads(PROBE.read_text())
    assert probe['schema'] == 'rm_dynamic_prediction_v1_tracker_probe/v2'
    size = box_size()
    rows = probe['rows']
    assert len(rows) == probe['scan_count']
    windows = {
        'all_precontact_confirmed': [r for r in rows if r['t'] < probe['first_body_overlap_s'] and r['confirmed_near_box']],
        'last_2s_precontact_confirmed': [r for r in rows if 26 <= r['t'] <= 28.1 and r['confirmed_near_box']],
    }
    result = {
        'schema': 'rm_dynamic_prediction_v1_visible_coverage/v1',
        'scope': 'Frozen-scan offline oracle only; actual box pose and full size are evaluation truth, never runtime tracker inputs.',
        'probe_sha256': sha256(PROBE),
        'box_sdf_sha256': sha256(BOX),
        'box_size_xy_m': size,
        'windows': {name: summarize(subset, size) for name, subset in windows.items()},
        'runtime_occupancy_validated': False,
        'accepted_for_deployment': False,
    }
    with (HERE / 'coverage_audit.json').open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps(result['windows'], indent=2))


if __name__ == '__main__':
    main()
