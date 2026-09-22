"""Separate fixture correction: derive required map width, preserve v1."""
import json
import math
import subprocess
import sys
import yaml
from pathlib import Path
import audit_dynamic
ex=audit_dynamic.ex
ex.RUNS=ex.WORK/'runs/dynamic_reference_pilot_v2'

def prepare():
    ex.RUNS.mkdir(exist_ok=False);(ex.RUNS/'profiles').mkdir()
    original=ex.WORK/'runs/dynamic_reference_pilot_v1';m=json.loads((original/'inputs.json').read_text())
    radius=math.hypot(.2800954544+.03,.191+.03)
    # Goal distance + padded radius/clearance + boundary cell + rolling quantization.
    # Nav2 width is an integer metre parameter. Retain resolution and all safety terms.
    minimum_width=2*(5.6+radius+.02+1e-7+2*.05)
    width=math.ceil(minimum_width);assert width==13
    for name in ('tdt_astar','tdt_qp'):
        before=yaml.safe_load((original/'profiles'/f'{name}.yaml').read_text());after=json.loads(json.dumps(before))
        after['global_costmap']['global_costmap']['ros__parameters']['width']=width
        target=ex.RUNS/'profiles'/f'{name}.yaml';target.write_text(yaml.safe_dump(after,sort_keys=False))
        m['profiles'][name]=ex.sha(target)
        after['global_costmap']['global_costmap']['ros__parameters']['width']=12
        assert before==after
    m['fixture_correction']={'only_parameter':'global_costmap.global_costmap.ros__parameters.width',
        'before_m':12,'after_m':width,'minimum_derived_m':minimum_width,
        'reason':'5.6m goal plus derived padded-radius/clearance and full boundary/rolling cells must fit from initial pose. No footprint or safety threshold changes.'}
    m['source_commit']=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    m['files'].update({str(p.relative_to(ex.REPO)):ex.sha(p) for p in ex.HERE.iterdir() if p.suffix in ('.py','.sh')})
    ex.write_new(ex.RUNS/'inputs.json',m)
if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    elif sys.argv[1]=='run':sys.exit(ex.run(sys.argv[2]))
