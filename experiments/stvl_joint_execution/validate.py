"""Assert decisive checker fixtures against labels frozen before execution."""
import pathlib,json
ROOT=pathlib.Path(__file__).resolve().parents[2];cache=ROOT/'build/joint_cache';rows=[]
for case,expected in json.loads((cache/'unit_expectations.json').read_text()).items():
    result=json.loads((cache/'unit_results'/f'{case}.json').read_text());actual=result['results'][0]
    ok=all(actual[k]==v for k,v in expected.items());rows.append({'case':case,'pass':ok,'expected':expected,'observed':actual})
assert len(rows)==8 and all(r['pass'] for r in rows),rows
(ROOT/'experiments/stvl_joint_execution/evidence/unit_results.json').write_text(json.dumps(rows,indent=2)+'\n');print('8/8 frozen decisive fixtures pass')
