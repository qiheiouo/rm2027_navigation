"""Verify preserved evidence and freeze this diagnostic without self-hashing."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT/'build/tdt_p2b'
PREVIOUS = [
    'snapshot_revalidation_20260922',
    'dynamic_reference_20260922',
    'dynamic_failure_analysis_20260922',
]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def write_new(path,data):
    with path.open('x') as f:
        json.dump(data,f,indent=2);f.write('\n')

checks=[]
for name in PREVIOUS:
    p=HERE.parent/name/'manifest.json'
    old=json.loads(p.read_text())
    bad=[key for key,digest in old['files'].items() if sha(ROOT/key)!=digest]
    checks.append({'manifest':str(p.relative_to(ROOT)),'sha256':sha(p),'checked':len(old['files']),'mismatches':bad})
assert all(not c['mismatches'] for c in checks)
previous=json.loads((HERE/'preservation_before.json').read_text())
assert previous == checks
if (HERE/'preservation_after.json').exists():
    assert json.loads((HERE/'preservation_after.json').read_text()) == checks
else:
    write_new(HERE/'preservation_after.json',checks)

paths=[]
paths.extend(p for p in HERE.iterdir() if p.is_file() and p.name != 'manifest.json')
for name in ('dynamic_cycle_diagnostic_v1','dynamic_cycle_diagnostic_v2'):
    paths.extend(p for p in (WORK/'runs'/name).rglob('*') if p.is_file())
source=WORK/'mppi_cycle_diagnostic_v1'
paths.extend(p for p in source.iterdir() if p.is_file())
for sub in ('tests_ros','cost_test_ros','cost_test_ros_v2','probe_capture','controller_trace_smoke','cost_critic_trace_v2'):
    paths.extend(p for p in (source/sub).rglob('*') if p.is_file())
paths.extend(p for p in (source/'install/nav2_mppi_controller/lib').glob('*.so') if p.is_file())
files={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(paths))}
manifest={'schema':'rm_tdt_planner/mppi_cycle_diagnostic_evidence/v1',
          'scope':'Local source-built diagnostic, two phase-zero first cases; previous manifests preserved; no hardware or deployment acceptance.',
          'previous_manifest_entries_checked':sum(c['checked'] for c in checks),
          'self_hash_excluded':True,'files':files}
out=HERE/'manifest.json'
if out.exists():
    assert json.loads(out.read_text()) == manifest, 'frozen evidence changed'
else:
    write_new(out,manifest)
print('preserved',sum(c['checked'] for c in checks),'new files',len(files),'mismatches',sum(len(c['mismatches']) for c in checks))
