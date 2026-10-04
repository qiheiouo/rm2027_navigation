import gzip,hashlib,json
from pathlib import Path
root=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004')
evidence=root/'docs/dynamic_navigation/evidence/temporal_mpc_timing_20261004'
m=json.loads((evidence/'manifest.json').read_text());errors=[];streams=[]
for f in m['files']:
 p=evidence/f['path'];raw=p.read_bytes()
 if len(raw)!=f['bytes'] or hashlib.sha256(raw).hexdigest()!=f['sha256']:errors.append(f['path']+': stored digest')
 if 'original_sha256' in f:
  h=hashlib.sha256();size=0;rows=0
  with gzip.open(p,'rb') as stream:
   for line in stream:
    h.update(line);size+=len(line);json.loads(line);rows+=1
  if h.hexdigest()!=f['original_sha256'] or size!=f['original_bytes']:errors.append(f['path']+': original digest')
  streams.append(dict(path=f['path'],rows=rows,uncompressed_bytes=size))
expected={x['path'] for x in m['files']}|{'manifest.json'}
actual={str(p.relative_to(evidence)) for p in evidence.rglob('*') if p.is_file()}
if expected!=actual:errors.append('manifest coverage differs')
s=json.loads((evidence/'source_identity.json').read_text())
source_ok=all(s[k] for k in ['physical_sources_identical','current_runtime_sources_match_physical_snapshot','same_prestart_binaries_and_dependencies','all_identities_prestart','nav2_engineering_binaries_and_dependencies_match_physical','control_core_unchanged_since_ca89b527','native_validate_body_unchanged_since_ca89b527'])
if not source_ok:errors.append('source identity checks')
for p,item in json.loads((evidence/'head_on_b0_01/runtime_identity.json').read_text())['binaries'].items():
 if hashlib.sha256((evidence/'binaries'/Path(p).name).read_bytes()).hexdigest()!=item['sha256']:errors.append('binary '+p)
lineage=json.loads((evidence/'prerequisite_lineage.json').read_text())
for item in lineage['old_age_inputs']+list(lineage['calibration_inputs'].values()):
 p=root/item['archive']
 if hashlib.sha256(p.read_bytes()).hexdigest()!=item['archive_sha256']:errors.append('prerequisite archive '+str(p))
 if p.suffix=='.gz':
  h=hashlib.sha256()
  with gzip.open(p,'rb') as stream:
   for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
  if h.hexdigest()!=item['original_sha256']:errors.append('prerequisite lossless original '+str(p))
result=dict(pass_=not errors,errors=errors,manifest_files=len(m['files']),stored_bytes_excluding_manifest=sum(f['bytes'] for f in m['files']),jsonl_streams=len(streams),jsonl_rows=sum(s['rows'] for s in streams),streams=streams,source_hash_groups_verified=source_ok,prestart_binary_copies_verified=not errors,old_input_lineage_verified=not errors,scope='Independent stored hashes, lossless complete gzip bytes, every JSON row, manifest coverage, frozen source/binary checks and committed prerequisite hashes. No performance/safety inference.')
result['pass']=result.pop('pass_')
(root/'docs/dynamic_navigation/temporal_mpc_timing_integrity_20261004.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='streams'},indent=2))
if errors:raise SystemExit(1)
