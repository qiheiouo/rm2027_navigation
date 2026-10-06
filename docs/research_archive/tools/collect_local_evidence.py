#!/usr/bin/env python3
"""Archive management only: copy selected immutable evidence; never run trials."""
import argparse, collections, gzip, hashlib, json, os, pathlib, re, shutil, subprocess

TEXT_SUFFIXES = {'.json', '.jsonl', '.csv', '.tsv', '.md', '.txt', '.log', '.yaml', '.yml', '.xml', '.sh', '.py', '.sdf', '.rviz', '.xacro', '.env'}
DATA_SUFFIXES = {'.png', '.svg', '.pgm', '.pdf', '.db3'}
SKIP_PARTS = {'CMakeFiles', 'Testing', '__pycache__', 'research_install', 'plugin_build', 'install', 'log', 'logs', 'ros_logs', 'sdk_build', 'upstream', 'workspace', '.git', 'dependencies', 'sanitized_source', 'osqp', 'OSQP', 'ros_log', 'default_configuration', 'ament_cmake_environment_hooks', 'digest_new', 'digest_old', 'downstream', 'release', 'sanitized'}
PATTERNS = [
 ('private_key', re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----\r?\n[A-Za-z0-9+/=\r\n]{40}')),
 ('github_token', re.compile(rb'gh[pousr]_[A-Za-z0-9]{32,}')),
 ('aws_key', re.compile(rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b')),
 ('credential_url', re.compile(rb'https?://[^\s/:@]{1,64}:[^\s/@]{8,}@')),
 ('credential_literal', re.compile(rb'(?i)(?:access_token|api_key|apikey|client_secret|password|passwd)\s*[=:]\s*["\']([^"\'\r\n]{12,})["\']')),
]
def scan(data, label, findings):
 for detector, pattern in PATTERNS:
  for match in pattern.finditer(data):
   if detector == 'credential_literal' and any(v in match.group(1).lower() for v in [b'example', b'placeholder', b'password', b'not-', b'${', b'<', b'getenv', b'os.environ']): continue
   findings.append({'path': label, 'detector': detector, 'byte_offset': match.start()})

def selected(path):
 if path.name in {'CMakeCache.txt', 'compile_commands.json', 'install_manifest.txt', 'CMakeLists.txt'}: return False, 'regenerable CMake build bookkeeping'
 if path.name in {'intake.json', 'provenance.json', 'sources.json', 'dependencies.json'}: return True, ''
 for part in path.parts[:-1]:
  if part in SKIP_PARTS or part == 'build' or part.endswith('_build') or part.endswith('_install'): return False, 'regenerable build/install/dependency/duplicate ROS logging'
 if path.name.startswith('core') and path.suffix not in {'.py', '.json'}: return False, 'core dump excluded'
 if path.suffix in TEXT_SUFFIXES | DATA_SUFFIXES: return True, ''
 return False, 'binary/cache or unsupported payload retained locally; source/summary preferred'

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--repository', required=True, type=pathlib.Path); ap.add_argument('--archive-root', type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1]); a=ap.parse_args()
 repo=a.repository.resolve(); archive=a.archive_root.resolve(); objects=archive/'data/objects'; objects.mkdir(parents=True, exist_ok=True)
 r4=repo/'build/r4_hws_prediction_consumption'; surface=repo/'build/dynamic_surface_recovery'
 groups=[('R4/'+p.name,p) for p in sorted((r4/'build').glob('r4_*')) if p.is_dir()]
 groups += [('surface/'+p.name,p) for p in sorted(surface.iterdir()) if p.is_dir() and p.name!='sdk_build']
 groups += [('surface/control_records',surface)]
 records=[]; excluded=[]; findings=[]; totals=collections.defaultdict(lambda:dict(files=0, original_bytes=0))
 seen=set()
 for group,source in groups:
  files=sorted(source.rglob('*')) if group!='surface/control_records' else sorted(source.iterdir())
  for f in files:
   if not f.is_file() or f.is_symlink(): continue
   label=f.relative_to(repo).as_posix()
   if label in seen: continue
   seen.add(label); rel=f.relative_to(source); keep,reason=selected(rel)
   if not keep:
    excluded.append(dict(source=label,bytes=f.stat().st_size,reason=reason));continue
   data=f.read_bytes(); scan(data,label,findings); digest=hashlib.sha256(data).hexdigest(); dest=objects/(digest+'.gz')
   if not dest.exists():
    with dest.open('wb') as out:
     with gzip.GzipFile(filename='',mode='wb',fileobj=out,mtime=0,compresslevel=9) as gz: gz.write(data)
   records.append(dict(group=group,source=label,source_relative_to_group=rel.as_posix(),sha256=digest,bytes=len(data),object=dest.relative_to(archive).as_posix(),encoding='gzip',stored_bytes=dest.stat().st_size))
   totals[group]['files']+=1;totals[group]['original_bytes']+=len(data)
  print(group,totals[group]['files'],round(totals[group]['original_bytes']/2**20,2),flush=True)
 manifest=dict(schema='rm2027.research_archive.objects.v1',source_repository=str(repo),scope='Lossless selected original records; generated dependencies/binaries excluded, omissions enumerated. Full per-trial analytical streams for A23-A26; A13/A16 include original ROS bags.',records=records,groups=dict(totals),unique_objects=len({r['object'] for r in records}),original_bytes=sum(v['bytes'] for v in records),stored_unique_bytes=sum((archive/p).stat().st_size for p in {r['object'] for r in records}),credential_findings=findings)
 (archive/'evidence_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
 (archive/'local_only_exclusions.json').write_text(json.dumps(excluded,ensure_ascii=False,indent=2)+'\n')
 print('TOTAL',len(records),manifest['unique_objects'],round(manifest['stored_unique_bytes']/2**20,2),'MiB; credential finding count',len(findings),flush=True)
 if findings: raise SystemExit('Review credential findings before publishing; values intentionally not logged.')
if __name__=='__main__': main()
