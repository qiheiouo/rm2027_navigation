#!/usr/bin/env python3
"""Small safety copy of valuable historical code/docs; never rewrites Git history."""
import argparse, gzip, hashlib, importlib.util, json, pathlib, subprocess
KEEP = {'.py','.cpp','.cc','.c','.h','.hpp','.md','.rst','.yaml','.yml','.xml','.sh','.cmake','.rviz','.xacro','.txt'}
EVIDENCE_NAMES = {'analysis.json','summary.json','assessment.json','protocol.json','provenance.json','manifest.json','freeze.json','results.json','result.json','trials.csv','comparison.csv'}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--repository',type=pathlib.Path,required=True);a=ap.parse_args();repo=a.repository.resolve();arc=pathlib.Path(__file__).resolve().parents[1];objects=arc/'data/objects';audit=json.loads((arc/'git_audit.json').read_text());spec=importlib.util.spec_from_file_location('scan',arc/'tools/collect_local_evidence.py');scanner=importlib.util.module_from_spec(spec);spec.loader.exec_module(scanner)
 def git(*args):return subprocess.check_output(['git',*args],cwd=repo)
 branches=[];by_oid={};findings=[];before={pathlib.Path(r['object']).name for r in json.loads((arc/'evidence_manifest.json').read_text())['records']};omitted=[]
 for b in audit['branches']:
  if b['branch'] in ['main','experiment/r4-hws-prediction-consumption']:continue
  changes={v.decode() for v in git('diff','--name-only','main',b['commit'],'--').splitlines()};entries=[]
  for line in git('ls-tree','-rl',b['commit']).splitlines():
   metadata,path=line.split(b'\t',1);mode,typ,oid,size=metadata.split();path=path.decode()
   if path not in changes or typ!=b'blob':continue
   size=int(size);p=pathlib.PurePosixPath(path)
   if any(part in {'third_party','thirdparty','vendor','vendored','external','upstream','.git'} for part in p.parts):continue
   keep=p.suffix in KEEP or p.name in EVIDENCE_NAMES or (p.suffix in {'.png','.svg'} and size<1024*1024)
   if not keep or size>1024*1024:
    omitted.append(dict(branch=b['branch'],commit=b['commit'],path=path,bytes=size,reason='Not in small source/config/report snapshot; preserved in original Git/local evidence'))
    continue
   oid=oid.decode()
   if oid not in by_oid:
    data=git('cat-file','blob',oid);scanner.scan(data,path,findings);sha=hashlib.sha256(data).hexdigest();dest=objects/(sha+'.gz')
    if not dest.exists():
     with dest.open('wb') as out:
      with gzip.GzipFile(filename='',fileobj=out,mode='wb',mtime=0,compresslevel=9) as gz:gz.write(data)
    by_oid[oid]=dict(git_blob=oid,sha256=sha,bytes=size,object=dest.relative_to(arc).as_posix(),stored_bytes=dest.stat().st_size)
   entries.append(dict(path=path,git_blob=oid,mode=mode.decode()))
  branches.append(dict(branch=b['branch'],commit=b['commit'],base_comparison='main',scope='Selected changed source/config/report files <=1MiB; partial tree copy, not original commit history or full raw record',files=entries));print(b['branch'],len(entries),flush=True)
 new=[p for p in objects.glob('*.gz') if p.name not in before];result=dict(schema='rm2027.research_archive.historical_snapshots.v1',branches=branches,objects=list(by_oid.values()),new_unique_objects=len(new),added_compressed_bytes=sum(p.stat().st_size for p in new),credential_findings=findings)

 for name,payload in [('historical_snapshots.json.gz',result),('historical_snapshot_omissions.json.gz',omitted)]:
  with (arc/name).open('wb') as out:
   with gzip.GzipFile(filename='',fileobj=out,mode='wb',mtime=0,compresslevel=9) as gz:gz.write((json.dumps(payload,ensure_ascii=False,indent=2)+'\n').encode())
 print('added objects',len(new),'MiB',result['added_compressed_bytes']/2**20,'findings',len(findings),flush=True)
 if findings:raise SystemExit('credential locations require review before publishing')
if __name__=='__main__':main()
