"""Selected small Git evidence; unique full raw remains losslessly in own cache."""
import pathlib,tarfile,json,io,gzip,csv,sys,hashlib,shutil
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/c83_cache';OUT=ROOT/'experiments/stvl_c83_endtoend/evidence'

def add_bytes(t,name,data):
 info=tarfile.TarInfo(name);info.size=len(data);info.mtime=0;t.addfile(info,io.BytesIO(data))
def pack_runs():
 summaries=[]
 for p in sorted(CACHE.glob('*/runs/*')):
  cohort=p.parent.parent.name
  with tarfile.open(OUT/f'{cohort}.tar.gz','w:gz') as t:
   for f in sorted(p.iterdir()):
    if not f.is_file() or '.raw' in f.name or f.name.startswith('first_') or f.name in ('prediction.jsonl.gz','execution_receipts.jsonl.gz'):continue
    if f.name=='control.jsonl.gz':
     data=[]
     for line in gzip.open(f,'rt'):
      j=json.loads(line);j.pop('critic_deltas',None);j.pop('native_final_costs_after_gamma',None);checks=j.pop('shadow_checks',[]);j['archived_shadow_counts']={flag:sum(q['id'].startswith('raw_') and q['joint_geometry']==flag for q in checks) for flag in ('Safe','Unknown','Unsafe')};j['archived_selected_check']=next((q for q in checks if q['id']=='selected_previous_yaw'),None);data.append(json.dumps(j,separators=(',',':')))
     add_bytes(t,'control.jsonl.gz',gzip.compress(('\n'.join(data)+'\n').encode(),mtime=0));continue
    if f.name=='metrics.json':
     j=json.loads(f.read_text());j.pop('snapshots',None);add_bytes(t,f.name,(json.dumps(j,separators=(',',':'))+'\n').encode());continue
    if f.name=='native_boundary.csv.gz' and cohort!='shadow_durable':
     with gzip.open(f,'rt') as a:
      reader=csv.DictReader(a);s=io.StringIO();w=csv.DictWriter(s,fieldnames=reader.fieldnames);w.writeheader();last=-1
      for row in reader:
       ns=int(row['sim_ns'])
       if last<0 or ns-last>=20000000:w.writerow(row);last=ns
     add_bytes(t,'native_boundary_20ms.csv.gz',gzip.compress(s.getvalue().encode(),mtime=0));continue
    t.add(f,arcname=f.name)
  summaries.append(dict(cohort=cohort,archive=f'{cohort}.tar.gz',source=str(p),full_binary_candidates='preserved in local cache, not duplicated into Git',native='full 1ms' if cohort=='shadow_durable' else '20ms representative'))
 (OUT/'archive_index.json').write_text(json.dumps(summaries,indent=2)+'\n')
if __name__=='__main__':
 p=OUT/'gate_results.json';j=json.loads(p.read_text());[x['metrics'].pop('snapshots',None) for x in j];p.write_text(json.dumps(j,separators=(',',':'))+'\n')
 pack_runs()
 for path in list(OUT.glob('*_paired_cycles.json'))+list(OUT.glob('*_causal.json')):
  data=path.read_bytes();z=gzip.compress(data,mtime=0);assert gzip.decompress(z)==data;pathlib.Path(str(path)+'.gz').write_bytes(z);path.unlink()
 size=sum(x.stat().st_size for x in OUT.rglob('*') if x.is_file());print('evidence bytes',size);assert size<=6*1024**2,'selected evidence budget; preserve local raw and compact derived duplication'
