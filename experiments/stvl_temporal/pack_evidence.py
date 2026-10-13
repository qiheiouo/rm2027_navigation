"""Bounded semantic evidence; full local traces remain untouched until review."""
import pathlib,json,tarfile,hashlib,gzip,shutil,io
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/stvl_cache';OUT=ROOT/'experiments/stvl_temporal/evidence';DEST=OUT/'runs';DEST.mkdir(exist_ok=True)
full_raw={'crossing_A_13','crossing_B_13','crossing_C_13','crossing_D_13','sudden_stop_D_13','narrow_D_29_gap','narrow_D_29_aligned','narrow_D_47_aligned','wait_release_C_29_gap','wait_release_C_29_aligned','stale_C_29_aligned','head_on_C_29','empty_A_12','crossing_C_11'}
index=[]
for run in sorted((CACHE/'runs').iterdir()):
 if not (run/'events.json').exists():raise RuntimeError('incomplete run must be classified: '+run.name)
 event=json.loads((run/'events.json').read_text());metrics=json.loads((run/'metrics.json').read_text()) if (run/'metrics.json').exists() else {};target=DEST/(run.name+'.tar.gz');records=[];retained_raw=0;omitted_raw=0
 controls=[json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()] if (run/'control.jsonl').exists() else []
 last_raw=max((i for i,r in enumerate(controls) if 'raw' in r),default=-1)
 failed=bool(event['contacts']) or event['result_status']!=4 or not metrics.get('safe_success',False)
 files=sorted(p for p in run.iterdir() if p.is_file())
 with tarfile.open(target,'w:gz',compresslevel=9) as archive:
  for p in files:
   if p.name=='control.jsonl':
    for i,r in enumerate(controls):
     if 'raw' in r:
      if run.name in full_raw or (failed and i==last_raw):retained_raw+=1
      else:r.pop('raw');omitted_raw+=1
    data=('\n'.join(json.dumps(r,separators=(',',':')) for r in controls)+'\n').encode() if controls else b''
   else:data=p.read_bytes()
   info=tarfile.TarInfo(p.name);info.size=len(data);info.mtime=0;info.mode=0o644;archive.addfile(info,io.BytesIO(data));records.append((p.name,hashlib.sha256(data).hexdigest()))
 with tarfile.open(target,'r:gz') as archive:
  for name,digest in records:
   if hashlib.sha256(archive.extractfile(name).read()).hexdigest()!=digest:raise RuntimeError('archive mismatch '+name)
 index.append(dict(run=run.name,archive=target.name,bytes=target.stat().st_size,files=len(files),outcome=event['finished'],source_goal_ns=event['goal_ns'],retained_raw_snapshots=retained_raw,omitted_raw_snapshots=omitted_raw,full_raw_local_path=str(run/'control.jsonl')))
assetdir=OUT/'assets';assetdir.mkdir(exist_ok=True)
for p in sorted((CACHE/'assets').glob('oracle_*.json')):
 with (assetdir/(p.name+'.gz')).open('wb') as out:
  with gzip.GzipFile(filename='',fileobj=out,mode='wb',mtime=0,compresslevel=9) as zipped:zipped.write(p.read_bytes())
for name in ['schedule.json','schedule_aligned.json','fixture_overrides.json','batch.log','batch_resume.log','batch_gap.log','batch_aligned.log','build.log','build_p3.log','check_p3.log','check_clock_build.log']:
 p=CACHE/name
 if p.exists():shutil.copy2(p,OUT/name)
(OUT/'archive_index.json').write_text(json.dumps({'method':'all trial files preserved; control JSON semantically reserialized; raw matrices limited to listed diagnostic cases and last captured failure matrix. All commands, prediction payloads, aggregates and per-critic deltas remain; original full local traces not deleted','full_raw_cases':sorted(full_raw),'runs':index},indent=2)+'\n')
size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());assert size<=150*1024**2;print('trials',len(index),'compact_MiB',size/1024**2,'raw_snapshots',sum(r['retained_raw_snapshots'] for r in index))
