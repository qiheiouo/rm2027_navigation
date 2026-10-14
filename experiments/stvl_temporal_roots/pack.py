"""Bounded incremental archives. Every attempt kept; selected matrices only."""
import pathlib,json,sys,gzip,hashlib,tarfile,io
from data import jsonlines,COHORTS
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'experiments/stvl_temporal'))
from analyze import stats
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_temporal_roots/evidence';DEST=OUT/'episodes';DEST.mkdir(exist_ok=True);index=[]
for cohort in COHORTS:
 for p in sorted((ROOT/'build/roots_cache'/cohort/'runs').glob('*')):
  if not (p/'events.json').exists():continue
  controls=jsonlines(p/'control.jsonl');captured=[r for r in controls if 'raw' in r];selected=set()
  critical=p.name in ('wait_release_C_29_aligned','stale_C_29_aligned','narrow_D_29_aligned','narrow_D_47_aligned')
  if cohort=='repair_odom' and (critical or p.name=='narrow_C_29_aligned'):selected={r['epoch_ns'] for r in captured[:1]}
  if cohort in ('heldout_abi','heldout_odom') and p.name=='narrow_D_61_aligned':selected={r['epoch_ns'] for r in captured[:1]}
  if cohort=='diag':selected={r['epoch_ns'] for r in captured[:1]}
  if cohort=='diag' and len(captured)>1:selected.add(captured[min(7,len(captured)-1)]['epoch_ns'])
  compact=[]
  for row in controls:
   r=dict(row)
   if r['epoch_ns'] not in selected:
    r.pop('raw',None)
    if 'critic_deltas' in r:r['critic_summary']={k:stats(v) for k,v in r.pop('critic_deltas').items()}
    if 'root_diagnostics' in r:
     extra=dict(r['root_diagnostics']);extra.pop('candidate_velocity',None);extra.pop('candidate_noised_controls',None)
     if 'cost_probe' in extra:probe=dict(extra['cost_probe']);probe.pop('grid',None);extra['cost_probe']=probe
     r['root_diagnostics']=extra
   compact.append(r)
  payload=('\n'.join(json.dumps(r,separators=(',',':')) for r in compact)+'\n').encode() if compact else b''
  target=DEST/(cohort+'__'+p.name+'.tar.gz')
  with target.open('wb') as f:
   with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:
    with tarfile.open(fileobj=z,mode='w') as tar:
     for source in sorted(p.iterdir()):
      if source.name.startswith('control.jsonl') or source.name=='record.log':continue
      if source.name.endswith('_inputs.json') and not critical and p.name!='narrow_D_61_aligned':continue
      if source.is_file():tar.add(source,arcname=source.name,recursive=False)
     info=tarfile.TarInfo('control.compact.jsonl');info.size=len(payload);info.mtime=0;tar.addfile(info,io.BytesIO(payload))
  raw=p/'control.jsonl';raw=raw if raw.exists() else pathlib.Path(str(raw)+'.gz')
  original=gzip.decompress(raw.read_bytes()) if raw.suffix=='.gz' else raw.read_bytes()
  index.append({'cohort':cohort,'case':p.name,'archive':str(target.relative_to(OUT)),'archive_bytes':target.stat().st_size,'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'full_control_path':str(raw),'full_control_sha256_decompressed':hashlib.sha256(original).hexdigest(),'full_control_bytes':len(original),'kept_matrix_epochs':sorted(selected),'compact_control_cycles':len(compact),'full_raw_unique':'complete owned source stream remains local; do not delete it. Archives retain all poses/commands/predictions/final-cost vectors and critic summaries; detailed critic vectors/maps/velocities at selected matrices. Noncritical ring inputs retained only in local full runs.'})
(OUT/'archive_index.json').write_text(json.dumps(index,indent=2)+'\n')
size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());assert size<=45*1024**2,f'incremental compact evidence exceeds budget: {size}'
print('archived',len(index),'attempts; total incremental evidence MiB',size/1024**2)
