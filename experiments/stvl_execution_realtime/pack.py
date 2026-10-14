"""Losslessly compress this round only; compact Git evidence avoids old duplication."""
import pathlib,json,gzip,hashlib,tarfile,io
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/realtime_cache';OUT=ROOT/'experiments/stvl_execution_realtime/evidence'
def digest(f):
    h=hashlib.sha256()
    for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
manifest=json.loads((OUT/'compression_manifest.json').read_text()) if (OUT/'compression_manifest.json').exists() else []
for cohort in ('fresh','boundary_enabled','no_capture','corrected_scene'):
    for p in sorted((CACHE/cohort/'runs').glob('*')):
        for source in [*p.glob('*.jsonl'),*p.glob('*.csv')]:
            target=pathlib.Path(str(source)+'.gz');assert not target.exists()
            with source.open('rb') as a,gzip.open(target,'wb',compresslevel=6) as b:
                for block in iter(lambda:a.read(1024*1024),b''):b.write(block)
            with source.open('rb') as a,gzip.open(target,'rb') as b:old,new=digest(a),digest(b)
            assert old==new;manifest.append({'source':str(source.relative_to(ROOT)),'compressed':str(target.relative_to(ROOT)),'sha256':old,'bytes':source.stat().st_size,'compressed_bytes':target.stat().st_size});source.unlink()
        with tarfile.open(OUT/(cohort+'__'+p.name+'.tar.gz'),'w:gz') as tar:
            for f in p.iterdir():
                if not f.is_file() or f.name in ('control.jsonl.gz','prediction.jsonl.gz','execution_receipts.jsonl.gz'):continue
                if f.name=='native_boundary.csv.gz' and cohort in ('boundary_enabled','no_capture'):
                    # Invalid physical-model evidence: keep full stream locally;
                    # Git needs representative states plus original wrong SDF.
                    with gzip.open(f,'rt') as s:lines=s.readlines()
                    selected=lines[:1]+lines[1::20];data=gzip.compress(''.join(selected).encode());m=tarfile.TarInfo(cohort+'/'+p.name+'/native_boundary_sampled.csv.gz');m.size=len(data);tar.addfile(m,io.BytesIO(data));continue
                tar.add(f,arcname=cohort+'/'+p.name+'/'+f.name)
            compact=[]
            with gzip.open(p/'control.jsonl.gz','rt') as stream:
                for line in stream:
                    try:r=json.loads(line)
                    except json.JSONDecodeError:continue
                    if 'raw' in r:r.pop('raw');r['full_raw_retained_local']=True
                    for key in ('candidate_velocity','candidate_noised_controls'):r.get('root_diagnostics',{}).pop(key,None)
                    compact.append(json.dumps(r,separators=(',',':'))+'\n')
            data=gzip.compress(''.join(compact).encode());m=tarfile.TarInfo(cohort+'/'+p.name+'/control.jsonl.gz');m.size=len(data);tar.addfile(m,io.BytesIO(data))
with tarfile.open(OUT/'benchmark_records.tar.gz','w:gz') as tar:
    for directory in ('profile','benchmark','classification','updates','views','execution_geometry'):
        for p in sorted((CACHE/directory).glob('*.json')):tar.add(p,arcname=directory+'/'+p.name)
    for p in sorted((CACHE/'fresh/assets').glob('oracle_*.json')):tar.add(p,arcname='assets/'+p.name)
(OUT/'compression_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());assert size<=8*1024**2,'incremental evidence cap';print('verified complete owned streams',len(manifest),'Git evidence MiB',size/1024**2)
