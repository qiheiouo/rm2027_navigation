"""Reuse exact physical metric implementation; only change cache/output path."""
import pathlib,json,sys,gzip,hashlib,tarfile
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/brake_window_cache';OUT=ROOT/'experiments/stvl_brake_window/evidence';OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(ROOT/'experiments/stvl_lifecycle_hold'));import fixture_metrics as fm
fm.OUT=OUT
summary=[]
for label in ('brake_late04','brake_late08'):
 p=CACHE/label;r=fm.run(p);summary.append({k:v for k,v in r.items() if k!='gaps'})
 # Save complete, byte-verified gzip streams and retain a per-run manifest.
 manifest=json.loads((p/'compression_manifest.json').read_text()) if (p/'compression_manifest.json').exists() else []
 for f in list(p.glob('*.csv'))+list(p.glob('*.jsonl')):
  raw=f.read_bytes();z=gzip.compress(raw);assert gzip.decompress(z)==raw
  pathlib.Path(str(f)+'.gz').write_bytes(z);manifest.append(dict(source=f.name,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()));f.unlink()
 (p/'compression_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
selected=[p for label in ('brake_late04','brake_late08') for p in sorted((CACHE/label).iterdir()) if p.is_file()]+[p for p in CACHE.glob('adapter_*.json')]+[p for p in CACHE.glob('fixture_offset_*.py')]
with tarfile.open(OUT/'execution_complete.tar.gz','w:gz') as tf:
 for p in selected:tf.add(p,arcname=str(p.relative_to(CACHE)))
(OUT/'manifest.json').write_text(json.dumps([dict(path=str(p.relative_to(CACHE)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in selected],indent=2)+'\n')
assert sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())<2*1024**2
print(json.dumps([{k:r[k] for k in ('label','dynamic_min_gap_m','contact_count','wall_overlap_samples','prefix_comparison','profiles')} for r in summary]))
