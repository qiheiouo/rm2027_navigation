"""Lossless compression of this round only, compact text evidence, no old deletion."""
import pathlib,json,gzip,hashlib,tarfile,io
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/joint_cache';OUT=ROOT/'experiments/stvl_joint_execution/evidence'
def digest(stream):
    h=hashlib.sha256()
    for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def main():
    manifest=json.loads((OUT/'compression_manifest.json').read_text()) if (OUT/'compression_manifest.json').exists() else []
    for cohort in ('fresh','observer_retry'):
        for p in sorted((CACHE/cohort/'runs').glob('*')):
            for source in p.glob('*.jsonl'):
                target=source.with_suffix(source.suffix+'.gz');assert not target.exists(),target
                with source.open('rb') as a,gzip.open(target,'wb',compresslevel=6) as b:
                    for chunk in iter(lambda:a.read(1024*1024),b''):b.write(chunk)
                with source.open('rb') as a,gzip.open(target,'rb') as b:old,new=digest(a),digest(b)
                assert old==new;manifest.append({'source':str(source.relative_to(ROOT)),'compressed':str(target.relative_to(ROOT)),'bytes':source.stat().st_size,'compressed_bytes':target.stat().st_size,'sha256':old});source.unlink()
            archive=OUT/(cohort+'__'+p.name+'.tar.gz')
            with tarfile.open(archive,'w:gz') as tar:
                # All trials include compact configuration/commands/feedback/errors,
                # all control periods, bounded raw, truth/prediction and receipts.
                # Launch logs and complete failure/trace files also remain local.
                names=['nav2.yaml','manifest.json','events.json','supervisor.json','scene.sdf','map.pgm','map.yaml','scenario.json','commands.csv','odometry.csv','costmap.csv','metrics.json','truth.jsonl.gz','prediction.jsonl.gz','plans.jsonl.gz','output_endpoints.txt','controller_odom_contract.txt','execution_endpoints.jsonl.gz','execution_receipts.jsonl.gz','first_native_failure.json','first_native_failure_inputs.json','first_failure_inputs.json','tap.log','launch.log','record.log','supervisor_error.txt']
                for name in names:
                    if (p/name).exists():tar.add(p/name,arcname=cohort+'/'+p.name+'/'+name)
                # Derived compact control retains every period, selected sequence,
                # predictions and costs. Complete raw remains in local .gz unchanged.
                compact=[]
                with gzip.open(p/'control.jsonl.gz','rt') as f:
                    for line in f:
                        try:r=json.loads(line)
                        except json.JSONDecodeError:continue
                        if 'raw' in r:r.pop('raw');r['raw_capture_in_full_local_cache']=True
                        ex=r.get('root_diagnostics',{})
                        for key in ('candidate_velocity','candidate_noised_controls'):ex.pop(key,None)
                        compact.append(json.dumps(r,separators=(',',':'))+'\n')
                data=gzip.compress(''.join(compact).encode());member=tarfile.TarInfo(cohort+'/'+p.name+'/control.jsonl.gz');member.size=len(data);tar.addfile(member,io.BytesIO(data))
    with tarfile.open(OUT/'diagnostic_snapshots.tar.gz','w:gz') as tar:
        selected_history=('diag__narrow_D_29_aligned','heldout_abi__narrow_D_61_aligned','repair_odom__wait_release_C_13_aligned')
        selected_fresh=('fresh__crossing_B_29_aligned__6823000000','fresh__wait_release_C_13_aligned__6769000000')
        for name in ('inputs','results','fresh_inputs','fresh_diagnosis','unit_inputs','unit_results'):
            for p in sorted((CACHE/name).glob('*.json')):
                if name=='inputs' and p.stem not in selected_history:continue
                if name=='fresh_inputs' and p.stem not in selected_fresh:continue
                tar.add(p,arcname=name+'/'+p.name)
        for name in ('unit_expectations.json',):tar.add(CACHE/name,arcname=name)
    with tarfile.open(OUT/'oracle_assets.tar.gz','w:gz') as tar:
        for p in sorted((CACHE/'fresh/assets').glob('oracle_*.json')):tar.add(p,arcname='assets/'+p.name)
    (OUT/'compression_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    assert sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())<=12*1024**2,'incremental evidence limit'
    print('losslessly compressed',len(manifest),'owned streams; evidence MiB',sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())/1024**2)
if __name__=='__main__':main()
