"""One scheduling insertion into byte-fixed previous fixture; no copied controller."""
import pathlib,subprocess,sys,hashlib,json
offset=float(sys.argv[1]);assert offset in (.4,.8)
ROOT=pathlib.Path(__file__).resolve().parents[2]
source=subprocess.check_output(['git','show','055f29beb74c3df7eaf7cba9e659e2c80e9371dc:experiments/stvl_lifecycle_hold/fixture.py'],cwd=ROOT,text=True)
line='brake_trigger_s=(float(np.interp(probe_pub,nt,sim))-goal)/1e9'
assert source.count(line)==1
adapted=source.replace(line,line+'\n brake_trigger_s+='+repr(offset))
cache=pathlib.Path('/check');(cache/('fixture_offset_'+str(offset)+'.py')).write_text(adapted)
(cache/('adapter_'+str(offset)+'.json')).write_text(json.dumps(dict(offset_s=offset,frozen_sha256=hashlib.sha256(source.encode()).hexdigest(),change='only add fixed scheduling offset after original DDS-sim anchor'),indent=2)+'\n')
sys.argv=[str(ROOT/'experiments/stvl_lifecycle_hold/fixture.py'),'brake']
exec(compile(adapted,sys.argv[0],'exec'))
