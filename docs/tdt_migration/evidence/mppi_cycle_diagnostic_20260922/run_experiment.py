"""New first-case diagnostic series; byte-identical profiles and frozen fixtures."""
from pathlib import Path
import json,hashlib,os,subprocess,sys
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];WORK=ROOT/'build/tdt_p2b';RUNS=WORK/'runs/dynamic_cycle_diagnostic_v2'
sys.path.insert(0,str(HERE.parent/'dynamic_reference_20260922'))
import audit_dynamic
ex=audit_dynamic.ex

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):
 with p.open('x') as f:json.dump(d,f,indent=2);f.write('\n')
def prepare():
 RUNS.mkdir(exist_ok=False);(RUNS/'profiles').mkdir()
 base=WORK/'runs/dynamic_reference_pilot_v2';m=json.loads((base/'inputs.json').read_text())
 for name,h in m['profiles'].items():
  p=base/'profiles'/f'{name}.yaml';assert sha(p)==h;(RUNS/'profiles'/p.name).write_bytes(p.read_bytes())
 m['source_commit']=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
 m['diagnostic_tools']={str(p.relative_to(ROOT)):sha(p) for p in HERE.iterdir() if p.suffix in ('.py','.sh','.hpp','.cpp','.patch') or p.name=='probe_CMakeLists.txt'}
 m['diagnostic_libraries']={str(p.relative_to(ROOT)):sha(p) for p in (WORK/'mppi_cycle_diagnostic_v1/install/nav2_mppi_controller/lib').glob('*.so')}
 assert len(m['diagnostic_libraries'])==2
 m['scope']='Instrumented Nav2 1.1.20 MPPI in separate prefix; passive observer; unchanged controls/profiles/phase/gates; not acceptance matrix.'
 write(RUNS/'inputs.json',m)
def run(name):
 m=json.loads((RUNS/'inputs.json').read_text());assert name in ('tdt_astar','tdt_qp')
 for block in ('files','diagnostic_tools','diagnostic_libraries'):
  for n,h in m[block].items():assert sha(ROOT/n)==h,n
 assert not subprocess.check_output(['git','diff','a419654','--','src','experiments'])
 target=RUNS/f'{name}_1';target.mkdir(exist_ok=False)
 profile=RUNS/'profiles'/f'{name}.yaml';assert sha(profile)==m['profiles'][name];(target/'profile.yaml').write_bytes(profile.read_bytes())
 image=subprocess.check_output(['docker','image','inspect','rm2027_navigation:humble','--format','{{.Id}}'],text=True).strip()
 assert image=='sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172'
 write(target/'metadata.json',{'planner':name,'trial':1,'source_commit':m['source_commit'],'runtime_source_commit':m['runtime_source_commit'],
    'image_id':image,'profile_sha256':m['profiles'][name],'diagnostic_libraries':m['diagnostic_libraries'],
    'scope':'Isolated same-phase dynamic diagnostic; not deployment acceptance','performance_is_not_algorithm_rejection':True})
 out='/work/'+str(target.relative_to(WORK))
 args=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}','--entrypoint','bash',
 '-v',f'{ROOT}:/ws:ro','-v',f'{WORK}:/work','--cidfile',str(target/'container_id')]
 for e in ['ROS_DOMAIN_ID=174','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1','TMPDIR=/work/tmp','LIBGL_ALWAYS_SOFTWARE=true','QT_QPA_PLATFORM=offscreen','TDT_HEADING_AB=1','TDT_PHASE_SECONDS=0',f'IGN_PARTITION=tdt_dynamic_cycle_v2_{name}']:args+=['-e',e]
 args+=['rm2027_navigation:humble','/ws/'+str(HERE.relative_to(ROOT))+'/run_trial.sh',out,out+'/profile.yaml']
 status=subprocess.call(args);(target/'docker_exit.txt').write_text(str(status)+'\n')
 print('docker exit',status,'at',target,flush=True)
 if not (target/'runtime_geometry_preflight.json').is_file():return 2
 s=ex.analyze(target,m)
 print(json.dumps({k:s[k] for k in ('planner','evidence_valid','limited_dynamic_geometry_and_goal_pass','action_status','recoveries','checks')},indent=2))
 return 0 if s['limited_dynamic_geometry_and_goal_pass'] else 1
if __name__=='__main__':
 if sys.argv[1]=='prepare':prepare()
 elif sys.argv[1]=='run':sys.exit(run(sys.argv[2]))
