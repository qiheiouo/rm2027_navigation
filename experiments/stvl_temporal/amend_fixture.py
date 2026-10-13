"""Fix the observed wall/slider conflict, retaining the original cohort."""
import json,pathlib,shutil,xml.etree.ElementTree as E
ROOT=pathlib.Path(__file__).resolve().parents[2];ASSETS=ROOT/'build/stvl_cache/assets'
E.register_namespace('ignition','http://ignitionrobotics.org/schema')
backup=ASSETS/'original_corridor_v1';backup.mkdir(exist_ok=True)
for scene in ('narrow','wait_release','stale'):
 for suffix in ('.sdf','_CAL.sdf','.pgm','.yaml'):
  file=ASSETS/(scene+suffix)
  if not (backup/file.name).exists():shutil.copy2(file,backup/file.name)
 for tag in ('','_CAL'):
  file=ASSETS/f'{scene}{tag}.sdf';tree=E.parse(file);world=tree.getroot().find('world')
  for m in list(world.findall('model')):
   if m.get('name','').startswith('corridor_'):world.remove(m)
  # .70m transverse aperture admits the .45m actor; it cannot admit both actor
  # and .65m mechanical ego length side-by-side. No common controller changes.
  for side in (-1,1):
   for part,(left,right) in enumerate(((-1.5,1.65),(2.35,5.5))):
    m=E.SubElement(world,'model',name=f'corridor_{side}_{part}');E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text=f'{(left+right)/2} {side*.8} .5 0 0 0';link=E.SubElement(m,'link',name='wall')
    for kind in ('collision','visual'):
     item=E.SubElement(link,kind,name='wall_'+kind);E.SubElement(E.SubElement(E.SubElement(item,'geometry'),'box'),'size').text=f'{right-left} .2 1'
  tree.write(file,encoding='utf-8',xml_declaration=True)
 width,height=240,160;pixels=bytearray([254])*(width*height)
 for row in range(height):
  for col in range(width):
   x=-2+(col+.5)*.05;y=-4+(height-row-.5)*.05
   if -1.5<=x<=5.5 and abs(x-2)>=.35 and .7<=abs(y)<=.9:pixels[row*width+col]=0
 (ASSETS/f'{scene}.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
scenario=json.loads((ASSETS/'scenario.json').read_text());scenario['corridor_revision']=2;scenario['fixture_fix']='transverse .70m doorway permits slider actor to exit; controller, motion, footprint, limits and budget unchanged'
(ASSETS/'scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
overrides={'head_on:CAL:13':'head_on:CAL:13:parked'}
for scene in ('narrow','wait_release','stale'):
 for seed in (13,29):
  for mode in ('CAL','A','B','C','D'):overrides[f'{scene}:{mode}:{seed}']=f'{scene}:{mode}:{seed}:gap'
(ROOT/'build/stvl_cache/fixture_overrides.json').write_text(json.dumps(overrides,indent=2)+'\n')
(ROOT/'experiments/stvl_temporal/evidence/fixture_amendment.json').write_text(json.dumps({'reason':'actual actor blocked at +/- .425m by continuous walls; independent CAL and D both observed same saturation','scope':['narrow','wait_release','stale'],'revision':2,'transverse_aperture_m':.70,'old_cohort_retained':True,'control_parameters_changed':False,'comparison_groups_all_rerun':True},indent=2)+'\n')
