"""Original immutable map-server occupancy, no dynamic future truth."""
import pathlib,sys,json,yaml
from PIL import Image
p=pathlib.Path(sys.argv[1])
for scene in ('narrow','crossing'):
 cfg=yaml.safe_load((p/f'{scene}.yaml').read_text());im=Image.open(p/f'{scene}.pgm');w,h=im.size;pix=list(im.getdata());grid=[]
 for y in range(h):
  for x in range(w):
   v=pix[(h-1-y)*w+x];occ=(v if cfg.get('negate',0) else 255-v)/255;grid.append(254 if occ>cfg['occupied_thresh'] else 0 if occ<cfg['free_thresh'] else 255)
 (p/f'{scene}_static.json').write_text(json.dumps(dict(width=w,height=h,resolution=cfg['resolution'],origin=cfg['origin'][:2],grid=grid,scope='original immutable map-server occupancy; STVL current layer separate'),separators=(',',':'))+'\n')
