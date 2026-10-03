import subprocess,sys,os
from pathlib import Path
root=Path('/tmp/rm_surface_reveal_initial');os.environ['MPLCONFIGDIR']=str(root/'mpl_config')
script=Path('/home/qihei/rm2027_navigation/src/rm_dynamic_obstacle_critic/tools/plot_surface_reveal.py')
for mode in ['visible','nominal']:
 with (root/(mode+'_plot.log')).open('x') as stream:subprocess.run([sys.executable,'-B',str(script),str(root/(mode+'.json')),str(root/(mode+'_reveal')),'--title',f'V2 {mode}: sensor reveal labels; actual braking/replanning margin UNKNOWN'],check=True,stdout=stream,stderr=subprocess.STDOUT)
