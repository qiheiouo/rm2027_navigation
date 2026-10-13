"""Scenario intent only. Never imported by the CV/ZOH adapter."""
def target(scene,t,seed):
 t=max(0.,t+{11:0,13:-.2,29:0,47:.2}.get(seed,0))
 if scene=='crossing':return min(2.5,-1.8+.6*max(0.,t-1.))
 if scene=='head_on':return max(-4.8,-.35*t)
 if scene=='narrow':return min(2.5,-1.4+.3*max(0.,t-1.))
 if scene in ('wait_release','stale'):return min(2.5,-1.2+.6*min(max(0.,t-1.),2)+.6*max(0.,t-7.))
 if scene=='sudden_stop':return -1.5+.6*min(max(0.,t-1.),2.5)
 return 0.
