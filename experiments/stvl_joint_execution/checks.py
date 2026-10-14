"""Small decisive geometry/time/feasibility fixtures, not a navigation test suite."""
import pathlib,json,math,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[2];cache=ROOT/'build/joint_cache';inputs=cache/'unit_inputs';inputs.mkdir(exist_ok=True)
fp=[[-.34,-.29],[.34,-.29],[.34,.29],[-.34,.29]]
def context(name):
 source={'schema':'rm_time_prediction/research_v1','frame':'map','complete':True,'source_ns':10_000_000_000,'dt':.1,'tracks':[]}
 return {'case':name,'pose':[0,0,0],'speed':[0,0,0],'pose_epoch_ns':10_000_000_000,'dt':.1,'identity_frame_fixture':True,'feedback_consistent':True,'execution_mapping_known':True,'footprint':fp,'prediction':{'source':source,'consume_ns':10_000_000_000},'static_map':{'width':120,'height':120,'resolution':.05,'origin':[-3,-3],'grid':[0]*14400},'trajectories':[{'id':'wait','poses':[[0,0,0]]*30,'velocities':[[0,0,0]]*30}]}
fixtures=[]
x=context('known_safe_wait');fixtures.append((x,{'joint_complete':'Safe'}))
x=context('interior_cell_missed_by_perimeter');x['static_map']['grid'][62*120+63]=254;fixtures.append((x,{'static':'Unsafe','native_same_static_grid_discrete_hits':0}))
x=context('dynamic_between_discrete_points');q=.379/math.sqrt(2);a=.15/math.sqrt(2);points=[[.34+q-a+3/math.sqrt(2)*k*.1,.29+q+a-3/math.sqrt(2)*k*.1] for k in range(36)];x['prediction']['source']['tracks']=[{'id':1,'state':2,'size':[.1,.1],'observed_ns':10_000_000_000,'points':points}];fixtures.append((x,{'dynamic':'Unsafe'}))
x=context('instant_zero_from_moving_is_not_a_stop_certificate');x['speed']=[.5,0,0];x['execution_mapping_known']=False;fixtures.append((x,{'nominal_rate':'Unsafe','mapped_execution':'Unknown'}))
x=context('bounded_braking_reference');x['speed']=[.5,0,0];vel=[max(0,.5-.08*k) for k in range(30)];pos=0.;poses=[]
for v in vel:pos+=v*.1;poses.append([pos,0,0])
x['trajectories']=[{'id':'bounded_brake','poses':poses,'velocities':[[v,0,0] for v in vel]}];fixtures.append((x,{'joint_complete':'Safe'}))
x=context('future_prediction_unknown');x['prediction']['source']['source_ns']=11_000_000_000;fixtures.append((x,{'dynamic':'Unknown'}))
x=context('unknown_static_support');x['static_map']['grid'][62*120+63]=255;fixtures.append((x,{'static':'Unknown'}))
x=context('uncovered_past_prefix');x['prediction']['source']['source_ns']=10_010_000_000;x['prediction']['consume_ns']=10_020_000_000;fixtures.append((x,{'dynamic':'Unknown'}))
for x,expected in fixtures:(inputs/(x['case']+'.json')).write_text(json.dumps(x,separators=(',',':'))+'\n')
(cache/'unit_expectations.json').write_text(json.dumps({x['case']:v for x,v in fixtures},indent=2)+'\n')
print('prepared',len(fixtures),'decisive fixtures')
