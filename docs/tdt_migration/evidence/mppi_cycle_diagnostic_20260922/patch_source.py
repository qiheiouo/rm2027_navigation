"""Apply only observer insertions to the independently pinned MPPI package."""
from pathlib import Path
import difflib,hashlib,json,sys
here=Path(__file__).resolve().parent
source=Path(sys.argv[1]);up=json.loads((here/'upstream.json').read_text())
for name,h in up['files'].items():assert hashlib.sha256((source/name).read_bytes()).hexdigest()==h,name
pkg=source/'nav2_mppi_controller';before={str(p.relative_to(pkg)):p.read_text() for p in pkg.rglob('*') if p.is_file() and p.suffix in ('.cpp','.hpp','.txt')}
def insert(file,old,new):
 p=pkg/file;s=p.read_text();assert s.count(old)==1,(file,old,s.count(old));p.write_text(s.replace(old,new))
for name in ('controller','optimizer','critic_manager','critics/cost_critic'):
 p=pkg/'src'/f'{name}.cpp';s=p.read_text();p.write_text('#include "nav2_mppi_controller/cycle_trace.hpp"\n'+s)
insert('src/controller.cpp','  optimizer_.shutdown();','  optimizer_.shutdown();\n  tdt_trace::shutdown();')
insert('src/controller.cpp','  nav2_costmap_2d::Costmap2D * costmap =','  tdt_trace::Cycle trace(robot_pose, robot_speed, clock_);\n  nav2_costmap_2d::Costmap2D * costmap =')
insert('src/controller.cpp','  geometry_msgs::msg::TwistStamped cmd =','  trace.locked(costmap_ros_, transformed_plan);\n  geometry_msgs::msg::TwistStamped cmd =')
insert('src/controller.cpp','  return cmd;','  trace.output(cmd);\n  return cmd;')
insert('src/optimizer.cpp','  prepare(robot_pose, robot_speed, plan, goal_checker);','  prepare(robot_pose, robot_speed, plan, goal_checker);\n  tdt_trace::value("settings", {{"dt", settings_.model_dt}, {"offset", settings_.shift_control_sequence ? 1 : 0}, {"temperature", settings_.temperature}, {"gamma", settings_.gamma}});')
insert('src/optimizer.cpp','  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);','  tdt_trace::controls("before_filter", control_sequence_);\n  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);\n  tdt_trace::controls("after_filter", control_sequence_);')
insert('src/optimizer.cpp','    updateControlSequence();','    tdt_trace::scored(critics_data_);\n    updateControlSequence();\n    tdt_trace::tensor("weighted.costs", costs_);\n    tdt_trace::controls("after_update", control_sequence_);')
insert('src/critic_manager.cpp','    critics_[q]->score(data);','    critics_[q]->score(data);\n    tdt_trace::tensor("critic." + critics_[q]->getName(), data.costs);\n    tdt_trace::value("critic.fail", data.fail_flag);')
insert('src/critics/cost_critic.cpp','  bool all_trajectories_collide = true;','  std::vector<uint8_t> trace_collisions(tdt_trace::active() ? data.trajectories.x.shape(0) : 0);\n  bool all_trajectories_collide = true;')
insert('src/critics/cost_critic.cpp','    if (!trajectory_collide) {','    if (!trace_collisions.empty()) trace_collisions[i] = trajectory_collide;\n    if (!trajectory_collide) {')
insert('src/critics/cost_critic.cpp','  data.costs += xt::pow','  if (tdt_trace::active()) tdt_trace::collision_mask(trace_collisions, near_goal);\n  data.costs += xt::pow')
insert('CMakeLists.txt','  src/controller.cpp','  src/cycle_trace.cpp\n  src/controller.cpp')
insert('CMakeLists.txt','set(libraries mppi_controller mppi_critics)','target_link_libraries(mppi_critics mppi_controller)\nset(libraries mppi_controller mppi_critics)')
(pkg/'src/cycle_trace.cpp').write_bytes((here/'cycle_trace.cpp').read_bytes());(pkg/'include/nav2_mppi_controller/cycle_trace.hpp').write_bytes((here/'cycle_trace.hpp').read_bytes())
patch=[]
for name,old in before.items():
 new=(pkg/name).read_text()
 if new!=old:patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
(here/'observer.patch').write_text(''.join(patch))
print('Applied observer additions; original computation statements retained.')
