// Standalone performance/semantic comparison. No ROS node or control mutation.
#include "allocation_probe.hpp"
#include "profile_reference.hpp"
#include "optimized.hpp"
#include <fstream>
#include <iostream>
#include <sys/resource.h>
using J=JointJson;using Clock=timing::Clock;
J counters(const timing::Stats&s){return {{"prepare_ms",s.preparation_ms},{"sweep_ms",s.sweep_ms},{"static_ms",s.static_ms},{"dynamic_ms",s.dynamic_ms},{"rate_ms",s.rate_ms},{"result_ms",s.result_ms},{"pose_samples",s.pose_samples},{"cell_visits",s.cell_visits},{"occupied_candidates",s.occupied_candidates},{"exact_box_calls",s.exact_box_calls}};}
J run_reference(const J&ctx,bool profile){
 allocation::reset();auto begin=Clock::now();auto map_begin=Clock::now();timing::Stats stats;timing::ProfileGrid grid(ctx.at("static_map"),stats);double map_ms=timing::elapsed(map_begin);J rows=J::array();
 for(const auto&t:ctx.at("trajectories")){auto r=profile?timing::diagnose_profiled(ctx,t,grid,stats):joint::diagnose(ctx,t,grid);r.erase("compute_ms");r["id"]=t.at("id");rows.push_back(r);}
 double total=timing::elapsed(begin);allocation::active=false;size_t count=allocation::count,bytes=allocation::bytes;
 return {{"total_ms",total},{"map_init_ms",map_ms},{"stages",counters(stats)},{"allocation_count",count},{"allocated_bytes",bytes},{"results",rows}};
}
void equal(const J&a,const J&b){if(a.size()!=b.size())throw std::runtime_error("size mismatch");for(size_t i=0;i<a.size();i++)for(auto it=a[i].begin();it!=a[i].end();++it){const auto&v=b[i].at(it.key());if(it->is_number_float()){if(std::abs(it->get<double>()-v.get<double>())>1e-12)throw std::runtime_error("numeric mismatch "+it.key());}else if(*it!=v)throw std::runtime_error("predicate mismatch "+it.key());}}
J run_fast(const J&ctx,bool profile,bool selected=false){
 allocation::reset();auto begin=Clock::now();auto prep_begin=Clock::now();prepared::Snapshot snapshot(ctx,selected);double preparation=timing::elapsed(prep_begin);timing::Stats stats;stats.preparation_ms=preparation;J rows=J::array();
 for(auto&t:snapshot.trajectories)rows.push_back(profile?prepared::diagnose<true>(snapshot,t,stats):prepared::diagnose<false>(snapshot,t,stats));
 double total=timing::elapsed(begin);allocation::active=false;size_t count=allocation::count,bytes=allocation::bytes;
 return {{"total_ms",total},{"snapshot_prepare_ms",preparation},{"stages",counters(stats)},{"allocation_count",count},{"allocated_bytes",bytes},{"results",rows}};
}
J run_classification(const J&ctx){
 allocation::reset();auto begin=Clock::now();prepared::Snapshot snapshot(ctx);double prep=timing::elapsed(begin);J rows=J::array();size_t visited=0;
 for(auto&t:snapshot.trajectories){auto r=prepared::classify(snapshot,t);visited+=r["visited_samples"].get<size_t>();rows.push_back(r);}
 double total=timing::elapsed(begin);allocation::active=false;size_t count=allocation::count,bytes=allocation::bytes;
 return {{"total_ms",total},{"snapshot_prepare_ms",prep},{"allocation_count",count},{"allocated_bytes",bytes},{"visited_samples",visited},{"results",rows}};
}
J native_comparison(const J&ctx){auto begin=Clock::now();joint::Grid g(ctx.at("static_map"));nav2_costmap_2d::Costmap2D map(g.w,g.h,g.r,g.x,g.y,0);for(unsigned y=0;y<g.h;y++)for(unsigned x=0;x<g.w;x++)map.setCost(x,y,g.cells[y*g.w+x]);nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);std::vector<geometry_msgs::msg::Point>fp;for(auto v:ctx.at("footprint")){geometry_msgs::msg::Point p;p.x=v[0];p.y=v[1];fp.push_back(p);}int hits=0;for(auto&t:ctx.at("trajectories"))for(auto p:t.at("poses"))hits+=checker.footprintCostAtPose(p[0],p[1],p[2],fp)==254;return {{"ms",timing::elapsed(begin)},{"pose_hits",hits}};}
int main(int argc,char**argv){try{
 if(argc<2)return 2;int repeats=argc>2?std::stoi(argv[2]):3;auto read_begin=Clock::now();std::ifstream f(argv[1]);J ctx;f>>ctx;double parse_ms=timing::elapsed(read_begin);
 if(argc>3&&std::string(argv[3])=="classify"){
  auto reference=run_reference(ctx,false),cold=run_classification(ctx);J runs=J::array(),witnesses;
  auto compare=[&](const J&r){if(r.size()!=reference["results"].size())throw std::runtime_error("classification size");for(size_t i=0;i<r.size();i++)for(auto key:{"joint_geometry","joint_complete"})if(r[i][key]!=reference["results"][i][key])throw std::runtime_error("classification mismatch "+std::string(key));};
  compare(cold["results"]);witnesses=cold["results"];cold.erase("results");
  for(int i=0;i<repeats;i++){auto r=run_classification(ctx);compare(r["results"]);r.erase("results");runs.push_back(r);}
  std::cout<<J{{"case",ctx["case"]},{"cold",cold},{"runs",runs},{"equality",true},{"predicate_witnesses",witnesses}}.dump()<<'\n';return 0;
 }
 auto cold=run_reference(ctx,false),fast_cold=run_fast(ctx,false);equal(cold["results"],fast_cold["results"]);J reference=J::array(),profiled=J::array(),fast=J::array(),fast_profiled=J::array(),single=J::array();
 for(int i=0;i<repeats;i++){
  // Alternate plain reference/optimized order, same snapshot and CPU.
  J r,p,b,bp,s;if(i%2){b=run_fast(ctx,false);r=run_reference(ctx,false);}else{r=run_reference(ctx,false);b=run_fast(ctx,false);}
  p=run_reference(ctx,true);bp=run_fast(ctx,true);s=run_fast(ctx,false,true);
  for(auto*row:{&r,&p,&b,&bp})equal(cold["results"],(*row)["results"]);equal(J::array({cold["results"].back()}),s["results"]);
  for(auto*row:{&r,&p,&b,&bp,&s})row->erase("results");reference.push_back(r);profiled.push_back(p);fast.push_back(b);fast_profiled.push_back(bp);single.push_back(s);
 }
 auto serialize_begin=Clock::now();auto serialized=cold["results"].dump();double serialization_ms=timing::elapsed(serialize_begin);J flags=cold["results"];cold.erase("results");fast_cold.erase("results");rusage usage{};getrusage(RUSAGE_SELF,&usage);
 J out={{"case",ctx.at("case")},{"parse_ms",parse_ms},{"trajectories",ctx.at("trajectories").size()},{"steps",ctx.at("trajectories")[0]["poses"].size()},{"vertices",ctx.at("footprint").size()},{"map_cells",ctx["static_map"]["grid"].size()},{"cold_reference",cold},{"cold_optimized",fast_cold},{"reference",reference},{"profiled",profiled},{"optimized",fast},{"optimized_profiled",fast_profiled},{"selected_only",single},{"native_offline_comparison",native_comparison(ctx)},{"serialization_ms",serialization_ms},{"serialized_bytes",serialized.size()},{"peak_RSS_kB",usage.ru_maxrss},{"profile_equality",true},{"optimization_equality",true},{"predicate_results",flags}};
 std::cout<<out.dump()<<'\n';
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
