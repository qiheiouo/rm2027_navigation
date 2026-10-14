// Exact existing inline validator, not a reimplementation or runtime adapter.
#include "model.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;namespace dyn=rm_dynamic_obstacle_critic;
void stamp(builtin_interfaces::msg::Time &t,int64_t ns){t.sec=ns/1000000000;t.nanosec=ns%1000000000;}
dyn::Array decode(const J&p){dyn::Array a;stamp(a.header.stamp,p.at("source_ns"));a.header.frame_id=p.at("frame");a.schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;a.authority=dyn::Array::AUTHORITY_SHADOW_ONLY;a.complete=p.at("complete");a.prediction_dt=p.at("dt");a.prediction_steps=30;
 for(const auto &x:p.at("tracks")){dyn::Obstacle t;t.track_id=x.at("id");t.state=x.at("state");stamp(t.last_observation_stamp,x.at("observed_ns"));t.position.x=x.at("position")[0];t.position.y=x.at("position")[1];t.velocity.x=x.at("velocity")[0];t.velocity.y=x.at("velocity")[1];t.size.x=x.at("size")[0];t.size.y=x.at("size")[1];a.tracks.push_back(t);}a.total_track_count=a.tracks.size();return a;}
int main(int argc,char**argv){if(argc!=2)return 2;std::ifstream file(argv[1]);if(!file.is_open()){std::cerr<<"input file missing\n";return 2;}J cases;file>>cases;if(!cases.is_array()||cases.size()!=7){std::cerr<<"expected seven registered cases\n";return 2;}J results=J::array();dyn::Limits limits;limits.input_schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;
 for(const auto &c:cases){auto original=decode(c.at("prediction"));auto projected=original;projected.tracks.clear();J dropped=J::array(),retained=J::array();for(const auto&t:original.tracks){if(t.state==dyn::Obstacle::STATE_TENTATIVE)dropped.push_back(t.track_id);else{projected.tracks.push_back(t);retained.push_back(t.track_id);}}projected.total_track_count=projected.tracks.size();double now=c.at("now_ns").get<int64_t>()*1e-9;
  auto before=dyn::validate(original,now,limits);auto after=dyn::validate(projected,now,limits);if(after.empty()&&projected.tracks.empty())after="active_subset_empty_unknown";
  results.push_back(J{{"case",c.at("case")},{"historical_reason",c.at("historical_reason")},{"before",before},{"projected_after",after},{"dropped_tentative_ids",dropped},{"unchanged_active_ids",retained},{"source_ns_unchanged",true},{"active_observation_ns_unchanged",true}});
 }
 bool ok=results[0]["before"]==""&&results[0]["projected_after"]==""&&results[1]["before"]=="observation_anchor_epoch"&&results[1]["projected_after"]==""&&results[2]["before"]=="observation_anchor_epoch"&&results[2]["projected_after"]==""&&results[3]["projected_after"]=="stale_observation"&&results[4]["projected_after"]=="observation_anchor_epoch"&&results[5]["projected_after"]=="active_subset_empty_unknown";
 std::cout<<J{{"cases",results},{"registered_conditions_pass",ok},{"scope","offline consumed-track projection; unchanged existing validator and thresholds; no control, tracking or time rewrite"}}.dump()<<'\n';return ok?0:3;
}
