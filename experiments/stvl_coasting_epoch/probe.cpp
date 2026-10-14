// Local pure validator contract; no tracking, planning, output or confidence reset.
#ifdef COASTING_EQUAL_ALLOWED
#include "model_coasting.hpp"
#else
#include "model.hpp"
#endif
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;namespace dyn=rm_dynamic_obstacle_critic;
void stamp(builtin_interfaces::msg::Time&t,int64_t ns){t.sec=ns/1000000000;t.nanosec=ns%1000000000;}
dyn::Array decode(const J&p){dyn::Array a;stamp(a.header.stamp,p.at("source_ns"));a.header.frame_id=p.at("frame");a.schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;a.authority=dyn::Array::AUTHORITY_SHADOW_ONLY;a.complete=p.at("complete");a.prediction_dt=p.at("dt");a.prediction_steps=30;
 for(const auto&x:p.at("tracks")){dyn::Obstacle t;t.track_id=x.at("id");t.state=x.at("state");stamp(t.last_observation_stamp,x.at("observed_ns"));t.position.x=x.at("position")[0];t.position.y=x.at("position")[1];t.velocity.x=x.at("velocity")[0];t.velocity.y=x.at("velocity")[1];t.size.x=x.at("size")[0];t.size.y=x.at("size")[1];a.tracks.push_back(t);}a.total_track_count=a.tracks.size();return a;}
int main(int argc,char**argv){if(argc!=2)return 2;std::ifstream f(argv[1]);if(!f.is_open())return 2;J cases;f>>cases;if(!cases.is_array()||cases.size()!=8)return 2;dyn::Limits limits;limits.input_schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;J rows=J::array();bool pass=true;
#ifdef COASTING_EQUAL_ALLOWED
 const std::string expected_key="expected_after",variant="equal_allowed";
#else
 const std::string expected_key="expected_before",variant="original_strict";
#endif
 for(const auto&c:cases){auto array=decode(c.at("prediction"));auto reason=dyn::validate(array,c.at("now_ns").get<int64_t>()*1e-9,limits);bool correct=reason==c.at(expected_key).get<std::string>();pass=pass&&correct;rows.push_back(J{{"case",c.at("case")},{"reason",reason},{"expected",c.at(expected_key)},{"matches",correct}});}
 std::cout<<J{{"variant",variant},{"cases",rows},{"registered_acceptance_pass",pass},{"scope","pure copied validator only, original or one relational operator; no motion"}}.dump()<<'\n';return pass?0:3;}
