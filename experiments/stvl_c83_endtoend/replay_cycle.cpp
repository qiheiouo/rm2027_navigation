// Standalone exact same-input checks. No ROS node or velocity output.
#include "../stvl_execution_realtime/optimized.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){try{if(argc<2)return 2;std::ifstream f(argv[1]);JointJson ctx;f>>ctx;prepared::Snapshot snap(ctx);JointJson rows=JointJson::array();for(auto&t:snap.trajectories){timing::Stats stats;auto x=prepared::diagnose(snap,t,stats);x["id"]=t.id;rows.push_back(x);}std::cout<<rows.dump()<<'\n';return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
