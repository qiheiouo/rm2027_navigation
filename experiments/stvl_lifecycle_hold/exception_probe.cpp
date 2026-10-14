// Direct fixed installed library call, no ROS node, optimizer or controller fork.
#include <nav2_mppi_controller/optimizer.hpp>
#include <nav2_core/exceptions.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
struct Fixture: mppi::Optimizer {
 void setup(){settings_.batch_size=2;settings_.time_steps=3;settings_.retry_attempt_limit=1;setMotionModel("Omni");reset();}
 void failed(bool v){critics_data_.fail_flag=v;}bool flag(){return critics_data_.fail_flag;}
 bool retry(bool fail){return fallback(fail);}
};
int main(){try{Fixture f;f.setup();f.failed(true);bool first=f.retry(true);bool retained=f.flag();std::string kind="none",message;try{f.retry(true);}catch(const nav2_core::PlannerException&e){kind="PlannerException";message=e.what();}catch(const std::runtime_error&e){kind="runtime_error";message=e.what();}
 f.failed(false);bool normal=f.retry(false);auto j=nlohmann::json{{"native_version","1.1.20"},{"first_fallback_returns_retry",first},{"fail_flag_after_native_reset",retained},{"exhausted_exception_type",kind},{"message",message},{"normal_returns_retry",normal},{"scope","2x3 data fixture, real installed fallback/reset, no command output"}};std::cout<<j.dump()<<'\n';return first&&retained&&kind=="runtime_error"&&!normal?0:2;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 3;}}
