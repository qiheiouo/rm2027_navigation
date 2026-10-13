// Reuse actual IMU fixture visibility and the complete production timer/callbacks.
#define main existing_imu_fixture_main
#include "../fastlio_deterministic/imu_fixture.cpp"
#undef main
#define main fastlio_backend_main
#include BUNDLE_VISIBLE
#undef main

livox_ros_driver2::msg::CustomMsg::UniquePtr plane(double t, bool right) {
  auto m=std::make_unique<livox_ros_driver2::msg::CustomMsg>(); stamp(m->header.stamp,t);
  m->header.frame_id=right?"mid360_right_frame":"mid360_left_frame";m->timebase=llround(t*1e9);m->lidar_id=right?2:1;
  for(int i=0;i<900;++i){livox_ros_driver2::msg::CustomPoint p;
    const double y=-3.+(i%30)*.2;p.x=right?-5.:5.;p.y=right?-(y+.24):y;p.z=-3.+(i/30)*.2;
    p.offset_time=static_cast<unsigned>(i*99900000./899);p.reflectivity=50;p.line=i%4;p.tag=0;m->points.push_back(p);}
  m->point_num=m->points.size();return m;
}
void feed_pair(LaserMappingBundleNode& n,double t){livox_pcl_cbk(plane(t,false));n.timer_callback();livox_pcl_cbk2(plane(t,true));n.timer_callback();}
void imu(LaserMappingBundleNode& n,double t){auto m=std::make_unique<sensor_msgs::msg::Imu>();stamp(m->header.stamp,t);m->linear_acceleration.z=9.81;imu_cbk(std::move(m));n.timer_callback();}
void imus(LaserMappingBundleNode& n,double begin,double end){for(double t=begin;t<=end+1.e-7;t+=.005)imu(n,t);}
void snapshot(const char* phase){auto x=kf.get_x();const auto p=kf.get_P();
  std::cout<<"TRACE {\"phase\":\""<<phase<<"\",\"updates\":"<<processed_scans_for_zero
    <<",\"last_end\":"<<p_imu->last_lidar_end_time_<<",\"publish_stamp\":"<<publish_lidar_time
    <<",\"state\":["<<x.pos.x()<<","<<x.pos.y()<<","<<x.pos.z()<<","<<x.rot.coeffs().x()<<","<<x.rot.coeffs().y()<<","<<x.rot.coeffs().z()<<","<<x.rot.coeffs().w()
    <<"],\"cov_trace\":"<<p.trace()<<",\"cov_max\":"<<p.cwiseAbs().maxCoeff()<<",\"finite\":"<<(p.allFinite()&&x.pos.allFinite()?"true":"false")<<"}"<<std::endl;
}
int main(int argc,char**argv){std::string mode=argc>1?argv[1]:"control";const bool recovery=argc>2&&std::string(argv[2])=="recovery";rclcpp::init(argc,argv);std::cout<<std::setprecision(17);
 auto n=std::make_shared<LaserMappingBundleNode>();
 const double start=mode=="new_epoch"?900.1:1000.1;
 for(int k=0;k<8;++k){double t=start+k*.15;feed_pair(*n,t);imus(*n,t,t+.105);n->timer_callback();}
 snapshot("warm");
 if(mode=="control"){feed_pair(*n,1001.3);imus(*n,1001.26,1001.4);}
 if(mode=="backfill"){for(int k=0;k<3;++k)feed_pair(*n,1001.3+k*.15);snapshot("waiting");imus(*n,1001.26,1001.71);}
 if(mode=="rewind"){feed_pair(*n,1001.3);feed_pair(*n,900.1);imus(*n,900.1,900.205);snapshot("rewound_waiting");imu(*n,1001.71);}
 if(mode=="gap"){feed_pair(*n,1001.3);imu(*n,1001.71);}
 if(mode=="imu_rewind"){feed_pair(*n,1001.3);imu(*n,900.1);imu(*n,1001.71);}
 if(mode=="lidar_restart"){livox_pcl_cbk2(plane(900.1,true));n->timer_callback();imu(*n,1001.71);}
 if(mode=="duplicate"){feed_pair(*n,1001.3);imus(*n,1001.26,1001.4);imu(*n,1001.4);imu(*n,1001.4);}
 if(recovery&&(mode=="gap"||mode=="rewind"||mode=="imu_rewind"||mode=="lidar_restart")){
   snapshot("fault");feed_pair(*n,1001.8);imus(*n,1001.8,1001.905);
 }

 snapshot("final");n.reset();rclcpp::shutdown();
}
