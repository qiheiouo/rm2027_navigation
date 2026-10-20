// Full production IMU methods and real ESKF. Visibility-only access follows
// the existing imu_init_native audit fixture; no production method is replaced.
#include <omp.h>
#include <cassert>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "common_lib.h"
#include "use-ikfom.hpp"
#define private public
#include "src/IMU_Processing.hpp"
#undef private
using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
void observe(state_ikfom&, esekfom::dyn_share_datastruct<double>& d) { d.valid = false; }
void initialize(Filter& f) {
  double e[23]; std::fill(e, e+23, 1.e-7);
  f.init_dyn_share(get_f, df_dx, df_dw, observe, 4, e);
}
void stamp(builtin_interfaces::msg::Time& s, double t) {
  auto ns = llround(t*1e9); s.sec = ns/1000000000; s.nanosec = ns%1000000000;
}
MeasureGroup frame(double start, int count, double gyro = 0.) {
  MeasureGroup m; m.lidar_beg_time = m.lidar_beg_time2 = start;
  m.lidar_end_time = m.lidar_end_time2 = start + count*.005;
  m.lidar.reset(new PointCloudXYZI()); m.lidar2.reset(new PointCloudXYZI());
  for (float offset : {0.f, 25.f, 49.f}) {
    PointType p{}; p.x=5; p.y=1; p.z=1; p.curvature=offset;
    m.lidar->push_back(p); m.lidar2->push_back(p);
  }
  for (int i=0; i<count; ++i) {
    auto a=std::make_shared<sensor_msgs::msg::Imu>(); stamp(a->header.stamp,start+i*.005);
    a->linear_acceleration.z=9.81; a->angular_velocity.z=gyro; m.imu.push_back(a);
  }
  return m;
}
void configure(ImuProcess& p) { p.set_acc_cov(V3D(.1,.1,.1)); p.set_gyr_cov(V3D(.1,.1,.1)); }

void set_imu(MeasureGroup& m, const V3D& acc, const V3D& gyro) {
  for (auto& imu : m.imu) {
    auto replacement = std::make_shared<sensor_msgs::msg::Imu>(*imu);
    replacement->linear_acceleration.x = acc.x();
    replacement->linear_acceleration.y = acc.y();
    replacement->linear_acceleration.z = acc.z();
    replacement->angular_velocity.x = gyro.x();
    replacement->angular_velocity.y = gyro.y();
    replacement->angular_velocity.z = gyro.z();
    imu = replacement;
  }
}

void assert_filter_unchanged(Filter& f, const state_ikfom& before_state,
                             const Filter::cov& before_covariance) {
  Eigen::Matrix<double, 23, 1> delta;
  f.get_x().boxminus(delta, before_state);
  assert(delta.allFinite());
  assert(delta.cwiseAbs().maxCoeff() == 0.0);
  assert((f.get_P() - before_covariance).cwiseAbs().maxCoeff() == 0.0);
}

void invalid_initialization_case(const std::string& name) {
  Filter f;
  initialize(f);
  ImuProcess p;
  configure(p);
  auto out = std::make_shared<PointCloudXYZI>();
  auto bad = frame(300.0, 20);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  if (name == "zero_acc") set_imu(bad, V3D::Zero(), V3D::Zero());
  else if (name == "near_zero_acc") set_imu(bad, V3D(1.e-12, 0.0, 0.0), V3D::Zero());
  else if (name == "nan_acc") set_imu(bad, V3D(nan, 0.0, 9.81), V3D::Zero());
  else if (name == "inf_acc") set_imu(bad, V3D(0.0, inf, 9.81), V3D::Zero());
  else if (name == "nan_gyro") set_imu(bad, V3D(0.0, 0.0, 9.81), V3D(nan, 0.0, 0.0));
  else if (name == "inf_gyro") set_imu(bad, V3D(0.0, 0.0, 9.81), V3D(0.0, inf, 0.0));
  else assert(false && "unknown invalid initialization case");

  const auto before_state = f.get_x();
  const auto before_covariance = f.get_P();
  p.Process(bad, f, out, false, 1);
  assert(p.imu_need_init_);
  assert(p.init_iter_num == 1);
  assert(out->empty());
  assert_filter_unchanged(f, before_state, before_covariance);

  auto recovery = frame(301.0, 20);
  p.Process(recovery, f, out, false, 1);
  assert(!p.imu_need_init_);
  assert(out->empty());
  assert(f.get_P().allFinite());
  Eigen::Matrix<double, 23, 1> recovered_delta;
  f.get_x().boxminus(recovered_delta, state_ikfom());
  assert(recovered_delta.allFinite());

  p.Reset();
  const auto reset_state = f.get_x();
  const auto reset_covariance = f.get_P();
  bad = frame(400.0, 20);
  if (name == "zero_acc") set_imu(bad, V3D::Zero(), V3D::Zero());
  else if (name == "near_zero_acc") set_imu(bad, V3D(1.e-12, 0.0, 0.0), V3D::Zero());
  else if (name == "nan_acc") set_imu(bad, V3D(nan, 0.0, 9.81), V3D::Zero());
  else if (name == "inf_acc") set_imu(bad, V3D(0.0, inf, 9.81), V3D::Zero());
  else if (name == "nan_gyro") set_imu(bad, V3D(0.0, 0.0, 9.81), V3D(nan, 0.0, 0.0));
  else set_imu(bad, V3D(0.0, 0.0, 9.81), V3D(0.0, inf, 0.0));
  p.Process(bad, f, out, false, 1);
  assert(p.imu_need_init_ && p.init_iter_num == 1 && out->empty());
  assert_filter_unchanged(f, reset_state, reset_covariance);
  p.Process(frame(401.0, 20), f, out, false, 1);
  assert(!p.imu_need_init_ && f.get_P().allFinite());
  std::cout << "INVALID_INIT_REJECTED " << name << std::endl;
}

void noisy_initialization_case() {
  Filter f;
  initialize(f);
  ImuProcess p;
  configure(p);
  auto out = std::make_shared<PointCloudXYZI>();
  auto noisy = frame(500.0, 20);
  for (std::size_t i = 0; i < noisy.imu.size(); ++i) {
    const double sign = i % 2 == 0 ? 1.0 : -1.0;
    auto replacement = std::make_shared<sensor_msgs::msg::Imu>(*noisy.imu[i]);
    replacement->linear_acceleration.x = sign * 0.002;
    replacement->linear_acceleration.y = -sign * 0.001;
    replacement->linear_acceleration.z = 9.81 + sign * 0.004;
    replacement->angular_velocity.x = sign * 1.e-4;
    noisy.imu[i] = replacement;
  }
  p.Process(noisy, f, out, false, 1);
  assert(!p.imu_need_init_ && out->empty() && f.get_P().allFinite());
  Eigen::Matrix<double, 23, 1> delta;
  f.get_x().boxminus(delta, state_ikfom());
  assert(delta.allFinite());
  std::cout << "NOISY_INIT_ACCEPTED" << std::endl;
}

void mixed_initialization_case() {
  Filter f;
  initialize(f);
  ImuProcess p;
  configure(p);
  auto out = std::make_shared<PointCloudXYZI>();
  auto mixed = frame(600.0, 20);
  set_imu(mixed, V3D::Zero(), V3D::Zero());
  for (int i = 0; i < 9; ++i) {
    auto valid = std::make_shared<sensor_msgs::msg::Imu>(*mixed.imu[i]);
    valid->linear_acceleration.z = 9.81;
    mixed.imu[i] = valid;
  }
  p.Process(mixed, f, out, false, 1);
  assert(p.imu_need_init_ && p.init_iter_num == 10 && out->empty());
  p.Process(frame(601.0, 1), f, out, false, 1);
  assert(!p.imu_need_init_ && p.init_iter_num == 11 && out->empty());
  assert(f.get_P().allFinite());
  std::cout << "MIXED_INIT_COUNTS_VALID_ONLY" << std::endl;
}

int main(int argc, char** argv) {
  std::cout<<std::setprecision(17);
  if (argc == 3 && std::string(argv[1]) == "invalid") {
    invalid_initialization_case(argv[2]);
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "noise") {
    noisy_initialization_case();
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "mixed") {
    mixed_initialization_case();
    return 0;
  }
  assert(argc == 1);
  for (int mode=0; mode<3; ++mode) {
    Filter f; initialize(f); ImuProcess p; configure(p);
    auto out=std::make_shared<PointCloudXYZI>(); bool dual=mode==1; int side=mode==2?2:1;
    // Multiple initialization frames, then completion and the first propagation.
    p.Process(frame(100.,4),f,out,dual,side); assert(p.imu_need_init_ && out->empty());
    p.Process(frame(100.02,4),f,out,dual,side); assert(p.imu_need_init_ && out->empty());
    auto init=frame(100.04,20);
    if (mode==2) init.lidar_end_time=100.135; // selected lidar2 still ends at .14
    if (dual) init.lidar_end_time=100.135;
    p.Process(init,f,out,dual,side);
    assert(!p.imu_need_init_ && out->empty());
    assert(std::abs(p.last_lidar_end_time_-100.14)<1e-12);
    assert(p.acc_s_last.isZero() && p.angvel_last.isZero());
    // Independently specified clipped IMU intervals for these finite inputs.
    Filter expected; initialize(expected); auto initial=f.get_x(); auto covariance=f.get_P();
    expected.change_x(initial); expected.change_P(covariance);
    auto noise=p.Q;
    noise.block<3,3>(0,0).diagonal().setConstant(.1);
    noise.block<3,3>(3,3).diagonal().setConstant(.1);
    noise.block<3,3>(6,6).diagonal().setConstant(.0001);
    noise.block<3,3>(9,9).diagonal().setConstant(.0001);
    double max_state_error=0, max_cov_error=0;
    for (int k=0; k<5; ++k) {
      auto m=frame(100.14+k*.1,20,.1);
      input_ikfom in; in.acc=V3D(0,0,G_m_s2); in.gyro=V3D(0,0,k==0?.05:.1);
      double dt=0.; expected.predict(dt,noise,in); in.gyro.z()=.1;
      dt=.005; for(int j=0;j<20;++j) expected.predict(dt,noise,in);
      p.Process(m,f,out,dual,side);
      auto x=f.get_x(); Eigen::Matrix<double,23,1> dx; x.boxminus(dx,expected.get_x());
      max_state_error=std::max(max_state_error,dx.cwiseAbs().maxCoeff());
      max_cov_error=std::max(max_cov_error,(f.get_P()-expected.get_P()).cwiseAbs().maxCoeff());
      assert(max_state_error<1e-10 && max_cov_error<1e-10);
      assert(std::abs(p.last_lidar_end_time_-m.lidar_end_time)<1e-12);
      assert(out->size()==(dual?6:3)); assert(f.get_P().allFinite());
      for (auto& point:out->points) assert(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z));
    }
    // Reset requires reinitialization; the old epoch cannot supply its time.
    p.Reset(); auto reset=frame(200.,20); p.Process(reset,f,out,dual,side);
    assert(!p.imu_need_init_ && std::abs(p.last_lidar_end_time_-200.1)<1e-12 && p.acc_s_last.isZero());
    std::cout<<"RESULT {\"mode\":"<<mode<<",\"updates\":5,\"state_error\":"<<max_state_error<<",\"cov_error\":"<<max_cov_error<<"}"<<std::endl;
  }
}
