// Four ideal omni contacts. Only wheel joint velocities are commanded.
// Mass, roller friction, mounting directions and actuator response are hypotheses.
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/Utility.hh>

namespace rm_sim
{
class OmniWheelDrive : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  std::array<gz::sim::Entity, 4> joints{};
  std::array<gz::sim::Entity, 4> supports{};
  // FL, FR, RL, RR. Rolling tangents in base coordinates at home pose.
  std::array<double, 4> dy{{-1, 1, 1, -1}};
  std::array<double, 4> px{}, py{};
  gz::sim::Entity base{gz::sim::kNullEntity}, sensor{gz::sim::kNullEntity};
  gz::sim::Entity gimbal{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher odomPub, sensorPub, jointPub;
  std::mutex mutex;
  gz::msgs::Twist command;
  double radius{0}, gimbalSpeed{0}, staleSeconds{0};
  bool fresh{false};
  std::chrono::steady_clock::duration commandStamp{}, previous{}, lastPublish{};
  bool havePrevious{false};
  gz::math::Pose3d previousPose;

  void OnCommand(const gz::msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    command = msg;
    fresh = true;
  }
  void OnGimbal(const gz::msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (std::isfinite(msg.data())) gimbalSpeed = msg.data();
  }
  static void Velocity(gz::sim::Entity e, double v, gz::sim::EntityComponentManager & ecm)
  {
    auto c = ecm.Component<gz::sim::components::JointVelocityCmd>(e);
    if (c) c->Data() = {v};
    else ecm.CreateComponent(e, gz::sim::components::JointVelocityCmd({v}));
  }
  static void Stamp(gz::msgs::Header * h, const gz::sim::UpdateInfo & info,
                    const std::string & child)
  {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    h->mutable_stamp()->set_sec(ns / 1000000000);
    h->mutable_stamp()->set_nsec(ns % 1000000000);
    auto d = h->add_data(); d->set_key("frame_id"); d->add_value("map");
    d = h->add_data(); d->set_key("child_frame_id"); d->add_value(child);
  }

public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    base = model.LinkByName(ecm, "base_link");
    sensor = model.LinkByName(ecm, "sim_lidar_link");
    gimbal = model.JointByName(ecm, "gimbal_yaw_joint");
    radius = sdf->Get<double>("wheel_radius");
    const auto hx = sdf->Get<double>("wheelbase") / 2;
    const auto hy = sdf->Get<double>("track") / 2;
    staleSeconds = sdf->Get<double>("command_timeout");
    if (!(radius > 0 && hx > 0 && hy > 0 && staleSeconds > 0))
      throw std::runtime_error("invalid omni dimensions or timeout");
    px = {hx, hx, -hx, -hx}; py = {hy, -hy, hy, -hy};
    const std::array<std::string, 4> names = {"front_left", "front_right", "rear_left", "rear_right"};
    for (size_t i = 0; i < 4; ++i) {
      joints[i] = model.JointByName(ecm, names[i] + "_wheel_joint");
      if (joints[i] == gz::sim::kNullEntity) throw std::runtime_error("missing wheel joint");
      ecm.CreateComponent(joints[i], gz::sim::components::JointPosition());
      ecm.CreateComponent(joints[i], gz::sim::components::JointVelocity());
      supports[i] = model.JointByName(ecm, names[i] + "_support_joint");
      if (supports[i] != gz::sim::kNullEntity) {
        ecm.CreateComponent(supports[i], gz::sim::components::JointPosition());
        ecm.CreateComponent(supports[i], gz::sim::components::JointVelocity());
      }
    }
    if (base == gz::sim::kNullEntity || sensor == gz::sim::kNullEntity || gimbal == gz::sim::kNullEntity)
      throw std::runtime_error("missing base, gimbal or sensor");
    ecm.CreateComponent(gimbal, gz::sim::components::JointPosition());
    ecm.CreateComponent(gimbal, gz::sim::components::JointVelocity());
    node.Subscribe("/model/new_car/cmd_vel", &OmniWheelDrive::OnCommand, this);
    node.Subscribe("/model/new_car/gimbal_velocity", &OmniWheelDrive::OnGimbal, this);
    odomPub = node.Advertise<gz::msgs::Odometry>("/model/new_car/world_odometry");
    sensorPub = node.Advertise<gz::msgs::Odometry>("/model/new_car/sensor_world_odometry");
    jointPub = node.Advertise<gz::msgs::Model>("/model/new_car/joint_state");
  }
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) return;
    std::lock_guard<std::mutex> lock(mutex);
    if (fresh) { commandStamp = info.simTime; fresh = false; }
    const bool alive = std::chrono::duration<double>(info.simTime - commandStamp).count() <= staleSeconds;
    double vx = alive ? command.linear().x() : 0;
    double vy = alive ? command.linear().y() : 0;
    double wz = alive ? command.angular().z() : 0;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) vx = vy = wz = 0;
    for (size_t i = 0; i < 4; ++i) {
      const double speed = (vx + dy[i] * vy + (px[i] * dy[i] - py[i]) * wz) /
        (std::sqrt(2.0) * radius);
      Velocity(joints[i], speed, ecm);
    }
    Velocity(gimbal, gimbalSpeed, ecm);
  }
  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || info.simTime - lastPublish < std::chrono::milliseconds(20)) return;
    lastPublish = info.simTime;
    auto pose = gz::sim::worldPose(base, ecm);
    gz::msgs::Odometry odom;
    Stamp(odom.mutable_header(), info, "base_link");
    gz::msgs::Set(odom.mutable_pose(), pose);
    if (havePrevious && info.simTime > previous) {
      const double dt = std::chrono::duration<double>(info.simTime - previous).count();
      const auto v = pose.Rot().RotateVectorReverse((pose.Pos() - previousPose.Pos()) / dt);
      gz::msgs::Set(odom.mutable_twist()->mutable_linear(), v);
      // Body angular velocity from a quaternion delta; full SE(3), no Euler subtraction.
      auto q = previousPose.Rot().Inverse() * pose.Rot();
      if (q.W() < 0) q = gz::math::Quaterniond(-q.W(), -q.X(), -q.Y(), -q.Z());
      const double sn = std::sqrt(q.X()*q.X() + q.Y()*q.Y() + q.Z()*q.Z());
      if (sn > 1e-12) {
        const double f = 2 * std::atan2(sn, q.W()) / (sn * dt);
        gz::msgs::Set(odom.mutable_twist()->mutable_angular(), gz::math::Vector3d(q.X()*f,q.Y()*f,q.Z()*f));
      }
    }
    previous = info.simTime; previousPose = pose; havePrevious = true;
    odomPub.Publish(odom);
    gz::msgs::Odometry lidar;
    Stamp(lidar.mutable_header(), info, "sim_lidar_link");
    gz::msgs::Set(lidar.mutable_pose(), gz::sim::worldPose(sensor, ecm));
    sensorPub.Publish(lidar);
    gz::msgs::Model state;
    Stamp(state.mutable_header(), info, "new_car");
    for (size_t i = 0; i < 9; ++i) {
      const auto e = i < 4 ? joints[i] : (i == 4 ? gimbal : supports[i - 5]);
      if (e == gz::sim::kNullEntity) continue;
      auto j = state.add_joint();
      const std::array<std::string,9> names = {"front_left_wheel_joint","front_right_wheel_joint",
        "rear_left_wheel_joint","rear_right_wheel_joint","gimbal_yaw_joint", "front_left_support_joint",
        "front_right_support_joint", "rear_left_support_joint", "rear_right_support_joint"};
      j->set_name(names[i]);
      auto p = ecm.Component<gz::sim::components::JointPosition>(e);
      auto v = ecm.Component<gz::sim::components::JointVelocity>(e);
      if (p && !p->Data().empty()) j->mutable_axis1()->set_position(p->Data()[0]);
      if (v && !v->Data().empty()) j->mutable_axis1()->set_velocity(v->Data()[0]);
    }
    jointPub.Publish(state);
  }
};
}
GZ_ADD_PLUGIN(rm_sim::OmniWheelDrive, gz::sim::System,
  rm_sim::OmniWheelDrive::ISystemConfigure, rm_sim::OmniWheelDrive::ISystemPreUpdate,
  rm_sim::OmniWheelDrive::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(rm_sim::OmniWheelDrive, "rm_sim::OmniWheelDrive")
