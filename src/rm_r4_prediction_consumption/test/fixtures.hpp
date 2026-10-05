#pragma once
#include "rm_r4_prediction_consumption/consumption.hpp"
namespace r4_test
{
using namespace rm_r4_prediction_consumption;
constexpr int64_t source = 1820000000123456789;
inline builtin_interfaces::msg::Time stamp(int64_t ns)
{
  builtin_interfaces::msg::Time out; out.sec = ns / 1000000000;
  out.nanosec = ns % 1000000000; return out;
}
inline BodyPolicy body()
{
  return {{{-.32, -.27}, {.32, -.27}, {.32, .27}, {-.32, .27}}, .02, 0., .05};
}
inline void align(rm_r4_interfaces::msg::ObservedPredictionEnvelope & e)
{
  const auto & s = e.prediction.header.stamp;
  const int64_t ns = static_cast<int64_t>(s.sec) * 1000000000 + s.nanosec;
  auto & t = e.prediction.tracks[0]; const auto & m = e.tracks[0];
  const int64_t obs = static_cast<int64_t>(m.last_observation_stamp.sec) * 1000000000 + m.last_observation_stamp.nanosec;
  const double dt = (ns - obs) * 1e-9;
  t.position.x = m.centroid_at_observation.x + dt * t.velocity.x;
  t.position.y = m.centroid_at_observation.y + dt * t.velocity.y;
  t.prediction.clear();
  for (int i = 1; i <= 15; ++i) {
    geometry_msgs::msg::Point p; p.x = t.position.x + i * .1 * t.velocity.x;
    p.y = t.position.y + i * .1 * t.velocity.y; t.prediction.push_back(p);
  }
}
inline rm_r4_interfaces::msg::ObservedPredictionEnvelope envelope(int64_t ns = source)
{
  rm_r4_interfaces::msg::ObservedPredictionEnvelope e;
  e.schema = e.SCHEMA; e.producer_id = "canonical-test-producer";
  e.sequence = 2; e.complete = true; e.reason = "ok";
  auto & p = e.prediction; p.header.frame_id = "map"; p.header.stamp = stamp(ns);
  p.schema = p.SCHEMA_OBSERVATION_ANCHOR; p.authority = p.AUTHORITY_SHADOW_ONLY;
  p.prediction_dt = .1; p.prediction_steps = 15; p.complete = true; p.total_track_count = 1;
  rm_competition_interfaces::msg::DynamicObstaclePrediction t;
  t.track_id = 7; t.state = t.STATE_COASTING; t.velocity.x = .5;
  t.size.x = t.size.y = .2; t.last_observation_stamp = stamp(ns - 100000000);
  t.observation_count = 3; t.miss_count = 1; p.tracks.push_back(t);
  rm_r4_interfaces::msg::ObservedTrackMembers m;
  m.track_id = 7; m.last_observation_stamp = t.last_observation_stamp;
  m.association_sequence = 1; m.centroid_at_observation.x = 1.;
  for (double x : {-.0625, 0., .0625}) {
    geometry_msgs::msg::Point point; point.x = x; m.local_endpoints.push_back(point);
  }
  m.source_member_ids = {1, 3, 9}; e.tracks.push_back(m); align(e); return e;
}
inline nav_msgs::msg::OccupancyGrid grid(double ox = -5., double oy = -4.)
{
  nav_msgs::msg::OccupancyGrid g; g.header.frame_id = "map";
  g.info.width = 200; g.info.height = 160; g.info.resolution = .05;
  g.info.origin.position.x = ox; g.info.origin.position.y = oy;
  g.info.origin.orientation.w = 1.; g.data.assign(g.info.width * g.info.height, 0); return g;
}
inline nav_msgs::msg::Path path(double dx = 0., double dy = 0.)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (double x : {-1., 0., 1.}) {
    geometry_msgs::msg::PoseStamped pose; pose.header.frame_id = "map";
    pose.pose.position.x = x + dx; pose.pose.position.y = dy; pose.pose.orientation.w = 1.;
    p.poses.push_back(pose);
  }
  return p;
}
}
