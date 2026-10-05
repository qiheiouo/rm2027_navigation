#pragma once
#include <nav2_core/controller.hpp>
#include <rm_r4_prediction_consumption/follow.hpp>

namespace rm_r4_nav2_controller
{
namespace r4 = rm_r4_prediction_consumption;
struct CycleToken
{
  r4::FollowIdentity identity;
  uint64_t plan_revision{}, speed_limit_revision{}, lifecycle_revision{};
};
bool same_token(const CycleToken & a, const CycleToken & b);
struct BoundCycle {CycleToken token; r4::FollowInput input;};
struct CycleResult {CycleToken token; r4::FollowResult result;};
// Typed in-process seam used by the original controller host, not another
// command topic or a frame_id protocol. Caller serializes lifecycle/bind/call/take.
class CycleAdapter
{
public:
  virtual ~CycleAdapter() = default;
  virtual bool bind_cycle(BoundCycle cycle) = 0;
  virtual std::optional<CycleResult> take_result(const CycleToken & token) = 0;
  virtual uint64_t plan_revision() const = 0;
  virtual uint64_t speed_limit_revision() const = 0;
  virtual uint64_t lifecycle_revision() const = 0;
};
class R4Controller final : public nav2_core::Controller, public CycleAdapter
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override;
  void setSpeedLimit(const double & limit, const bool & percentage) override;
  bool bind_cycle(BoundCycle cycle) override;
  std::optional<CycleResult> take_result(const CycleToken & token) override;
  uint64_t plan_revision() const override {return plan_revision_;}
  uint64_t speed_limit_revision() const override {return speed_revision_;}
  uint64_t lifecycle_revision() const override {return lifecycle_revision_;}
private:
  bool configured_{}, active_{}, speed_restricted_{};
  uint64_t plan_revision_{}, speed_revision_{}, lifecycle_revision_{};
  std::string plan_digest_;
  std::unique_ptr<r4::FollowSolver> solver_;
  std::optional<BoundCycle> pending_;
  std::optional<CycleResult> result_;
  void invalidate();
};
}  // namespace rm_r4_nav2_controller
