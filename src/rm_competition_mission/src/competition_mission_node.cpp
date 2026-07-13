#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_competition_interfaces/msg/chassis_mode.hpp"
#include "rm_competition_interfaces/msg/mission_state.hpp"
#include "rm_competition_interfaces/msg/referee_state.hpp"
#include "rm_competition_interfaces/srv/set_mission_mode.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

bool validMode(const std::string & mode)
{
  return mode == "hold" || mode == "home" || mode == "patrol" ||
         mode == "pursuit" || mode == "auto";
}

}  // namespace

class CompetitionMissionNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  CompetitionMissionNode()
  : Node("competition_mission_node")
  {
    tree_xml_ = declare_parameter<std::string>("tree_xml", "");
    startup_enabled_ = declare_parameter<bool>("startup_enabled", false);
    requested_mode_ = declare_parameter<std::string>("default_mode", "hold");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    navigate_action_ = declare_parameter<std::string>(
      "navigate_action", "/navigate_to_pose");
    tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 10.0);
    require_localization_valid_ = declare_parameter<bool>(
      "require_localization_valid", true);
    require_referee_state_ = declare_parameter<bool>("require_referee_state", true);
    require_game_running_ = declare_parameter<bool>("require_game_running", true);
    require_chassis_mode_ = declare_parameter<bool>("require_chassis_mode", true);
    allow_pursuit_ = declare_parameter<bool>("allow_pursuit", false);
    game_running_value_ = declare_parameter<int>("game_running_value", 4);
    retreat_hp_threshold_ = declare_parameter<int>("retreat_hp_threshold", 150);
    goal_update_distance_ = declare_parameter<double>("goal_update_distance", 0.35);
    goal_update_yaw_ = declare_parameter<double>("goal_update_yaw", 0.35);
    minimum_goal_update_sec_ = declare_parameter<double>("minimum_goal_update_sec", 0.5);
    goal_failure_retry_sec_ = declare_parameter<double>("goal_failure_retry_sec", 2.0);
    max_consecutive_goal_failures_ = declare_parameter<int>(
      "max_consecutive_goal_failures", 3);
    home_values_ = declare_parameter<std::vector<double>>(
      "home_pose", std::vector<double>{});
    patrol_values_ = declare_parameter<std::vector<double>>(
      "patrol_waypoints", std::vector<double>{});

    if (tree_xml_.empty()) {
      throw std::invalid_argument("tree_xml must point to a behavior tree file");
    }
    if (!validMode(requested_mode_)) {
      throw std::invalid_argument("default_mode must be hold, home, patrol, pursuit or auto");
    }
    if (tick_rate_hz_ <= 0.0 || goal_update_distance_ < 0.0 ||
      goal_update_yaw_ < 0.0 || minimum_goal_update_sec_ < 0.0 ||
      goal_failure_retry_sec_ < 0.0 || max_consecutive_goal_failures_ < 1)
    {
      throw std::invalid_argument("mission timing and goal thresholds are invalid");
    }
    loadWaypoints();
    operator_enabled_ = startup_enabled_;

    registerTreeNodes();
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_xml_));
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_);

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    mission_state_pub_ = create_publisher<rm_competition_interfaces::msg::MissionState>(
      "/mission/state", latched);
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/global_localization_valid", latched,
      [this](std_msgs::msg::Bool::SharedPtr message) {localization_valid_ = message->data;});
    referee_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/referee/state_valid", latched,
      [this](std_msgs::msg::Bool::SharedPtr message) {referee_valid_ = message->data;});
    referee_sub_ = create_subscription<rm_competition_interfaces::msg::RefereeState>(
      "/referee/state", latched,
      [this](rm_competition_interfaces::msg::RefereeState::SharedPtr message) {
        referee_state_ = *message;
        referee_received_ = true;
      });
    chassis_sub_ = create_subscription<rm_competition_interfaces::msg::ChassisMode>(
      "/chassis/mode", latched,
      [this](rm_competition_interfaces::msg::ChassisMode::SharedPtr message) {
        chassis_mode_ = *message;
        chassis_received_ = true;
      });
    pursuit_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mission/pursuit_goal_valid", latched,
      [this](std_msgs::msg::Bool::SharedPtr message) {pursuit_goal_valid_ = message->data;});
    pursuit_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission/pursuit_goal", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message) {pursuit_goal_ = *message;});

    set_mode_service_ = create_service<rm_competition_interfaces::srv::SetMissionMode>(
      "/mission/set_mode",
      [this](
        const std::shared_ptr<rm_competition_interfaces::srv::SetMissionMode::Request> request,
        std::shared_ptr<rm_competition_interfaces::srv::SetMissionMode::Response> response)
      {
        if (!validMode(request->mode)) {
          response->accepted = false;
          response->message = "mode must be hold, home, patrol, pursuit or auto";
          return;
        }
        operator_enabled_ = request->enable;
        requested_mode_ = request->mode;
        failed_goal_.reset();
        consecutive_goal_failures_ = 0;
        response->accepted = true;
        response->message = operator_enabled_ ? "mission enabled" : "mission disabled";
      });

    const auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});
    RCLCPP_WARN(
      get_logger(),
      "competition mission ready: enabled=%s mode=%s pursuit=%s. "
      "Only this node may convert mission choices into NavigateToPose goals.",
      operator_enabled_ ? "true" : "false", requested_mode_.c_str(),
      allow_pursuit_ ? "true" : "false");
  }

private:
  struct GoalRequest
  {
    geometry_msgs::msg::PoseStamped pose;
    std::string source;
  };

  void loadWaypoints()
  {
    if (!home_values_.empty() && home_values_.size() != 3) {
      throw std::invalid_argument("home_pose must be empty or [x, y, yaw]");
    }
    if (patrol_values_.size() % 3 != 0) {
      throw std::invalid_argument("patrol_waypoints must contain x, y, yaw triples");
    }
    if (home_values_.size() == 3) {
      home_pose_ = makePose(
        map_frame_, home_values_[0], home_values_[1], home_values_[2]);
    }
    for (std::size_t index = 0; index < patrol_values_.size(); index += 3) {
      patrol_poses_.push_back(makePose(
          map_frame_, patrol_values_[index], patrol_values_[index + 1],
          patrol_values_[index + 2]));
    }
  }

  void registerTreeNodes()
  {
    factory_.registerSimpleCondition(
      "MissionEnabled", [this](BT::TreeNode &) {
        return operator_enabled_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "SafetyReady", [this](BT::TreeNode &) {
        return safetyReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "ShouldReturnHome", [this](BT::TreeNode &) {
        const bool low_hp = referee_valid_ && referee_received_ &&
          referee_state_.current_hp <= retreat_hp_threshold_;
        return (requested_mode_ == "home" || low_hp) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "CanPursue", [this](BT::TreeNode &) {
        const bool mode_allows = requested_mode_ == "pursuit" || requested_mode_ == "auto";
        return (allow_pursuit_ && mode_allows && pursuit_goal_valid_ &&
               pursuit_goal_.header.frame_id == map_frame_) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "CanPatrol", [this](BT::TreeNode &) {
        const bool mode_allows = requested_mode_ == "patrol" || requested_mode_ == "auto";
        return (mode_allows && !patrol_poses_.empty()) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleAction(
      "SelectHome", [this](BT::TreeNode &) {return selectHome();});
    factory_.registerSimpleAction(
      "SelectPursuit", [this](BT::TreeNode &) {return selectPursuit();});
    factory_.registerSimpleAction(
      "SelectPatrol", [this](BT::TreeNode &) {return selectPatrol();});
    factory_.registerSimpleAction(
      "SelectHold", [this](BT::TreeNode &) {
        active_branch_ = "hold";
        desired_goal_.reset();
        return BT::NodeStatus::SUCCESS;
      });
  }

  bool safetyReady() const
  {
    if (require_localization_valid_ && !localization_valid_) {
      return false;
    }
    if (require_referee_state_ && (!referee_valid_ || !referee_received_)) {
      return false;
    }
    if (require_game_running_ &&
      (!referee_received_ || referee_state_.game_progress != game_running_value_))
    {
      return false;
    }
    if (require_chassis_mode_ &&
      (!chassis_received_ || !chassis_mode_.online ||
      !chassis_mode_.autonomous_enabled || chassis_mode_.emergency_stop))
    {
      return false;
    }
    return true;
  }

  BT::NodeStatus selectHome()
  {
    if (!home_pose_.has_value()) {
      active_branch_ = "hold_missing_home";
      desired_goal_.reset();
      return BT::NodeStatus::FAILURE;
    }
    desired_goal_ = GoalRequest{*home_pose_, "home"};
    active_branch_ = "home";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus selectPursuit()
  {
    if (!pursuit_goal_valid_ || pursuit_goal_.header.frame_id != map_frame_) {
      return BT::NodeStatus::FAILURE;
    }
    desired_goal_ = GoalRequest{pursuit_goal_, "pursuit"};
    active_branch_ = "pursuit";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus selectPatrol()
  {
    if (patrol_poses_.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto index = patrol_index_ % patrol_poses_.size();
    desired_goal_ = GoalRequest{patrol_poses_[index], "patrol"};
    active_branch_ = "patrol";
    return BT::NodeStatus::SUCCESS;
  }

  void tick()
  {
    desired_goal_.reset();
    tree_->tickRoot();
    reconcileNavigation();
    publishState();
  }

  bool goalChanged(const GoalRequest & left, const GoalRequest & right) const
  {
    if (left.source != right.source) {
      return true;
    }
    const double dx = left.pose.pose.position.x - right.pose.pose.position.x;
    const double dy = left.pose.pose.position.y - right.pose.pose.position.y;
    constexpr double kTwoPi = 6.28318530717958647692;
    const double dyaw = std::remainder(
      yawFromPose(left.pose.pose) - yawFromPose(right.pose.pose), kTwoPi);
    return std::hypot(dx, dy) > goal_update_distance_ || std::abs(dyaw) > goal_update_yaw_;
  }

  void reconcileNavigation()
  {
    if (!desired_goal_.has_value()) {
      cancelNavigation("mission hold or safety gate");
      return;
    }
    if (completed_goal_.has_value() && !goalChanged(*desired_goal_, *completed_goal_)) {
      navigation_status_ = "goal_complete";
      return;
    }
    if (failed_goal_.has_value()) {
      if (goalChanged(*desired_goal_, *failed_goal_)) {
        failed_goal_.reset();
        consecutive_goal_failures_ = 0;
      } else if (consecutive_goal_failures_ >= max_consecutive_goal_failures_) {
        navigation_status_ = "goal_retry_limit";
        return;
      } else if ((now() - last_goal_send_time_).seconds() < goal_failure_retry_sec_) {
        navigation_status_ = "goal_retry_backoff";
        return;
      }
    }
    if (last_sent_goal_.has_value() && !goalChanged(*desired_goal_, *last_sent_goal_)) {
      return;
    }
    const double since_send = (now() - last_goal_send_time_).seconds();
    if (last_goal_send_time_.nanoseconds() > 0 && since_send < minimum_goal_update_sec_) {
      return;
    }
    sendGoal(*desired_goal_);
  }

  void sendGoal(GoalRequest request)
  {
    if (!navigation_client_->action_server_is_ready()) {
      navigation_status_ = "waiting_for_nav2";
      return;
    }
    cancelNavigation("goal update", false);
    request.pose.header.stamp = now();
    NavigateToPose::Goal goal;
    goal.pose = request.pose;
    const std::uint64_t generation = ++goal_generation_;
    last_sent_goal_ = request;
    completed_goal_.reset();
    last_goal_send_time_ = now();
    navigation_status_ = "goal_pending";

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](GoalHandle::SharedPtr handle) {
        if (generation != goal_generation_) {
          if (handle) {
            navigation_client_->async_cancel_goal(handle);
          }
          return;
        }
        active_goal_handle_ = handle;
        navigation_status_ = handle ? "goal_active" : "goal_rejected";
        if (!handle) {
          failed_goal_ = last_sent_goal_;
          ++consecutive_goal_failures_;
          last_goal_send_time_ = now();
          last_sent_goal_.reset();
        }
      };
    options.result_callback =
      [this, generation](const GoalHandle::WrappedResult & result) {
        if (generation != goal_generation_) {
          return;
        }
        active_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          navigation_status_ = "goal_succeeded";
          failed_goal_.reset();
          consecutive_goal_failures_ = 0;
          completed_goal_ = last_sent_goal_;
          if (last_sent_goal_.has_value() && last_sent_goal_->source == "patrol" &&
            !patrol_poses_.empty())
          {
            patrol_index_ = (patrol_index_ + 1) % patrol_poses_.size();
            completed_goal_.reset();
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          navigation_status_ = "goal_canceled";
        } else {
          navigation_status_ = "goal_failed";
          failed_goal_ = last_sent_goal_;
          ++consecutive_goal_failures_;
          last_goal_send_time_ = now();
          last_sent_goal_.reset();
        }
      };
    navigation_client_->async_send_goal(goal, options);
  }

  void cancelNavigation(const std::string & reason, bool clear_completed = true)
  {
    if (active_goal_handle_) {
      navigation_client_->async_cancel_goal(active_goal_handle_);
    }
    if (active_goal_handle_ || last_sent_goal_.has_value()) {
      ++goal_generation_;
      navigation_status_ = "goal_canceled: " + reason;
    } else {
      navigation_status_ = "holding";
    }
    active_goal_handle_.reset();
    last_sent_goal_.reset();
    if (clear_completed) {
      completed_goal_.reset();
    }
  }

  void publishState()
  {
    rm_competition_interfaces::msg::MissionState state;
    state.header.stamp = now();
    state.header.frame_id = map_frame_;
    state.enabled = operator_enabled_;
    state.requested_mode = requested_mode_;
    state.active_branch = active_branch_;
    state.navigation_status = navigation_status_;
    state.detail = safetyReady() ? "safety_ready" : "safety_gate_closed";
    mission_state_pub_->publish(state);
  }

  std::string tree_xml_;
  bool startup_enabled_;
  bool operator_enabled_{false};
  std::string requested_mode_;
  std::string map_frame_;
  std::string navigate_action_;
  double tick_rate_hz_;
  bool require_localization_valid_;
  bool require_referee_state_;
  bool require_game_running_;
  bool require_chassis_mode_;
  bool allow_pursuit_;
  int game_running_value_;
  int retreat_hp_threshold_;
  double goal_update_distance_;
  double goal_update_yaw_;
  double minimum_goal_update_sec_;
  double goal_failure_retry_sec_;
  int max_consecutive_goal_failures_;
  std::vector<double> home_values_;
  std::vector<double> patrol_values_;
  std::optional<geometry_msgs::msg::PoseStamped> home_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> patrol_poses_;
  std::size_t patrol_index_{0};

  bool localization_valid_{false};
  bool referee_valid_{false};
  bool referee_received_{false};
  bool chassis_received_{false};
  bool pursuit_goal_valid_{false};
  rm_competition_interfaces::msg::RefereeState referee_state_;
  rm_competition_interfaces::msg::ChassisMode chassis_mode_;
  geometry_msgs::msg::PoseStamped pursuit_goal_;

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::optional<GoalRequest> desired_goal_;
  std::optional<GoalRequest> last_sent_goal_;
  std::optional<GoalRequest> completed_goal_;
  std::optional<GoalRequest> failed_goal_;
  int consecutive_goal_failures_{0};
  std::string active_branch_{"hold"};
  std::string navigation_status_{"holding"};
  rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t goal_generation_{0};
  GoalHandle::SharedPtr active_goal_handle_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Publisher<rm_competition_interfaces::msg::MissionState>::SharedPtr
    mission_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr referee_valid_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::RefereeState>::SharedPtr
    referee_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::ChassisMode>::SharedPtr
    chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pursuit_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pursuit_goal_sub_;
  rclcpp::Service<rm_competition_interfaces::srv::SetMissionMode>::SharedPtr
    set_mode_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompetitionMissionNode>());
  rclcpp::shutdown();
  return 0;
}
