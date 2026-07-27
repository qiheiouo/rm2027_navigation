#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_dog_hole/geometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

}  // namespace

class DogHoleManager : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
  using Trigger = std_srvs::srv::Trigger;

  DogHoleManager()
  : Node("dog_hole_manager"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    loadParameters();
    validateParameters();

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/dog_hole/state", 10);
    control_mode_pub_ =
      create_publisher<std_msgs::msg::String>("/dog_hole/control_mode", 10);
    path_crosses_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/dog_hole/path_crosses", rclcpp::QoS(1).reliable().transient_local());
    lateral_error_pub_ =
      create_publisher<std_msgs::msg::Float64>("/dog_hole/lateral_error", 10);
    heading_error_pub_ =
      create_publisher<std_msgs::msg::Float64>("/dog_hole/heading_error", 10);
    clearance_pub_ =
      create_publisher<std_msgs::msg::Float64>(
      "/dog_hole/minimum_wall_clearance", 10);
    command_diagnostic_pub_ =
      create_publisher<geometry_msgs::msg::Twist>("/dog_hole/command", 10);
    succeeded_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/dog_hole/succeeded", rclcpp::QoS(1).reliable().transient_local());

    compute_path_client_ =
      rclcpp_action::create_client<ComputePath>(this, compute_path_action_);
    navigate_client_ =
      rclcpp_action::create_client<Navigate>(this, navigate_action_);

    start_service_ = create_service<Trigger>(
      "/dog_hole/start",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
      {
        response->success = startMission();
        response->message = response->success ?
          "dog-hole mission started" : "mission busy or Nav2 actions unavailable";
      });
    cancel_service_ = create_service<Trigger>(
      "/dog_hole/cancel",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
      {
        cancelMission();
        response->success = true;
        response->message = "dog-hole mission canceled";
      });

    openLog();
    steady_start_time_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {controlTick();});

    publishSucceeded(false);
    RCLCPP_WARN(
      get_logger(),
      "New-car simulation candidate only: corridor %.2f m x %.2f m, "
      "robot %.2f m x %.2f m, command=%s. Real serial is not used.",
      corridor_.length, corridor_.width, robot_length_, robot_width_,
      command_topic_.c_str());
  }

private:
  enum class State
  {
    IDLE,
    PLANNING,
    APPROACHING,
    ALIGNING,
    CROSSING,
    EXITING,
    FINISHED,
    FAILED
  };

  enum class GoalPurpose
  {
    APPROACH,
    ALIGN,
    FINAL
  };

  void loadParameters()
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    navigate_action_ =
      declare_parameter<std::string>("navigate_action", "/navigate_to_pose");
    compute_path_action_ =
      declare_parameter<std::string>("compute_path_action", "/compute_path_to_pose");
    command_topic_ =
      declare_parameter<std::string>("command_topic", "/cmd_vel_nav");

    corridor_.center_x = declare_parameter<double>("dog_hole.center_x", -3.0);
    corridor_.center_y = declare_parameter<double>("dog_hole.center_y", 0.0);
    corridor_.yaw = declare_parameter<double>("dog_hole.yaw", M_PI);
    corridor_.width = declare_parameter<double>("dog_hole.width", 0.8);
    corridor_.length = declare_parameter<double>("dog_hole.length", 1.0);
    entry_clearance_ =
      declare_parameter<double>("dog_hole.entry_clearance", 0.8);
    exit_clearance_ =
      declare_parameter<double>("dog_hole.exit_clearance", 0.5);

    robot_length_ = declare_parameter<double>("robot.length", 0.6);
    robot_width_ = declare_parameter<double>("robot.width", 0.5);
    approach_offset_ = declare_parameter<double>("approach_offset", 0.15);

    final_goal_values_ = declare_parameter<std::vector<double>>(
      "final_goal", std::vector<double>{-5.0, 0.0, M_PI});
    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_start_delay_sec_ =
      declare_parameter<double>("auto_start_delay_sec", 8.0);

    control_rate_hz_ = declare_parameter<double>("control.rate_hz", 30.0);
    forward_speed_ = declare_parameter<double>("control.forward_speed", 0.35);
    lateral_gain_ = declare_parameter<double>("control.lateral_gain", 1.8);
    heading_gain_ = declare_parameter<double>("control.heading_gain", 2.5);
    max_lateral_speed_ =
      declare_parameter<double>("control.max_lateral_speed", 0.2);
    max_angular_speed_ =
      declare_parameter<double>("control.max_angular_speed", 0.8);
    minimum_clearance_ =
      declare_parameter<double>("control.minimum_clearance", 0.03);
    maximum_heading_error_ =
      declare_parameter<double>("control.maximum_heading_error", 0.4);
    crossing_timeout_sec_ =
      declare_parameter<double>("control.crossing_timeout_sec", 12.0);
    log_csv_ = declare_parameter<std::string>(
      "log_csv", "/tmp/rm2027_dog_hole_sim/dog_hole_run.csv");
  }

  void validateParameters() const
  {
    if (map_frame_.empty() || base_frame_.empty() || command_topic_.empty()) {
      throw std::invalid_argument("frame and command topic parameters must not be empty");
    }
    if (corridor_.width <= 0.0 || corridor_.length <= 0.0 ||
      robot_width_ <= 0.0 || robot_length_ <= 0.0 ||
      robot_width_ >= corridor_.width)
    {
      throw std::invalid_argument("corridor and robot dimensions are invalid");
    }
    if (final_goal_values_.size() != 3) {
      throw std::invalid_argument("final_goal must be [x, y, yaw]");
    }
    if (entry_clearance_ < 0.0 || exit_clearance_ < 0.0 ||
      approach_offset_ < 0.0 || control_rate_hz_ <= 0.0 ||
      forward_speed_ <= 0.0 || forward_speed_ > 0.8 ||
      lateral_gain_ < 0.0 || heading_gain_ < 0.0 ||
      max_lateral_speed_ < 0.0 || max_angular_speed_ < 0.0 ||
      minimum_clearance_ < 0.0 || maximum_heading_error_ <= 0.0 ||
      crossing_timeout_sec_ <= 0.0 || auto_start_delay_sec_ < 0.0)
    {
      throw std::invalid_argument("dog-hole timing, control, or clearance parameter is invalid");
    }
    const double nominal_clearance = 0.5 * (corridor_.width - robot_width_);
    if (minimum_clearance_ >= nominal_clearance) {
      throw std::invalid_argument("minimum clearance leaves no feasible centerline");
    }
  }

  void openLog()
  {
    if (log_csv_.empty()) {
      return;
    }
    try {
      const std::filesystem::path path(log_csv_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      const bool write_header =
        !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
      log_stream_.open(log_csv_, std::ios::out | std::ios::app);
      if (log_stream_ && write_header) {
        log_stream_ <<
          "time_sec,state,control_mode,longitudinal_m,lateral_m,"
          "heading_error_rad,minimum_clearance_m,vx_mps,vy_mps,wz_radps\n";
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV log: %s", error.what());
    }
  }

  bool actionsReady()
  {
    using namespace std::chrono_literals;
    return compute_path_client_->wait_for_action_server(0s) &&
           navigate_client_->wait_for_action_server(0s);
  }

  bool startMission()
  {
    if (state_ != State::IDLE && state_ != State::FINISHED && state_ != State::FAILED) {
      return false;
    }
    if (!actionsReady()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for Nav2 actions.");
      return false;
    }

    ++mission_generation_;
    path_crosses_ = false;
    auto_start_done_ = true;
    mission_start_time_ = now();
    publishSucceeded(false);
    transition(State::PLANNING, "PLANNER");
    requestPlan(mission_generation_);
    return true;
  }

  void cancelMission()
  {
    ++mission_generation_;
    compute_path_client_->async_cancel_all_goals();
    navigate_client_->async_cancel_all_goals();
    publishZero();
    transition(State::IDLE, "STOPPED");
    path_crosses_ = false;
    publishPathCrosses();
    publishSucceeded(false);
  }

  void requestPlan(std::size_t generation)
  {
    geometry_msgs::msg::TransformStamped base_transform;
    if (!lookupBase(base_transform)) {
      fail("cannot plan without map -> base_link");
      return;
    }

    ComputePath::Goal goal;
    goal.goal = finalGoalPose();
    goal.use_start = false;

    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](const ComputePathGoalHandle::SharedPtr & goal_handle)
      {
        if (generation != mission_generation_) {
          return;
        }
        if (!goal_handle) {
          fail("ComputePathToPose goal rejected");
        }
      };
    options.result_callback =
      [this, generation](const ComputePathGoalHandle::WrappedResult & result)
      {
        if (generation != mission_generation_) {
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
          fail("ComputePathToPose failed");
          return;
        }
        std::vector<std::pair<double, double>> points;
        points.reserve(result.result->path.poses.size());
        for (const auto & pose : result.result->path.poses) {
          points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        path_crosses_ = rm_dog_hole::pathCrossesCorridor(
          points, corridor_, entry_clearance_, exit_clearance_);
        publishPathCrosses();
        RCLCPP_INFO(
          get_logger(), "Global path has %zu poses; dog-hole crossing=%s.",
          points.size(), path_crosses_ ? "true" : "false");

        if (path_crosses_) {
          transition(State::APPROACHING, "NAV2");
          sendNavigationGoal(
            poseAt(-0.5 * corridor_.length - entry_clearance_),
            GoalPurpose::APPROACH, generation);
        } else {
          transition(State::EXITING, "NAV2");
          sendNavigationGoal(finalGoalPose(), GoalPurpose::FINAL, generation);
        }
      };
    compute_path_client_->async_send_goal(goal, options);
  }

  geometry_msgs::msg::PoseStamped poseAt(double longitudinal) const
  {
    const auto point = rm_dog_hole::pointAtLongitudinal(corridor_, longitudinal);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation = quaternionFromYaw(corridor_.yaw);
    return pose;
  }

  geometry_msgs::msg::PoseStamped finalGoalPose() const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = final_goal_values_[0];
    pose.pose.position.y = final_goal_values_[1];
    pose.pose.orientation = quaternionFromYaw(final_goal_values_[2]);
    return pose;
  }

  void sendNavigationGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    GoalPurpose purpose,
    std::size_t generation)
  {
    Navigate::Goal goal;
    goal.pose = pose;
    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](const NavigateGoalHandle::SharedPtr & goal_handle)
      {
        if (generation != mission_generation_) {
          return;
        }
        if (!goal_handle) {
          fail("NavigateToPose goal rejected");
        }
      };
    options.result_callback =
      [this, generation, purpose](const NavigateGoalHandle::WrappedResult & result)
      {
        if (generation != mission_generation_) {
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          fail("NavigateToPose failed or was canceled");
          return;
        }
        handleNavigationSuccess(purpose, generation);
      };
    navigate_client_->async_send_goal(goal, options);
  }

  void handleNavigationSuccess(GoalPurpose purpose, std::size_t generation)
  {
    if (purpose == GoalPurpose::APPROACH) {
      transition(State::ALIGNING, "NAV2");
      sendNavigationGoal(
        poseAt(-0.5 * corridor_.length - approach_offset_),
        GoalPurpose::ALIGN, generation);
      return;
    }
    if (purpose == GoalPurpose::ALIGN) {
      crossing_start_time_ = now();
      transition(State::CROSSING, "CENTERLINE");
      return;
    }

    publishZero();
    transition(State::FINISHED, "STOPPED");
    publishSucceeded(true);
    RCLCPP_INFO(get_logger(), "Dog-hole mission finished successfully.");
  }

  bool lookupBase(geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF %s -> %s unavailable: %s",
        map_frame_.c_str(), base_frame_.c_str(), error.what());
      return false;
    }
  }

  void crossingTick()
  {
    if ((now() - crossing_start_time_).seconds() > crossing_timeout_sec_) {
      fail("centerline crossing timeout");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupBase(transform)) {
      publishZero();
      return;
    }
    const double base_yaw = yawFromQuaternion(transform.transform.rotation);
    const auto pose = rm_dog_hole::evaluatePose(
      transform.transform.translation.x,
      transform.transform.translation.y,
      base_yaw,
      corridor_,
      robot_width_);
    latest_pose_ = pose;

    if (pose.minimum_wall_clearance < minimum_clearance_) {
      fail("minimum wall clearance violated");
      return;
    }
    if (std::abs(pose.heading_error) > maximum_heading_error_) {
      fail("base heading error exceeded crossing limit");
      return;
    }

    const double exit_longitudinal = 0.5 * corridor_.length + exit_clearance_;
    if (pose.longitudinal >= exit_longitudinal) {
      publishZero();
      transition(State::EXITING, "NAV2");
      sendNavigationGoal(finalGoalPose(), GoalPurpose::FINAL, mission_generation_);
      return;
    }

    const double axis_x = std::cos(corridor_.yaw);
    const double axis_y = std::sin(corridor_.yaw);
    const double normal_x = -axis_y;
    const double normal_y = axis_x;
    const double heading_scale = std::clamp(
      1.0 - std::abs(pose.heading_error) / maximum_heading_error_, 0.2, 1.0);
    const double forward = forward_speed_ * heading_scale;
    const double lateral = std::clamp(
      -lateral_gain_ * pose.lateral, -max_lateral_speed_, max_lateral_speed_);
    const double map_velocity_x = forward * axis_x + lateral * normal_x;
    const double map_velocity_y = forward * axis_y + lateral * normal_y;

    geometry_msgs::msg::Twist command;
    command.linear.x =
      std::cos(base_yaw) * map_velocity_x + std::sin(base_yaw) * map_velocity_y;
    command.linear.y =
      -std::sin(base_yaw) * map_velocity_x + std::cos(base_yaw) * map_velocity_y;
    command.angular.z = std::clamp(
      heading_gain_ * pose.heading_error, -max_angular_speed_, max_angular_speed_);
    publishCommand(command);
  }

  void controlTick()
  {
    if (auto_start_ && !auto_start_done_ && state_ == State::IDLE) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - steady_start_time_).count();
      if (elapsed >= auto_start_delay_sec_) {
        startMission();
      }
    }

    if (state_ == State::CROSSING) {
      crossingTick();
    }
    publishStatus();
  }

  void publishCommand(const geometry_msgs::msg::Twist & command)
  {
    latest_command_ = command;
    command_pub_->publish(command);
    command_diagnostic_pub_->publish(command);
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist zero;
    publishCommand(zero);
  }

  void publishPathCrosses()
  {
    std_msgs::msg::Bool message;
    message.data = path_crosses_;
    path_crosses_pub_->publish(message);
  }

  void publishSucceeded(bool succeeded)
  {
    std_msgs::msg::Bool message;
    message.data = succeeded;
    succeeded_pub_->publish(message);
  }

  void publishStatus()
  {
    std_msgs::msg::String state_message;
    state_message.data = stateName(state_);
    state_pub_->publish(state_message);

    std_msgs::msg::String mode_message;
    mode_message.data = control_mode_;
    control_mode_pub_->publish(mode_message);

    std_msgs::msg::Float64 value;
    value.data = latest_pose_.lateral;
    lateral_error_pub_->publish(value);
    value.data = latest_pose_.heading_error;
    heading_error_pub_->publish(value);
    value.data = latest_pose_.minimum_wall_clearance;
    clearance_pub_->publish(value);

    if (log_stream_) {
      const double elapsed =
        mission_start_time_.nanoseconds() == 0 ? 0.0 :
        (now() - mission_start_time_).seconds();
      log_stream_ << elapsed << ',' << stateName(state_) << ',' << control_mode_ << ',' <<
        latest_pose_.longitudinal << ',' << latest_pose_.lateral << ',' <<
        latest_pose_.heading_error << ',' << latest_pose_.minimum_wall_clearance << ',' <<
        latest_command_.linear.x << ',' << latest_command_.linear.y << ',' <<
        latest_command_.angular.z << '\n';
      log_stream_.flush();
    }
  }

  void transition(State state, const std::string & control_mode)
  {
    state_ = state;
    control_mode_ = control_mode;
    RCLCPP_INFO(
      get_logger(), "dog-hole state=%s control=%s",
      stateName(state_).c_str(), control_mode_.c_str());
  }

  void fail(const std::string & reason)
  {
    publishZero();
    transition(State::FAILED, "STOPPED");
    publishSucceeded(false);
    RCLCPP_ERROR(get_logger(), "Dog-hole mission failed: %s", reason.c_str());
  }

  static std::string stateName(State state)
  {
    switch (state) {
      case State::IDLE:
        return "IDLE";
      case State::PLANNING:
        return "PLANNING";
      case State::APPROACHING:
        return "APPROACHING";
      case State::ALIGNING:
        return "ALIGNING";
      case State::CROSSING:
        return "CROSSING";
      case State::EXITING:
        return "EXITING";
      case State::FINISHED:
        return "FINISHED";
      case State::FAILED:
        return "FAILED";
    }
    return "UNKNOWN";
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string navigate_action_;
  std::string compute_path_action_;
  std::string command_topic_;
  rm_dog_hole::Corridor corridor_;
  double entry_clearance_;
  double exit_clearance_;
  double robot_length_;
  double robot_width_;
  double approach_offset_;
  std::vector<double> final_goal_values_;
  bool auto_start_;
  double auto_start_delay_sec_;
  double control_rate_hz_;
  double forward_speed_;
  double lateral_gain_;
  double heading_gain_;
  double max_lateral_speed_;
  double max_angular_speed_;
  double minimum_clearance_;
  double maximum_heading_error_;
  double crossing_timeout_sec_;
  std::string log_csv_;

  State state_ = State::IDLE;
  std::string control_mode_ = "STOPPED";
  bool path_crosses_ = false;
  bool auto_start_done_ = false;
  std::size_t mission_generation_ = 0;
  rm_dog_hole::CorridorPose latest_pose_;
  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time crossing_start_time_;
  std::chrono::steady_clock::time_point steady_start_time_;
  std::ofstream log_stream_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_crosses_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr clearance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_diagnostic_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr succeeded_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogHoleManager>());
  rclcpp::shutdown();
  return 0;
}
