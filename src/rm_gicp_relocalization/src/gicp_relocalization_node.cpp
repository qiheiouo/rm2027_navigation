#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rm_gicp_relocalization/relocalization_math.hpp"

namespace
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(rotation.norm()) || rotation.norm() < 1.0e-9) {
    throw std::invalid_argument("initial pose contains an invalid quaternion");
  }
  rotation.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() << pose.position.x, pose.position.y, pose.position.z;
  transform.linear() = rotation.toRotationMatrix();
  if (!transform.matrix().allFinite()) {
    throw std::invalid_argument("initial pose contains non-finite values");
  }
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.rotation());
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

}  // namespace

class GicpRelocalizationNode : public rclcpp::Node
{
public:
  GicpRelocalizationNode()
  : Node("gicp_relocalization"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    prior_pcd_file_ = declare_parameter<std::string>("prior_pcd_file", "");
    map_id_ = declare_parameter<std::string>("map_id", "untracked");
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/lio/cloud_registered");
    raw_pose_topic_ = declare_parameter<std::string>(
      "raw_pose_topic", "/localization/gicp_pose_raw");
    valid_topic_ = declare_parameter<std::string>(
      "valid_topic", "/localization/gicp_registration_valid");
    score_topic_ = declare_parameter<std::string>(
      "score_topic", "/localization/gicp_fitness_score");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.25);
    source_leaf_size_ = declare_parameter<double>("source_leaf_size", 0.20);
    registration_rate_hz_ = declare_parameter<double>("registration_rate_hz", 2.0);
    max_correspondence_distance_ = declare_parameter<double>(
      "max_correspondence_distance", 1.0);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 1.0e-3);
    euclidean_fitness_epsilon_ = declare_parameter<double>(
      "euclidean_fitness_epsilon", 1.0e-3);
    max_iterations_ = declare_parameter<int>("max_iterations", 30);
    min_source_points_ = declare_parameter<int>("min_source_points", 100);
    min_target_points_ = declare_parameter<int>("min_target_points", 100);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 0.50);
    max_translation_jump_ = declare_parameter<double>("max_translation_jump", 2.0);
    max_yaw_jump_ = declare_parameter<double>("max_yaw_jump", 1.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);

    validateParameters();
    loadMap();

    raw_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      raw_pose_topic_, rclcpp::QoS(10).reliable());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      valid_topic_, rclcpp::QoS(1).reliable().transient_local());
    score_pub_ = create_publisher<std_msgs::msg::Float64>(score_topic_, 10);
    map_id_pub_ = create_publisher<std_msgs::msg::String>(
      "/localization/gicp_map_id", rclcpp::QoS(1).reliable().transient_local());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        handleCloud(*message);
      });
    initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        handleInitialPose(*message);
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_gicp",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        has_initial_guess_ = false;
        last_registered_generation_ = cloud_generation_;
        publishValid(false);
        response->success = true;
        response->message = "GICP initial guess and validity cleared";
      });
    registration_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / registration_rate_hz_)),
      [this]() {performRegistration();});

    publishValid(false);
    std_msgs::msg::String map_id_message;
    map_id_message.data = map_id_;
    map_id_pub_->publish(map_id_message);
    RCLCPP_INFO(
      get_logger(),
      "GICP 3D backend ready: map=%s cloud=%s raw_pose=%s. It publishes no TF and "
      "will not register until /initialpose provides a seed.",
      prior_pcd_file_.c_str(), input_cloud_topic_.c_str(), raw_pose_topic_.c_str());
  }

private:
  void validateParameters() const
  {
    if (prior_pcd_file_.empty()) {
      throw std::invalid_argument("prior_pcd_file is required");
    }
    if (
      map_id_.empty() || map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      !std::isfinite(map_leaf_size_) || !std::isfinite(source_leaf_size_) ||
      !std::isfinite(registration_rate_hz_) ||
      !std::isfinite(max_correspondence_distance_) ||
      !std::isfinite(transformation_epsilon_) ||
      !std::isfinite(euclidean_fitness_epsilon_) ||
      !std::isfinite(max_fitness_score_) || !std::isfinite(max_translation_jump_) ||
      !std::isfinite(max_yaw_jump_) || !std::isfinite(transform_timeout_sec_) ||
      map_leaf_size_ <= 0.0 || source_leaf_size_ <= 0.0 ||
      registration_rate_hz_ <= 0.0 || max_correspondence_distance_ <= 0.0 ||
      transformation_epsilon_ <= 0.0 || euclidean_fitness_epsilon_ <= 0.0 ||
      max_iterations_ <= 0 || min_source_points_ < 4 || min_target_points_ < 4 ||
      max_fitness_score_ < 0.0 || max_translation_jump_ <= 0.0 ||
      max_yaw_jump_ <= 0.0 || transform_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid GICP relocalization parameters");
    }
  }

  void loadMap()
  {
    auto raw_map = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<Point>(prior_pcd_file_, *raw_map) < 0) {
      throw std::runtime_error("cannot load prior PCD: " + prior_pcd_file_);
    }
    std::vector<int> finite_indices;
    auto finite_map = std::make_shared<Cloud>();
    pcl::removeNaNFromPointCloud(*raw_map, *finite_map, finite_indices);
    raw_map = finite_map;
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setLeafSize(
      static_cast<float>(map_leaf_size_),
      static_cast<float>(map_leaf_size_),
      static_cast<float>(map_leaf_size_));
    voxel_filter.setInputCloud(raw_map);
    target_map_ = std::make_shared<Cloud>();
    voxel_filter.filter(*target_map_);
    if (target_map_->size() < static_cast<std::size_t>(min_target_points_)) {
      throw std::runtime_error("prior PCD has too few points after downsampling");
    }
    RCLCPP_INFO(
      get_logger(), "Loaded prior PCD: raw=%zu filtered=%zu",
      raw_map->size(), target_map_->size());
  }

  bool lookupOdomToBase(
    const builtin_interfaces::msg::Time & stamp,
    Eigen::Isometry3d & odom_to_base)
  {
    try {
      geometry_msgs::msg::TransformStamped transform;
      if (stamp.sec == 0 && stamp.nanosec == 0) {
        transform = tf_buffer_.lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
      } else {
        transform = tf_buffer_.lookupTransform(
          odom_frame_, base_frame_, stamp,
          rclcpp::Duration::from_seconds(transform_timeout_sec_));
      }
      odom_to_base = tf2::transformToEigen(transform.transform);
      return odom_to_base.matrix().allFinite();
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot lookup %s -> %s: %s",
        odom_frame_.c_str(), base_frame_.c_str(), error.what());
      return false;
    }
  }

  bool transformCloudToOdom(
    const sensor_msgs::msg::PointCloud2 & message,
    Cloud & cloud_in_odom)
  {
    Cloud raw_cloud;
    try {
      pcl::fromROSMsg(message, raw_cloud);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot decode PointCloud2: %s", error.what());
      return false;
    }
    if (message.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting cloud with empty frame_id");
      return false;
    }
    if (message.header.frame_id == odom_frame_) {
      cloud_in_odom = std::move(raw_cloud);
    } else {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          odom_frame_, message.header.frame_id, message.header.stamp,
          rclcpp::Duration::from_seconds(transform_timeout_sec_));
        const Eigen::Isometry3d odom_to_cloud = tf2::transformToEigen(transform.transform);
        pcl::transformPointCloud(
          raw_cloud, cloud_in_odom, odom_to_cloud.matrix().cast<float>());
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot transform cloud %s -> %s: %s",
          message.header.frame_id.c_str(), odom_frame_.c_str(), error.what());
        return false;
      }
    }
    std::vector<int> finite_indices;
    Cloud finite_cloud;
    pcl::removeNaNFromPointCloud(cloud_in_odom, finite_cloud, finite_indices);
    cloud_in_odom = std::move(finite_cloud);
    return true;
  }

  void handleCloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    if (rclcpp::Time(message.header.stamp).nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejecting cloud with zero timestamp");
      return;
    }
    auto cloud = std::make_shared<Cloud>();
    if (!transformCloudToOdom(message, *cloud)) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = cloud;
    latest_cloud_stamp_ = message.header.stamp;
    ++cloud_generation_;
  }

  void handleInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    if (message.header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting initial pose in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    Eigen::Isometry3d odom_to_base;
    if (!lookupOdomToBase(message.header.stamp, odom_to_base)) {
      return;
    }
    try {
      const Eigen::Isometry3d map_to_base = poseToEigen(message.pose.pose);
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_guess_ = rm_gicp_relocalization::mapToOdomFromInitialPose(
        map_to_base, odom_to_base);
      has_initial_guess_ = true;
      last_registered_generation_ = 0;
      publishValid(false);
      RCLCPP_INFO(get_logger(), "Accepted /initialpose as the GICP registration seed");
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(get_logger(), "Rejecting initial pose: %s", error.what());
    }
  }

  void performRegistration()
  {
    Cloud::Ptr source;
    builtin_interfaces::msg::Time stamp;
    Eigen::Isometry3d guess;
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_initial_guess_ || !latest_cloud_ || cloud_generation_ == last_registered_generation_) {
        return;
      }
      source = latest_cloud_;
      stamp = latest_cloud_stamp_;
      guess = map_to_odom_guess_;
      generation = cloud_generation_;
    }

    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setLeafSize(
      static_cast<float>(source_leaf_size_),
      static_cast<float>(source_leaf_size_),
      static_cast<float>(source_leaf_size_));
    voxel_filter.setInputCloud(source);
    auto filtered_source = std::make_shared<Cloud>();
    voxel_filter.filter(*filtered_source);
    if (filtered_source->size() < static_cast<std::size_t>(min_source_points_)) {
      rejectRegistration("source cloud has too few points", generation);
      return;
    }

    pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
    registration.setInputTarget(target_map_);
    registration.setInputSource(filtered_source);
    registration.setMaximumIterations(max_iterations_);
    registration.setMaxCorrespondenceDistance(max_correspondence_distance_);
    registration.setTransformationEpsilon(transformation_epsilon_);
    registration.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    Cloud aligned;
    registration.align(aligned, guess.matrix().cast<float>());
    const double score = registration.getFitnessScore(max_correspondence_distance_);
    publishScore(score);
    if (!registration.hasConverged() || !std::isfinite(score) || score > max_fitness_score_) {
      rejectRegistration("GICP did not meet convergence or fitness limits", generation);
      return;
    }

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.matrix() = registration.getFinalTransformation().cast<double>();
    if (!result.matrix().allFinite()) {
      rejectRegistration("GICP returned a non-finite transform", generation);
      return;
    }
    const double translation_jump = (result.translation() - guess.translation()).norm();
    const double yaw_jump = std::abs(
      rm_gicp_relocalization::planarYawDifference(result, guess));
    if (translation_jump > max_translation_jump_ || yaw_jump > max_yaw_jump_) {
      rejectRegistration("GICP correction jump exceeds safety limits", generation);
      return;
    }

    Eigen::Isometry3d odom_to_base;
    if (!lookupOdomToBase(stamp, odom_to_base)) {
      rejectRegistration("timestamped odom->base_link is unavailable", generation);
      return;
    }
    const Eigen::Isometry3d map_to_base =
      rm_gicp_relocalization::mapToBaseFromCorrection(result, odom_to_base);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose = eigenToPose(map_to_base);
    const double variance = std::clamp(score, 0.01, 0.50);
    pose.pose.covariance[0] = variance;
    pose.pose.covariance[7] = variance;
    pose.pose.covariance[14] = std::min(1.0, 2.0 * variance);
    pose.pose.covariance[21] = std::min(1.0, 2.0 * variance);
    pose.pose.covariance[28] = std::min(1.0, 2.0 * variance);
    pose.pose.covariance[35] = variance;
    raw_pose_pub_->publish(pose);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_to_odom_guess_ = result;
      last_registered_generation_ = generation;
    }
    publishValid(true);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "GICP accepted: source=%zu score=%.4f correction_jump=%.3f m/%.3f rad",
      filtered_source->size(), score, translation_jump, yaw_jump);
  }

  void rejectRegistration(const std::string & reason, const std::uint64_t generation)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_registered_generation_ = generation;
    }
    publishValid(false);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
  }

  void publishValid(const bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    valid_pub_->publish(message);
  }

  void publishScore(const double score)
  {
    std_msgs::msg::Float64 message;
    message.data = score;
    score_pub_->publish(message);
  }

  std::string prior_pcd_file_;
  std::string map_id_;
  std::string input_cloud_topic_;
  std::string raw_pose_topic_;
  std::string valid_topic_;
  std::string score_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string initial_pose_topic_;
  double map_leaf_size_{0.25};
  double source_leaf_size_{0.20};
  double registration_rate_hz_{2.0};
  double max_correspondence_distance_{1.0};
  double transformation_epsilon_{1.0e-3};
  double euclidean_fitness_epsilon_{1.0e-3};
  int max_iterations_{30};
  int min_source_points_{100};
  int min_target_points_{100};
  double max_fitness_score_{0.50};
  double max_translation_jump_{2.0};
  double max_yaw_jump_{1.0};
  double transform_timeout_sec_{0.10};

  Cloud::Ptr target_map_;
  Cloud::Ptr latest_cloud_;
  builtin_interfaces::msg::Time latest_cloud_stamp_;
  Eigen::Isometry3d map_to_odom_guess_{Eigen::Isometry3d::Identity()};
  bool has_initial_guess_{false};
  std::uint64_t cloud_generation_{0};
  std::uint64_t last_registered_generation_{0};
  std::mutex mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr raw_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_id_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GicpRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
