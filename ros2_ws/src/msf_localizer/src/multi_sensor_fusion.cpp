// Port of catkin_ws/navigation/msf_localizer (MultiSensorFusionOld).
// Retains odometry prediction + NDT map correction; uses a REP-105 TF tree.
#include "msf_localizer/fusion_geometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace msf_localizer
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Matrix pose_matrix(const geometry_msgs::msg::Pose &p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  const Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6 || !t.allFinite()) {
    throw std::invalid_argument("non-finite pose or zero quaternion");
  }
  Matrix result = Matrix::Identity();
  result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  result.block<3, 1>(0, 3) = t;
  return result;
}

geometry_msgs::msg::Pose matrix_pose(const Matrix &m)
{
  geometry_msgs::msg::Pose p;
  p.position.x = m(0, 3); p.position.y = m(1, 3); p.position.z = m(2, 3);
  const Eigen::Quaterniond q(m.block<3, 3>(0, 0));
  p.orientation.x = q.x(); p.orientation.y = q.y();
  p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

class MultiSensorFusion : public rclcpp::Node
{
public:
  MultiSensorFusion() : Node("msf_localizer")
  {
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    for (const auto &frame : {map_frame_, odom_frame_, base_frame_}) {
      if (frame.empty() || frame.front() == '/') { throw std::invalid_argument("Invalid TF frame"); }
    }
    if (map_frame_ == odom_frame_ || odom_frame_ == base_frame_ || map_frame_ == base_frame_) {
      throw std::invalid_argument("map, odom and base frames must differ");
    }
    const auto cloud_topic = declare_parameter("cloud_topic", "/velodyne_points");
    const auto odom_topic = declare_parameter("odom_topic", "/odom");
    const auto secondary = declare_parameter("secondary_odom_topic", "");
    max_score_ = positive("max_fitness_score", 1.5);
    max_distance_ = positive("max_correction_distance", .8);
    max_angle_ = positive("max_correction_angle", .6);
    initial_distance_ = positive("initial_max_distance", 3.);
    initial_angle_ = positive("initial_max_angle", 1.2);
    max_gap_ = positive("max_odom_gap", .25);
    max_cloud_age_ = positive("max_cloud_age", 1.);
    localization_timeout_ = positive("localization_timeout", 2.);
    odom_timeout_ = positive("odom_timeout", .5);
    tolerance_ = positive("transform_tolerance", .1);
    scan_leaf_ = positive("scan_leaf_size", .4);
    min_range_ = positive("min_scan_range", 1.5);
    max_range_ = positive("max_scan_range", 60.);
    min_points_ = declare_parameter("min_scan_points", 50);
    const int iterations = declare_parameter("ndt_max_iterations", 30);
    if (max_range_ <= min_range_ || min_points_ < 6 || iterations < 1) {
      throw std::invalid_argument("Invalid NDT range, point count or iterations");
    }
    ndt_.setResolution(positive("ndt_resolution", 1.));
    ndt_.setStepSize(positive("ndt_step_size", .1));
    ndt_.setTransformationEpsilon(positive("ndt_epsilon", .01));
    ndt_.setMaximumIterations(iterations);
    iterations_ = iterations;
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cloud", rclcpp::QoS(1).transient_local().reliable());
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ndt_cloud", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    ndt_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/healthy", 1);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    load_map();
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr p) { odometry(*p, false); });
    if (!secondary.empty()) {
      secondary_sub_ = create_subscription<nav_msgs::msg::Odometry>(secondary, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr p) { odometry(*p, true); });
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(mutex_); pending_cloud_ = cloud;
      });
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr p) { initial_pose(*p); });
    match_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    match_timer_ = create_wall_timer(std::chrono::duration<double>(1. / positive("matching_rate", 5.)),
      [this] { match(); }, match_group_);
    health_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { health(); });
    RCLCPP_INFO(get_logger(), "NDT ready: waiting for /initialpose, %s and %s", odom_topic.c_str(), cloud_topic.c_str());
  }

private:
  double positive(const std::string &name, double value)
  {
    const double result = declare_parameter(name, value);
    if (!std::isfinite(result) || result <= 0.) { throw std::invalid_argument(name + " must be positive"); }
    return result;
  }

  Cloud::Ptr downsample(const Cloud::Ptr &cloud, double leaf)
  {
    auto result = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(leaf, leaf, leaf); filter.setInputCloud(cloud); filter.filter(*result);
    return result;
  }

  void load_map()
  {
    const auto file = declare_parameter("pcd_map", "");
    const auto dir = declare_parameter("pcd_dir", "");
    std::vector<std::filesystem::path> paths;
    if (!file.empty() && !dir.empty()) { throw std::invalid_argument("Set pcd_map OR pcd_dir"); }
    if (!file.empty()) { paths.emplace_back(file); }
    else if (!dir.empty()) {
      for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") { paths.push_back(entry.path()); }
      }
      std::sort(paths.begin(), paths.end());
    }
    if (paths.empty()) { throw std::invalid_argument("No PCD map: set pcd_map or pcd_dir"); }
    auto merged = std::make_shared<Cloud>();
    for (const auto &path : paths) {
      Cloud part;
      if (pcl::io::loadPCDFile(path.string(), part) < 0) { throw std::runtime_error("Cannot read " + path.string()); }
      for (const auto &p : part) { if (pcl::isFinite(p)) { merged->push_back(p); } }
    }
    map_ = downsample(merged, positive("map_leaf_size", .2));
    if (map_->size() < static_cast<size_t>(min_points_)) { throw std::runtime_error("PCD map is empty or too sparse"); }
    ndt_.setInputTarget(map_);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg); msg.header.frame_id = map_frame_; msg.header.stamp = now();
    map_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Loaded %zu map points from %zu PCD file(s)", map_->size(), paths.size());
  }

  bool healthy_locked(double time) const
  {
    return localized_ && !odom_.empty() && time >= last_match_ && time - last_match_ <= localization_timeout_ &&
      time >= odom_.back().stamp - .05 && time - odom_.back().stamp <= odom_timeout_;
  }

  void initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    try {
      if (msg.header.frame_id != map_frame_) { throw std::invalid_argument("/initialpose must be in " + map_frame_); }
      const auto pose = pose_matrix(msg.pose.pose);
      std::lock_guard<std::mutex> lock(mutex_);
      initial_ = pose; initialized_ = true; localized_ = false; ++epoch_;
      initial_pending_ = true; pending_cloud_.reset();
      reason_ = "Initial pose received; waiting for a qualified NDT match";
    } catch (const std::exception &e) { RCLCPP_WARN(get_logger(), "Rejected initial pose: %s", e.what()); }
  }

  void odometry(const nav_msgs::msg::Odometry &msg, bool secondary)
  {
    try {
      if (msg.child_frame_id != base_frame_ || (!secondary && msg.header.frame_id != odom_frame_)) {
        throw std::invalid_argument("Odometry frame mismatch; expected odom -> base_link (base pose, not IMU pose)");
      }
      const double stamp = rclcpp::Time(msg.header.stamp).seconds();
      if (stamp <= 0. || std::abs(now().seconds() - stamp) > 1.) { return; }
      const auto matrix = pose_matrix(msg.pose.pose);
      std::lock_guard<std::mutex> lock(mutex_);
      auto &history = secondary ? secondary_ : odom_;
      if (!history.empty() && stamp <= history.back().stamp) { return; }
      history.push_back({stamp, matrix});
      while (history.size() > 1500 || stamp - history.front().stamp > 15.) { history.pop_front(); }
      if (secondary || !healthy_locked(now().seconds())) { return; }
      const auto pose = matrix_pose(map_odom_ * matrix);
      geometry_msgs::msg::PoseStamped current;
      current.header.frame_id = map_frame_; current.header.stamp = msg.header.stamp; current.pose = pose;
      pose_pub_->publish(current);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = map_frame_; tf.child_frame_id = odom_frame_;
      tf.header.stamp = rclcpp::Time(msg.header.stamp) + rclcpp::Duration::from_seconds(tolerance_);
      const auto correction = matrix_pose(map_odom_);
      tf.transform.translation.x = correction.position.x; tf.transform.translation.y = correction.position.y;
      tf.transform.translation.z = correction.position.z; tf.transform.rotation = correction.orientation;
      broadcaster_->sendTransform(tf);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Rejected odometry: %s", e.what());
    }
  }

  void match()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    Matrix prediction, odom, secondary = Matrix::Identity(), secondary_prediction;
    bool has_secondary = false, first;
    uint64_t epoch;
    double stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initialized_ || !pending_cloud_) { return; }
      cloud = pending_cloud_; pending_cloud_.reset();
      stamp = rclcpp::Time(cloud->header.stamp).seconds();
      const double age = now().seconds() - stamp;
      if (age < -.05 || age > max_cloud_age_) { reason_ = "Stale cloud or sensor clock mismatch"; return; }
      const auto sample = sample_pose(odom_, stamp, max_gap_);
      if (!sample) { reason_ = "No synchronized odometry for cloud timestamp"; return; }
      odom = *sample;
      if (initial_pending_) { map_odom_ = initial_ * odom.inverse(); initial_pending_ = false; }
      prediction = map_odom_ * odom;
      const auto second = sample_pose(secondary_, stamp, max_gap_);
      if (second) {
        secondary = *second;
        has_secondary = secondary_anchor_valid_ && localized_;
        secondary_prediction = last_map_base_ * last_secondary_.inverse() * secondary;
      }
      first = !localized_; epoch = epoch_;
    }
    try {
      if (cloud->header.frame_id.empty()) { throw std::runtime_error("Point cloud has no frame_id"); }
      const auto extrinsic = buffer_->lookupTransform(base_frame_, cloud->header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::Pose extrinsic_pose;
      extrinsic_pose.position.x = extrinsic.transform.translation.x;
      extrinsic_pose.position.y = extrinsic.transform.translation.y;
      extrinsic_pose.position.z = extrinsic.transform.translation.z;
      extrinsic_pose.orientation = extrinsic.transform.rotation;
      const Matrix base_lidar = pose_matrix(extrinsic_pose);
      auto raw = std::make_shared<Cloud>();
      pcl::fromROSMsg(*cloud, *raw);
      auto filtered = std::make_shared<Cloud>();
      for (const auto &p : *raw) {
        if (!pcl::isFinite(p)) { continue; }
        const double range = p.getVector3fMap().norm();
        if (range >= min_range_ && range <= max_range_) { filtered->push_back(p); }
      }
      auto scan = downsample(filtered, scan_leaf_);
      if (scan->size() < static_cast<size_t>(min_points_)) { throw std::runtime_error("Too few finite scan points"); }
      ndt_.setInputSource(scan);
      std::vector<Matrix> guesses{prediction};
      if (has_secondary) { guesses.push_back(secondary_prediction); }
      double best_score = std::numeric_limits<double>::infinity();
      Matrix best = Matrix::Identity();
      for (const auto &guess : guesses) {
        Cloud unused;
        ndt_.align(unused, (guess * base_lidar).cast<float>().eval());
        const Matrix candidate = ndt_.getFinalTransformation().cast<double>() * base_lidar.inverse();
        const double score = ndt_.getFitnessScore();
        if (ndt_.getFinalNumIteration() < iterations_ && acceptable(
          ndt_.hasConverged(), score, candidate, prediction, max_score_,
          first ? initial_distance_ : max_distance_, first ? initial_angle_ : max_angle_) && score < best_score)
        { best = candidate; best_score = score; }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (epoch != epoch_) { return; }  // A new /initialpose invalidates in-flight work.
        if (!std::isfinite(best_score)) { reason_ = "NDT rejected: convergence, fitness or correction limit"; return; }
        const double age = now().seconds() - stamp;
        if (age < 0. || age > localization_timeout_) { reason_ = "NDT result expired during matching"; return; }
        map_odom_ = best * odom.inverse(); localized_ = true; last_match_ = stamp; score_ = best_score;
        last_map_base_ = best; last_secondary_ = secondary;
        secondary_anchor_valid_ = !secondary_.empty() && sample_pose(secondary_, stamp, max_gap_).has_value();
        reason_ = "NDT tracking";
      }
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.frame_id = map_frame_; pose.header.stamp = cloud->header.stamp; pose.pose.pose = matrix_pose(best);
      // Diagnostic covariance only; NDT fitness is not a calibrated covariance.
      for (int axis : {0, 1, 2}) { pose.pose.covariance[axis * 6 + axis] = std::max(.01, best_score); }
      for (int axis : {3, 4, 5}) { pose.pose.covariance[axis * 6 + axis] = .1; }
      ndt_pose_pub_->publish(pose);
      if (aligned_pub_->get_subscription_count()) {
        Cloud aligned;
        pcl::transformPointCloud(*scan, aligned, (best * base_lidar).cast<float>().eval());
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(aligned, msg); msg.header = pose.header; aligned_pub_->publish(msg);
      }
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (epoch == epoch_) { reason_ = e.what(); }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "NDT waiting: %s", e.what());
    }
  }

  void health()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto time = now();
    if (last_clock_ > time.seconds() + .05) {
      initialized_ = localized_ = secondary_anchor_valid_ = false; ++epoch_;
      odom_.clear(); secondary_.clear(); pending_cloud_.reset(); reason_ = "Clock reset; set /initialpose again";
    }
    last_clock_ = time.seconds();
    std_msgs::msg::Bool healthy; healthy.data = healthy_locked(time.seconds()); health_pub_->publish(healthy);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = time;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "msf_localizer/NDT"; status.hardware_id = "RoboSense+JD03+Wit";
    status.level = healthy.data ? 0 : 2;
    status.message = healthy.data ? reason_ : "Localization unavailable: " + reason_;
    diagnostic_msgs::msg::KeyValue score; score.key = "fitness_score_m2"; score.value = std::to_string(score_);
    status.values.push_back(score); diagnostics.status.push_back(status); diagnostics_pub_->publish(diagnostics);
  }

  std::string map_frame_, odom_frame_, base_frame_, reason_{"Waiting for /initialpose"};
  double max_score_, max_distance_, max_angle_, initial_distance_, initial_angle_, max_gap_, max_cloud_age_;
  double localization_timeout_, odom_timeout_, tolerance_, scan_leaf_, min_range_, max_range_;
  double last_match_{0.}, last_clock_{0.}, score_{std::numeric_limits<double>::infinity()};
  int min_points_, iterations_;
  bool initialized_{false}, localized_{false}, initial_pending_{false}, secondary_anchor_valid_{false};
  uint64_t epoch_{0};
  Matrix map_odom_{Matrix::Identity()}, initial_{Matrix::Identity()};
  Matrix last_map_base_{Matrix::Identity()}, last_secondary_{Matrix::Identity()};
  std::mutex mutex_;
  std::deque<TimedPose> odom_, secondary_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  Cloud::Ptr map_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::CallbackGroup::SharedPtr match_group_;
  rclcpp::TimerBase::SharedPtr match_timer_, health_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, secondary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};
}  // namespace msf_localizer

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<msf_localizer::MultiSensorFusion>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("msf_localizer"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
