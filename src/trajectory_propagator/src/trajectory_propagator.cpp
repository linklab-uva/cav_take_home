#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <spdlog/details/os.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <trajectory_propagator/trajectory_propagator.hpp>
#include <uva_iac_msgs/msg/batch_track.hpp>
#include <uva_iac_msgs/msg/batch_track_prediction.hpp>
#include <uva_iac_msgs/msg/track.hpp>
#include <uva_iac_msgs/msg/track_prediction.hpp>

#include "utils/math.hpp"

TrajectoryPropagator::TrajectoryPropagator(const rclcpp::NodeOptions& options)
    : Node("trajectory_propagator", options) {
  this->declare_parameter("track_name", "monza");
  this->declare_parameter("prediction_horizon", 5.0);
  this->declare_parameter("max_prediction_dist", 400.0);
  this->declare_parameter("publish_prediction", true);
  this->declare_parameter("publish_topic_name", "/prediction/trajectory_propagation");

  track_name_ = this->get_parameter("track_name").as_string();
  const std::string center_line = "center_line.csv";
  const std::string pitlane_center = "pitlane_center.csv";
  track_refline_ = load_reference_line(center_line);
  pits_refline_ = load_reference_line(pitlane_center);

  track_cloud_ = utils::PositionCloud(track_refline_);
  track_refline_tree_ = std::make_unique<utils::position_kdtree>(
      2, track_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  track_refline_tree_->buildIndex();

  pits_cloud_ = utils::PositionCloud(pits_refline_);
  pits_refline_tree_ = std::make_unique<utils::position_kdtree>(
      2, pits_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  pits_refline_tree_->buildIndex();

  auto qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());

#ifdef USING_OLD_BAG
  opponent_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "opponent_odom", qos_profile,
      std::bind(&TrajectoryPropagator::opponent_odom_callback, this, std::placeholders::_1));
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_out", qos_profile);
#endif

  batch_track_sub_ = this->create_subscription<uva_iac_msgs::msg::BatchTrack>(
      "opponent/tracks", qos_profile,
      std::bind(&TrajectoryPropagator::batch_track_callback, this, std::placeholders::_1));
  path_name_sub_ = this->create_subscription<std_msgs::msg::String>(
      "path_name", qos_profile, std::bind(&TrajectoryPropagator::path_name_callback, this, std::placeholders::_1));
  track_prediction_pub_ = this->create_publisher<uva_iac_msgs::msg::BatchTrackPrediction>(
      this->get_parameter("publish_topic_name").as_string(), qos_profile);

  track_prediction_objs_.resize(MAX_NUM_OF_OPPONENTS);
  prediction_trajectories_.resize(MAX_NUM_OF_OPPONENTS);

  RCLCPP_INFO(this->get_logger(), "Trajectory Propagator Ready for Prediction");
}

std::vector<utils::Position> TrajectoryPropagator::load_reference_line(const std::string& ref_line) {
  std::string prediction_share_dir = ament_index_cpp::get_package_share_directory("path_server_cpp");
  std::string reference_line_filename =
      std::filesystem::absolute(prediction_share_dir / std::filesystem::path("maps") /
                                std::filesystem::path(track_name_) / std::filesystem::path(ref_line));

  RCLCPP_INFO(this->get_logger(), "Loading reference line from file: %s", reference_line_filename.c_str());
  std::ifstream file(reference_line_filename);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + reference_line_filename);
  }

  std::string line;
  std::vector<Position> refline;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;

    while (std::getline(ss, value, ',')) {
      row.push_back(std::stod(value));
    }

    if (!row.empty()) {
      refline.emplace_back(Position(row[0], row[1]));
    }
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points into refline_", refline.size());

  if (refline.front().equals(refline.back())) {
    refline.pop_back();
  }

  return refline;
}

void TrajectoryPropagator::batch_track_callback(const uva_iac_msgs::msg::BatchTrack::SharedPtr batch_track) {
  if (!this->get_parameter("publish_prediction").as_bool()) {
    return;
  }
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
  max_prediction_dist_ = this->get_parameter("max_prediction_dist").as_double();

  batch_track_prediction_.header = batch_track->header;
  batch_track_prediction_.track_predictions.clear();

  // Generate trajectory predictions for each track
  for (size_t i = 0; i < batch_track->tracks.size() && i < track_prediction_objs_.size(); ++i) {
    float msg_time = static_cast<float>(batch_track->header.stamp.sec % 100) + batch_track->header.stamp.nanosec * 1e-9;

    process_pose_with_covariance(batch_track->tracks[i], batch_track->header, prediction_trajectories_[i], msg_time);

    track_prediction_objs_[i].track_id = batch_track->tracks[i].track_id;
    track_prediction_objs_[i].reputation = batch_track->tracks[i].reputation;
    track_prediction_objs_[i].position_history = prediction_trajectories_[i];

    batch_track_prediction_.track_predictions.push_back(track_prediction_objs_[i]);
  }

  track_prediction_pub_->publish(batch_track_prediction_);
}

void TrajectoryPropagator::path_name_callback(const std_msgs::msg::String::SharedPtr path_name) {
  use_pit_refline = path_name->data.find("pits") != std::string::npos;
}

#ifdef USING_OLD_BAG
void TrajectoryPropagator::opponent_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  if (!this->get_parameter("publish_prediction").as_bool()) {
    return;
  }
  process_pose_with_covariance(odom->pose, odom->twist, odom->header, prediction_trajectories_[0]);
  point_cloud_pub_->publish(prediction_trajectories_[0]);
}
#endif

double TrajectoryPropagator::get_velocity(const uva_iac_msgs::msg::Track& track, double ts) {
  uint32_t const track_id{track.track_id};

  if (velocity_history_.find(track_id) == velocity_history_.end()) {
    velocity_history_[track_id] = std::deque<double>();
    remove_old_track_ids(velocity_history_, track_id);
  }

  if (velocity_history_for_fit_.find(track_id) == velocity_history_for_fit_.end()) {
    velocity_history_for_fit_[track_id] = std::deque<std::pair<double, double>>();
    remove_old_track_ids(velocity_history_for_fit_, track_id);
  }

  double const speed{std::max(track.twist.twist.linear.x, 0.01)};

  // Update the history deque
  auto& velocities = velocity_history_[track_id];
  velocities.push_back(speed);
  if (velocities.size() > MAX_HISTORY) {
    velocities.pop_front();
  }

  auto& velocities_for_fit = velocity_history_for_fit_[track_id];
  velocities_for_fit.push_back(std::make_pair(ts, speed));
  if (velocities_for_fit.size() > MAX_HISTORY_FOR_FIT) {
    velocities_for_fit.pop_front();
  }

  // Calculate and return the average velocity
  return calculate_average(velocities);
}

double TrajectoryPropagator::get_acceleration(const uva_iac_msgs::msg::Track& track) {
  uint32_t const track_id{track.track_id};

  if (velocity_history_for_fit_.find(track_id) == velocity_history_for_fit_.end()) {
    return track.accel.twist.linear.x;
  }
  auto& velocities_for_fit = velocity_history_for_fit_[track_id];
  size_t n = velocities_for_fit.size();

  float constexpr TIME_IN_VEL_WINDOW_FOR_FIT{0.1F};
  uint32_t constexpr TRACKER_HZ{100U};
  if (n < static_cast<size_t>(TIME_IN_VEL_WINDOW_FOR_FIT * TRACKER_HZ)) {
    return track.accel.twist.linear.x;
  }
  // TODO(shreepa): ts is terrible, should be done recursivley similar to kalman filter
  Eigen::MatrixXd X(n, 2);
  Eigen::VectorXd y(n);

  size_t index = 0;

  float constexpr MAX_TIME_IN_QUEUE{3.0};
  if (std::fabs(velocities_for_fit.back().first - velocities_for_fit.front().first) > MAX_TIME_IN_QUEUE) {
    return track.accel.twist.linear.x;
  }

  for (const auto& [t, v] : velocities_for_fit) {
    X(index, 0) = t;  // slope
    X(index, 1) = 1;  // bias
    y(index) = v;
    ++index;
  }

  Eigen::Vector2d beta = (X.transpose() * X).ldlt().solve(X.transpose() * y);
  return beta(0);
}

sensor_msgs::msg::PointCloud2 TrajectoryPropagator::process_pose_with_covariance(
    const uva_iac_msgs::msg::Track& track, const std_msgs::msg::Header& header,
    sensor_msgs::msg::PointCloud2& pointcloud_out, double ts) {

  const geometry_msgs::msg::PoseWithCovariance& opponent_pose{track.pose};

  max_prediction_dist_ = this->get_parameter("max_prediction_dist").as_double();
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();

  auto& refline = use_pit_refline ? pits_refline_ : track_refline_;
  auto& refline_tree = use_pit_refline ? pits_refline_tree_ : track_refline_tree_;

  double vel = get_velocity(track, ts);
  double acc = get_acceleration(track);

  if (refline.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Reference line is empty. Cannot process pose.");
    return pointcloud_out;
  }

  // Calculates closest point to reference line.
  Position odom_position(track.pose.pose.position.x, track.pose.pose.position.y);
  size_t closest_idx = utils::get_closest_idx(*refline_tree, odom_position);
  size_t next_idx = (closest_idx + 1) % refline.size();

  // Calculate the offset vector from the closest point on the reference line.
  Eigen::Vector2d odom_pos(odom_position.x, odom_position.y);
  Eigen::Vector2d closest_refline_point(refline[closest_idx].x, refline[closest_idx].y);
  Eigen::Vector2d next_refline_point(refline[next_idx].x, refline[next_idx].y);

  Eigen::Vector2d long_basis{next_refline_point - closest_refline_point};
  long_basis.normalize();
  Eigen::Vector2d lat_basis(-long_basis[1], long_basis[0]);
  Eigen::Matrix2d reference_basis;
  reference_basis.col(0) = long_basis;
  reference_basis.col(1) = lat_basis;
  Eigen::Vector2d offset_raceline_basis = reference_basis.transpose() * (odom_pos - closest_refline_point);
  double lateral_offset{offset_raceline_basis.y()};

  size_t curr_idx = closest_idx;
  double total_dist = 0.0;

  output_cloud.reset(new pcl::PointCloud<uva_iac::perception::PointTimeType>());

  Eigen::Vector2d prev_pos;
  double time = 0.0;
  double prev_vel = vel;
  while (total_dist < max_prediction_dist_ && time < prediction_horizon_) {
    size_t next_idx = (curr_idx + 1) % refline.size();
    Eigen::Vector2d curr_point(refline[curr_idx].x, refline[curr_idx].y);
    Eigen::Vector2d next_point(refline[next_idx].x, refline[next_idx].y);

    Eigen::Vector2d long_basis{next_point - curr_point};
    long_basis.normalize();
    Eigen::Vector2d lat_basis(-long_basis[1], long_basis[0]);

    Eigen::Vector2d pred_pos = next_point + lateral_offset * lat_basis;
    double dist = 0.0;
    if (curr_idx != closest_idx) {
      dist = (pred_pos - prev_pos).norm();
    }

    total_dist += dist;

    // Compute delta time based on current velocity
    vel = sqrt(fmax(vel * vel + 2.0 * acc * dist, 0.0));
    if (vel < 0.1) {
      vel = 0.1;
    }
    double delta_time = 2.0 * dist / (vel + prev_vel);
    time += delta_time;
    prev_vel = vel;

    Eigen::Vector3f pos_vector(pred_pos.x(), pred_pos.y(), opponent_pose.pose.position.z);
    uva_iac::perception::PointTimeType point(pos_vector, vel, total_dist, time, DELTA_X, DELTA_Y);
    output_cloud->points.push_back(point);

    curr_idx = next_idx;
    prev_pos = pred_pos;
  }

  // Publish the output cloud.
  output_cloud->width = output_cloud->points.size();
  output_cloud->height = 1;
  // Convert the point cloud data to a PointCloud2 message.
  pcl::toROSMsg<uva_iac::perception::PointTimeType>(*output_cloud, pointcloud_out);

  pointcloud_out.header = header;
  return pointcloud_out;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryPropagator)
