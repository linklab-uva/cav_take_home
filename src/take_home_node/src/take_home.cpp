#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <cmath>
#include <vector>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {
  // Enable use_sim_time for bag playback
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Subscribers
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

  wheel_speed_subscriber_ = create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

  steering_subscriber_ = create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

  imu_subscriber_ = create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

  curvilinear_subscriber_ = create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));

  // Publishers
  metric_publisher_ = create_publisher<std_msgs::msg::Float32>("/metrics_output", qos_profile);
  slip_rr_publisher_ = create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
  slip_rl_publisher_ = create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
  slip_fr_publisher_ = create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
  slip_fl_publisher_ = create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
  jitter_publisher_ = create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
  lap_time_publisher_ = create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}





// Task 2A //
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  v_x_ = odom_msg->twist.twist.linear.x;
  v_y_ = odom_msg->twist.twist.linear.y;
  omega_ = odom_msg->twist.twist.angular.z;
  has_odom_ = true;

  double current_time = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1e9;
  time_ = current_time;

  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = std::abs(position_x + position_z) > 1e-6 ?
      (position_x + position_y + position_z) / (position_x + position_z) : 0.0;
  metric_publisher_->publish(metric_msg);

  if (has_odom_ && has_wheel_speeds_ && has_steering_) {
    compute_wheel_slip();
  }
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg) {
  wheel_speeds_[0] = wheel_msg->front_right * KMH_TO_MS;
  wheel_speeds_[1] = wheel_msg->front_left * KMH_TO_MS;
  wheel_speeds_[2] = wheel_msg->rear_right * KMH_TO_MS;
  wheel_speeds_[3] = wheel_msg->rear_left * KMH_TO_MS;
  has_wheel_speeds_ = true;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  steering_angle_ = (steering_msg->primary_steering_angle_fbk / STEERING_RATIO) * DEG_TO_RAD;
  has_steering_ = true;
}

void TakeHome::compute_wheel_slip() {
  std_msgs::msg::Float32 msg;

  double v_x_rr = v_x_ - 0.5 * omega_ * W_R;
  msg.data = std::abs(v_x_rr) > 1e-6 ? static_cast<float>((wheel_speeds_[2] - v_x_rr) / v_x_rr) : 0.0;
  slip_rr_publisher_->publish(msg);

  double v_x_rl = v_x_ + 0.5 * omega_ * W_R;
  msg.data = std::abs(v_x_rl) > 1e-6 ? static_cast<float>((wheel_speeds_[3] - v_x_rl) / v_x_rl) : 0.0;
  slip_rl_publisher_->publish(msg);

  double v_x_fr = v_x_ - 0.5 * omega_ * W_F;
  double v_y_fr = v_y_ + omega_ * L_F;
  double v_x_fr_delta = std::cos(steering_angle_) * v_x_fr - std::sin(steering_angle_) * v_y_fr;
  msg.data = std::abs(v_x_fr_delta) > 1e-6 ? static_cast<float>((wheel_speeds_[0] - v_x_fr_delta) / v_x_fr_delta) : 0.0;
  slip_fr_publisher_->publish(msg);

  double v_x_fl = v_x_ + 0.5 * omega_ * W_F;
  double v_y_fl = v_y_ + omega_ * L_F;
  double v_x_fl_delta = std::cos(steering_angle_) * v_x_fl - std::sin(steering_angle_) * v_y_fl;
  msg.data = std::abs(v_x_fl_delta) > 1e-6 ? static_cast<float>((wheel_speeds_[1] - v_x_fl_delta) / v_x_fl_delta) : 0.0;
  slip_fl_publisher_->publish(msg);
}





// Task 2B //
void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  double current_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9;
  if (current_time <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid IMU timestamp: %f", current_time);
    return;
  }
  static constexpr double TIMESTAMP_GAP_THRESHOLD = 1.0;
  if (!imu_timestamps_.empty() && 
      std::abs(current_time - imu_timestamps_.back()) > TIMESTAMP_GAP_THRESHOLD) {
    imu_timestamps_.clear();
    has_imu_data_ = false;
  }
  if (!imu_timestamps_.empty() && current_time < imu_timestamps_.back()) {
    RCLCPP_WARN(this->get_logger(), "Non-monotonic timestamp: %f < %f", 
                current_time, imu_timestamps_.back());
    return;
  }
  imu_timestamps_.push_back(current_time);
  has_imu_data_ = true;
  while (!imu_timestamps_.empty() && 
         (current_time - imu_timestamps_.front()) > JITTER_WINDOW) {
    imu_timestamps_.pop_front();
  }
  if (imu_timestamps_.size() >= 2) {
    compute_imu_jitter();
  }
}

void TakeHome::compute_imu_jitter() {
  std::vector<double> deltas;
  deltas.reserve(imu_timestamps_.size() - 1);
  for (size_t i = 1; i < imu_timestamps_.size(); ++i) {
    deltas.push_back(imu_timestamps_[i] - imu_timestamps_[i - 1]);
  }
  double mean_delta = 0.0;
  double m2 = 0.0;
  size_t n = deltas.size();
  for (size_t i = 0; i < n; ++i) {
    double delta = deltas[i];
    double diff = delta - mean_delta;
    mean_delta += diff / (i + 1);
    m2 += diff * (delta - mean_delta);
  }
  double variance = m2 / n;
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(variance);
  jitter_publisher_->publish(msg);
}





// Task 2C //
void TakeHome::curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg) {
  float curvilinear_distance = dist_msg->data;
  double current_time = time_;

  if (lap_start_time_ < 0.0) {
    lap_start_time_ = current_time;
  }

  double lap_time = current_time - lap_start_time_;

  if (curvilinear_distance + 1000 < last_curvilinear_ && lap_time > 3) {
    std_msgs::msg::Float32 lap_time_msg;
    lap_time_msg.data = static_cast<float>(lap_time);
    
    lap_time_publisher_->publish(lap_time_msg);
    RCLCPP_INFO(this->get_logger(), "Lap completed: %f seconds", lap_time);
    
    lap_start_time_ = current_time;
    has_curvilinear_ = true; 
  }

  last_curvilinear_ = curvilinear_distance;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
