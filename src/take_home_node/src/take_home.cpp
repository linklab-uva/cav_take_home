#include "take_home_node/take_home.hpp"
#include <cmath>
#include <cstdint>
#include <deque>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>

using namespace std::chrono_literals; // For timer

TakeHome::TakeHome(const rclcpp::NodeOptions &options)
    : Node("take_home_metrics", options) {

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  // Look at the hpp file to define all class variables, including subscribers
  // A subscriber will "listen" to a topic and whenever a message is published
  // to it, the subscriber will pass it onto the attached callback
  // (`TakeHome::odometry_callback` in this instance)

  // Example
  metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "metrics_output", qos_profile);

  /*
    A. Wheel slip
  */

  // Wheel slip ratio subscribers
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

  wheel_speed_subscriber_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "raptor_dbw_interface/wheel_speed_report", qos_profile,
          std::bind(&TakeHome::wheel_speed_callback, this,
                    std::placeholders::_1));

  wheel_angle_subscriber_ =
      this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
          "raptor_dbw_interface/steering_extended_report", qos_profile,
          std::bind(&TakeHome::wheel_angle_callback, this,
                    std::placeholders::_1));

  // Wheel slip ratio publishers
  slip_long_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "slip/long/rr", qos_profile);
  slip_long_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "slip/long/rl", qos_profile);
  slip_long_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "slip/long/fl", qos_profile);
  slip_long_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "slip/long/fr", qos_profile);

  // Publish the ratio (fixed rate)
  wheel_slip_timer_ = this->create_wall_timer(
      200ms, // Adjust as needed
      std::bind(&TakeHome::publish_slip_ratio_callback, this));

  /*
    B. IMU
  */

  // IMU Subscriber
  imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

  // IMU Publisher
  jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "imu_top/jitter", qos_profile);

  // Publish the ratio (fixed rate)
  imu_timer_ = this->create_wall_timer(
      200ms, // Adjust as needed
      std::bind(&TakeHome::publish_jitter_callback, this));

  /*
    C. Lap Time
  */

  lap_time_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::lap_time_callback, this, std::placeholders::_1));

  curvilenar_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));

  lap_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);

  // Publish the ratio (fixed rate)
  lap_time_timer_ = this->create_wall_timer(
      200ms, // Adjust as needed
      std::bind(&TakeHome::lap_time_publisher_callback, this));
}

/*
  A. Wheel Slip Ratio
*/

// Publish the wheel slip ratio every n ms
void TakeHome::publish_slip_ratio_callback() {
  for (int i = 0; i < 4; i++) {
    update_and_publish_k(i);
  }
}

void TakeHome::update_and_publish_k(int index) {
  float offset = (index % 2 == 0) ? -0.5f : 0.5f; // -0.5 for r, 0.5 for left

  if (index == 0 || index == 1) {
    v_x_per_wheel_[index] = v_x_ + offset * omega_ * w_r_;
    k_per_wheel_[index] =
        (v_w_per_wheel_[index] - v_x_per_wheel_[index]) / v_x_per_wheel_[index];
  } else if (index == 2 || index == 3) {
    v_x_per_wheel_[index] = v_x_ + offset * omega_ * w_f_;
    float v_y_f = v_y_ + omega_ * l_f_;
    float v_delta_x_f =
        cos(delta_) * v_x_per_wheel_[index] - sin(delta_) * v_y_f;
    k_per_wheel_[index] = (v_w_per_wheel_[index] - v_delta_x_f) / v_delta_x_f;
  }

  std_msgs::msg::Float32 k_msg;
  k_msg.data = k_per_wheel_[index];
  switch (index) {
  case 0:
    slip_long_rr_publisher_->publish(k_msg);
    break;
  case 1:
    slip_long_rl_publisher_->publish(k_msg);
    break;
  case 2:
    slip_long_fr_publisher_->publish(k_msg);
    break;
  default:
    slip_long_fl_publisher_->publish(k_msg);
    break;
  }
}

//
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the
 * subscriber passes the message onto this callback To see what is in each
 * message look for the corresponding .msg file in this repository For instance,
 * when running `ros2 bag info` on the given bag, we see the wheel speed report
 * has message type of raptor_dbw_msgs/msgs/WheelSpeedReport and from the
 * corresponding WheelSpeedReport.msg file we see that we can do msg->front_left
 * for the front left speed for instance. For the built in ROS2 messages, we can
 * find the documentation online: e.g.
 * https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {

  omega_ = odom_msg->twist.twist.angular.z; // Yaw rate (check)
  v_x_ = odom_msg->twist.twist.linear.x;
  v_y_ = odom_msg->twist.twist.linear.y;
}

// Update the wheel speed for each wheel in the buffer [rr, rl, fr, fl]
void TakeHome::wheel_speed_callback(
    raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {

  // KMPH
  float v_fl = wheel_speed_msg->front_left;
  float v_fr = wheel_speed_msg->front_right;
  float v_rl = wheel_speed_msg->rear_left;
  float v_rr = wheel_speed_msg->rear_right;

  float kmph_to_mps = 1000.0f / 3600.0f; // KMPH to m/s (1000 / 60 / 60)

  // Linear velocity for each wheel (m/s)
  v_w_per_wheel_[0] = v_rr * kmph_to_mps;
  v_w_per_wheel_[1] = v_rl * kmph_to_mps;
  v_w_per_wheel_[2] = v_fr * kmph_to_mps;
  v_w_per_wheel_[3] = v_fl * kmph_to_mps;
}

// Update the wheel angle
void TakeHome::wheel_angle_callback(
    raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr
        wheel_angle_msg) {

  float wheel_angle = wheel_angle_msg->primary_steering_angle_fbk; // deg
  delta_ = (wheel_angle * M_PI / 180) / 15;
}

/*
  B. IMU
*/

// Create a queue of timestamps in the past n sec
constexpr double IMU_BUFFER_WINDOW_SEC = 1.0;
void TakeHome::imu_callback(
    novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {

  int32_t sample_time_sec = imu_msg->header.stamp.sec;
  uint32_t sample_time_ns = imu_msg->header.stamp.nanosec;

  double sample_time = sample_time_sec + (sample_time_ns * 1e-9); // sec
  imu_sample_time_.push_back(sample_time);

  if (!imu_sample_time_.empty()) {
    double time_span = imu_sample_time_.back() - imu_sample_time_.front();
    while (time_span > IMU_BUFFER_WINDOW_SEC && imu_sample_time_.size() > 1) {
      imu_sample_time_.pop_front();
      time_span = imu_sample_time_.back() - imu_sample_time_.front();
    }
  }
}

// Publish variance of timestamps in last n sec
void TakeHome::publish_jitter_callback() {
  std::vector<double> deltas;
  for (uint32_t i = 1; i < imu_sample_time_.size(); i++) {
    deltas.push_back(imu_sample_time_[i] - imu_sample_time_[i - 1]);
  }

  // Mean of delta times
  double sum = std::accumulate(deltas.begin(), deltas.end(), 0.0);
  double mean = sum / deltas.size();

  // Variance (jitter)
  double variance = 0.0;
  for (const double &dt : deltas) {
    variance += std::pow(dt - mean, 2);
  }
  variance /= deltas.size();

  // Publish variance
  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = variance;
  jitter_publisher_->publish(jitter_msg);
}

/*
  C. Lap Time
*/

// Update record time flag when at start line
void TakeHome::curvilinear_callback(
    std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg) {
  float curvilinear_dist = curvilinear_msg->data;

  if (already_at_start_ == false && curvilinear_dist == 0) {
    record_time_ = true;
    already_at_start_ = true;
  } else if (curvilinear_dist != 0) {
    already_at_start_ = false;
  }
}

// Record the most recent time when record_time_ flag is set
void TakeHome::lap_time_callback(
    nav_msgs::msg::Odometry::ConstSharedPtr nav_msg) {
  if (record_time_ && lap_times_index_ < 2) {
    int sample_time_sec = nav_msg->header.stamp.sec;
    int sample_time_ns = nav_msg->header.stamp.nanosec;

    double sample_time = sample_time_sec + sample_time_ns * 1e-9; // in sec

    lap_times_[lap_times_index_] = sample_time;
    lap_times_index_++;
    record_time_ = false;
  }

  if (lap_times_index_ >= 2) {
    last_lap_time_ = std::abs(lap_times_[1] - lap_times_[0]);
    
    lap_times_index_ = 0;
  }
}

void TakeHome::lap_time_publisher_callback() {
  std_msgs::msg::Float32 lap_time_msg;
  lap_time_msg.data = last_lap_time_;
  lap_time_publisher_->publish(lap_time_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
