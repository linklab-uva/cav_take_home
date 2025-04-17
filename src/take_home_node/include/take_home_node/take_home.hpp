#pragma once

#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  explicit TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);
  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg);

 private:
  void compute_wheel_slip();
  void compute_imu_jitter();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  // Node Time
  double time_ = 0.0;

  // Vehicle state (Task 2A)
  double v_x_ = 0.0;  // Longitudinal velocity (m/s)
  double v_y_ = 0.0;  // Lateral velocity (m/s)
  double omega_ = 0.0;  // Yaw rate (rad/s)
  double wheel_speeds_[4] = {0.0, 0.0, 0.0, 0.0};  // fr, fl, rr, rl (m/s)
  double steering_angle_ = 0.0;  // radians

  // IMU state (Task 2B)
  std::deque<double> imu_timestamps_;  // Sliding window of timestamps (s)
  bool has_imu_data_ = false;

  // Lap time state (Task 2C)
  double last_curvilinear_ = 0.0;  // Previous distance (m)
  double lap_start_time_ = -1.0;  // Start of current lap (s)
  bool has_curvilinear_ = false;  // Flag for ~0.0 detection
  bool first_zero_seen_ = false;  // First lap start detected
  bool previous_value_was_zero_ = false;  // Was last distance ~0.0

  // Flags for data readiness (Task 2A)
  bool has_odom_ = false;
  bool has_wheel_speeds_ = false;
  bool has_steering_ = false;

  // Constants
  static constexpr double W_F = 1.638;  // Front track width (m)
  static constexpr double W_R = 1.523;  // Rear track width (m)
  static constexpr double L_F = 1.7238;  // COG to front wheels (m)
  static constexpr double STEERING_RATIO = 15.0;
  static constexpr double KMH_TO_MS = 1000.0 / 3600.0;
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double JITTER_WINDOW = 1.0;  // 1-second window for IMU jitter
};
