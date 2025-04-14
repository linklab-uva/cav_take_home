#pragma once

// Here we include message types which we can subscribe to or publish
#include <nav_msgs/msg/odometry.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <std_msgs/msg/float32.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

class TakeHome : public rclcpp::Node {
public:
  TakeHome(const rclcpp::NodeOptions &options);

  /*
    A. Wheel Slip callbacks
  */
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void wheel_speed_callback(
      raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void wheel_angle_callback(
      raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);

  void publish_slip_ratio_callback();

  /*
    B. IMU callbacks
  */
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void publish_jitter_callback();

  /*
    C. Lap Time callbacks
  */
  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg);
  void lap_time_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void lap_time_publisher_callback();

private:
  // Example
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr wheel_slip_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr lap_time_timer_;

  /*
    A. Wheel Slip
  */

  // Wheel Slip Ratio Sub/pub
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
      wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr
      wheel_angle_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fr_publisher_;

  void update_and_publish_k(int index);

  // Wheel Slip Variables
  const float w_f_ = 1.638f;
  const float w_r_ = 1.523f;
  const float l_f_ = 1.7328f;

  // rr, rl, fr, fl
  float v_x_per_wheel_[4]; // Forward speed of wheel (m / s)
  float v_w_per_wheel_[4]; // Wheel speed (m/s)?
  float k_per_wheel_[4];

  double omega_;
  double v_x_;
  double v_y_;
  float delta_; // Steering angle (rad)

  /*
    B. IMU
  */
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr
      imu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;

  std::deque<double> imu_sample_time_;

  /*
    C. Lap Time
  */
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      curvilenar_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lap_time_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  double lap_times_[2];
  int lap_times_index_ = 0;
  bool already_at_start_ = false;
  bool record_time_ = false;

  float last_lap_time_ = 0;
};
