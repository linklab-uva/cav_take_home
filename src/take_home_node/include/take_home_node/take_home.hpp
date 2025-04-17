#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

  void calculate_wheel_slip_ratio();
  void calculate_imu_jitter();


 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;


  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr last_wheel_speed_;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr last_steering_;


  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_jitter_publisher_;
  std::deque<rclcpp::Time> imu_timestamps_;


  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;
  bool previous_value_was_zero_ = false;
  bool first_zero_seen_ = false;
  double lap_start_time_ = 0.0;
  float last_curvilinear_distance_ = -1.0;
};
