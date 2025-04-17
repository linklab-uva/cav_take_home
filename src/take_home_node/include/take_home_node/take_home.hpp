#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "raptor_dbw_msgs/msg/steering_extended_report.hpp"
#include "novatel_oem7_msgs/msg/rawimu.hpp"


#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  void wheel_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg);

  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);

  void metrics_calculation();

  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

  void jitter_processing();

  void lap_time_process();

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fl_publisher_;

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_publisher_;

};
