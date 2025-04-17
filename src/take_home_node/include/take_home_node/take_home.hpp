#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  void wheelspeed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheelspeed_msg);

  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);

  bool publish_slip_ratios();

 private:
  const float w_f = 1.638;
  const float w_r = 1.523;
  const float l_f = 1.7238;

  
  // Store last message from each subscriber
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg = NULL;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheelspeed_msg = NULL;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg = NULL;
  // float steering_angle = 0;

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheelspeed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_;

};
