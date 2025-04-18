#pragma once

#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <ctime>

struct WheelData {
  int OdomData, SpeedData, SteeringData;
  float longitudal_speed;
  float tangential_speed;
  float angular_velocity;
  float rr_wheel_speed;
  float rl_wheel_speed;
  float fr_wheel_speed;
  float fl_wheel_speed;
  float wheel_angle;
};

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void publish_ratios_if_ready();
  void rawimu_callback(novatel_oem7_msgs::msg::RAWIMU::SharedPtr rawimu_msg);
  void curvilinear_distance_callback(std_msgs::msg::Float32::SharedPtr curv_msg);
 
 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_subscriber;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  WheelData wheelData;

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr rawimu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;
  //using a deque to store only the timestamps I need
  std::deque<double> imu_timestamps;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;
  double lap_start_time;
  float lap_start_pos;
}
;