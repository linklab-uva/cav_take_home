#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <vectornav_msgs/msg/imu_group.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg);
  void vectornav_imu_callback(vectornav_msgs::msg::ImuGroup::ConstSharedPtr imu_msg);
  void novatel_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vectornav_imu_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr novatel_imu_subscriber_;
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  
  // Slip ratio publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  
  // Jitter publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vectornav_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr novatel_jitter_publisher_;
  
  // Lap time publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;
  
  // Constants for slip ratio calculation
  const float w_f_ = 1.638;  // front track width in meters
  const float w_r_ = 1.523;  // rear track width in meters
  const float l_f_ = 1.7238;  // longitudinal distance from COG to front wheels in meters
  const float steering_ratio_ = 15.0;  // steering ratio
  
  // Store latest data for calculations
  float latest_vx_ = 0.0;
  float latest_vy_ = 0.0;
  float latest_omega_ = 0.0;
  float latest_steering_angle_ = 0.0;
  float latest_wheel_speed_fl_ = 0.0;
  float latest_wheel_speed_fr_ = 0.0;
  float latest_wheel_speed_rl_ = 0.0;
  float latest_wheel_speed_rr_ = 0.0;
  
  // Jitter calculation variables
  std::deque<double> vectornav_timestamps_;  // Store timestamps for vectornav jitter calculation
  std::deque<double> novatel_timestamps_;    // Store timestamps for novatel jitter calculation
  
  // Lap time tracking variables
  double lap_start_time_ = 0.0;  // Time when curvilinear distance was 0.0
  bool previous_value_was_zero_ = false;  // Flag to track if previous value was 0.0
  bool first_zero_seen_ = false;  // Flag to track if we've seen the first 0.0
  double last_curvilinear_distance_ = -1.0;  // Distance of the last curvilinear callback
  
  // Helper method to calculate slip ratios
  void calculate_and_publish_slip_ratios();
  
  // Helper methods to calculate jitter
  void calculate_and_publish_vectornav_jitter();
  void calculate_and_publish_novatel_jitter();
};
