#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class JitterNode : public rclcpp::Node {
 public:
  JitterNode(const rclcpp::NodeOptions& options);

  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

 private:

  double get_time(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  
  float get_variance();
  // Stores imu msgs within last second
  std::vector<novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr> msgArr;

  // Subscribers and Publishers
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;

};
