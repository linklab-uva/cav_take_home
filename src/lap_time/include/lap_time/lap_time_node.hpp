#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class LapTimeNode : public rclcpp::Node {
  public:
    LapTimeNode(const rclcpp::NodeOptions& options);
    void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void cl_distance_callback(std_msgs::msg::Float32::ConstSharedPtr cl_distance_msg);

 private:
  int start_sec = 0;
  int start_nsec = 0;
  float last_dist = -1;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg = NULL;
  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cl_distance_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr laptime_publisher_;

};
