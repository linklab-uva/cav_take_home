#include "lap_time/lap_time_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

LapTimeNode::LapTimeNode(const rclcpp::NodeOptions& options)
    : Node("lap_time", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&LapTimeNode::odometry_callback, this, std::placeholders::_1));
    
    cl_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&LapTimeNode::cl_distance_callback, this, std::placeholders::_1));

    laptime_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "lap_time", qos_profile);
}

void LapTimeNode::cl_distance_callback(std_msgs::msg::Float32::ConstSharedPtr cl_distance_msg) {
  if(last_dist> cl_distance_msg->data) {
    int dt_s = odom_msg->header.stamp.sec-start_sec; //odom_msg shouldnt be null here, so no check
    int dt_ns = odom_msg->header.stamp.nanosec-start_nsec;
    float dt = float(dt_s) + float(dt_ns)/1000000.0;
    std_msgs::msg::Float32 time_diff;
    time_diff.data = dt;
    laptime_publisher_->publish(time_diff);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Starting new lap, lap time was: " << dt);
  }
  last_dist = cl_distance_msg->data;
}
void LapTimeNode::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  this->odom_msg = odom_msg;
  if(start_sec == 0) {
    start_sec = odom_msg->header.stamp.sec;
    start_nsec = odom_msg->header.stamp.nanosec;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LapTimeNode)
