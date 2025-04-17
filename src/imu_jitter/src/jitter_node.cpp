#include "imu_jitter/jitter_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

JitterNode::JitterNode(const rclcpp::NodeOptions& options)
    : Node("imu_jitter", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&JitterNode::imu_callback, this, std::placeholders::_1));
}


void JitterNode::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  
}

RCLCPP_COMPONENTS_REGISTER_NODE(JitterNode)
