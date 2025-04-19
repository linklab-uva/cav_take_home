#include "imu_jitter/jitter_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

JitterNode::JitterNode(const rclcpp::NodeOptions& options)
    : Node("imu_jitter", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&JitterNode::imu_callback, this, std::placeholders::_1));

    jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "imu_top/jitter", qos_profile);
}

double JitterNode::get_time(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  return (static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec)/1e9f);
}

float JitterNode::get_variance() {
  double meandt = (get_time(msgArr[msgArr.size()-1]) - get_time(msgArr[0]))/(msgArr.size()-1);
  double smsq = 0.0;
  for(size_t i=1; i<msgArr.size(); i++) {
    double dt = get_time(msgArr[i]) - get_time(msgArr[i-1]);
    smsq += (dt-meandt)*(dt-meandt);
  }
  return static_cast<float>(smsq/(msgArr.size()-2));
}

void JitterNode::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  msgArr.push_back(imu_msg);
  if(msgArr.size()<=2) return; //variance of <=1 sample is undefined 
  int i = 0;
  while(get_time(imu_msg) - get_time(msgArr[i])> 1.0) i++;
  msgArr.erase(msgArr.begin(), msgArr.begin()+i); 
  float variance = get_variance();
  std_msgs::msg::Float32 vmsg;
  vmsg.data = variance;
  jitter_publisher_->publish(vmsg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JitterNode)
