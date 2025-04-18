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


void JitterNode::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  JitterNode::msgArr.push_back(imu_msg);
  if(JitterNode::msgArr.size()<=2) return;
  int i = 0;
  while(imu_msg->gnss_seconds-JitterNode::msgArr[i]->gnss_seconds > 1) i++;
  msgArr.erase(JitterNode::msgArr.begin(), JitterNode::msgArr.begin()+i); 
  float variance = get_variance();
  std_msgs::msg::Float32 vmsg;
  vmsg.data = variance;
  jitter_publisher_->publish(vmsg);
}

float JitterNode::get_variance() {
  auto ar = JitterNode::msgArr; 
  float meandt = (ar[ar.size()-1]->gnss_seconds - ar[0]->gnss_seconds)/(ar.size()-1);
  float smsq = 0.0;
  for(size_t i=1; i<ar.size(); i++) {
    float dt = ar[i]->gnss_seconds-ar[i-1]->gnss_seconds;
    smsq += (dt-meandt)*(dt-meandt);
  }
  return smsq/(ar.size()-2);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JitterNode)
