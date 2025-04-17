#include "wheel_slip_node/slip_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

SlipNode::SlipNode(const rclcpp::NodeOptions& options)
    : Node("wheel_slip_ratio", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`SlipNode::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&SlipNode::odometry_callback, this, std::placeholders::_1));

    wheelspeed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile, 
      std::bind(&SlipNode::wheelspeed_callback, this, std::placeholders::_1));
    
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile, 
      std::bind(&SlipNode::steering_callback, this, std::placeholders::_1));
    
    slip_rr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void SlipNode::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  SlipNode::odom_msg = odom_msg;
  publish_slip_ratios();
}

void SlipNode::wheelspeed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheelspeed_msg) {
  SlipNode::wheelspeed_msg = wheelspeed_msg;
}

void SlipNode::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  SlipNode::steering_msg = steering_msg;
}

bool SlipNode::publish_slip_ratios() {
  if(SlipNode::wheelspeed_msg == NULL || SlipNode::steering_msg == NULL || SlipNode::odom_msg == NULL) return false;
  float v_x = SlipNode::odom_msg->twist.twist.linear.x;
  float yaw = SlipNode::odom_msg->twist.twist.angular.z;

  //rear right
  float v_xrr = v_x-0.5*yaw*SlipNode::w_r;
  float vw_rr = SlipNode::wheelspeed_msg->rear_right*0.277778; //kmph -> m/s
  float k_rr = (vw_rr-v_xrr)/v_xrr;

  //rear left
  float v_xrl = v_x+0.5*yaw*SlipNode::w_r;
  float vw_rl = SlipNode::wheelspeed_msg-> rear_left*0.277778;
  float k_rl = (vw_rl-v_xrl)/v_xrl;
  
  //front right
  float delta = (SlipNode::steering_msg->primary_steering_angle_fbk * 0.01745329)/15.0; // deg->rad
  float v_y = SlipNode::odom_msg->twist.twist.linear.y;
  float v_xfr = v_x - 0.5*yaw*SlipNode::w_f;
  float v_yf = v_y + yaw*SlipNode::l_f;
  float vd_xfr = cos(delta)*v_xfr - sin(delta)*v_yf;
  float vw_fr = SlipNode::wheelspeed_msg->front_right*0.277778;
  float k_fr = (vw_fr-vd_xfr)/vd_xfr;

  //front left
  float v_xfl = v_x + 0.5*yaw*SlipNode::w_f;
  float vd_xfl = cos(delta)*v_xfl - sin(delta)*v_yf;
  float vw_fl = SlipNode::wheelspeed_msg->front_left*0.277778;
  float k_fl = (vw_fl-vd_xfl)/vd_xfl;

  std_msgs::msg::Float32 slip_ratio;
  slip_ratio.data = k_rr;
  SlipNode::slip_rr_->publish(slip_ratio);
  slip_ratio.data = k_rl;
  SlipNode::slip_rl_->publish(slip_ratio);
  slip_ratio.data = k_fr;
  SlipNode::slip_fr_->publish(slip_ratio);
  slip_ratio.data = k_fl;
  SlipNode::slip_fl_->publish(slip_ratio);

  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SlipNode)
