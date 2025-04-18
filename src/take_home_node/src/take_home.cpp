#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <math.h>
#define REAR_TRACK_WIDTH 1.523
#define FRONT_TRACK_WIDTH 1.638
#define LONG_DISTANCE 1.7238
#define PI 3.14159
#define KMPH_TO_MPS 1000.0 / 3600.0;

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    wheelData.OdomData = 0;
    wheelData.SpeedData = 0;
    wheelData.SteeringData = 0;
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));
    steering_subscriber = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));
    wheel_subscriber = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_callback, this, std::placeholders::_1));
      slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
      slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
      slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
      slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
      rawimu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::rawimu_callback, this, std::placeholders::_1));
      jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu/jitter", qos_profile);

      curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
      lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
      //dummy start values, changed during execution
      lap_start_time = -1.0;
      lap_start_pos = -1.0;
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */

//store odometry data that I need
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float longitudal_speed = odom_msg->twist.twist.linear.x;
  float angular_velocity = odom_msg->twist.twist.angular.z;
  float tangential_speed = odom_msg->twist.twist.linear.y;
  wheelData.longitudal_speed = longitudal_speed;
  wheelData.angular_velocity = angular_velocity;
  wheelData.tangential_speed = tangential_speed;
  wheelData.OdomData = 1;
  publish_ratios_if_ready();
}

//store wheel speed data that I need
void TakeHome::wheel_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg){
  float rr_wheel_speed = wheel_msg->rear_right*KMPH_TO_MPS;
  float rl_wheel_speed = wheel_msg->rear_left*KMPH_TO_MPS;
  float fl_wheel_speed = wheel_msg->front_left*KMPH_TO_MPS;
  float fr_wheel_speed = wheel_msg->front_right*KMPH_TO_MPS;
  wheelData.fr_wheel_speed = fr_wheel_speed;
  wheelData.rr_wheel_speed = rr_wheel_speed;
  wheelData.rl_wheel_speed = rl_wheel_speed;
  wheelData.fl_wheel_speed = fl_wheel_speed;
  wheelData.SpeedData = 1;
  publish_ratios_if_ready();
}

//store steering data that I need
void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg){
  float steering_angle = steering_msg->primary_steering_angle_fbk*(PI/180)*(1/15);
  wheelData.wheel_angle = steering_angle;
  wheelData.SteeringData = 1;
  publish_ratios_if_ready();
}

//I made the choice to always publish if new data from a topic is ready, even if it means
//reusing data from older messages in other topics. Alternatively, 
//I could have chosen to wait for all 3 topics to get new data, then publish
void TakeHome::publish_ratios_if_ready(){
  //if all necessary data has arrived, then follow the instructions 
  if (wheelData.OdomData && wheelData.SpeedData && wheelData.SteeringData){
    //plugging in the formulas
    float v_x_rr = wheelData.longitudal_speed-0.5*wheelData.angular_velocity*REAR_TRACK_WIDTH;
    float rr_ratio = (wheelData.rr_wheel_speed - v_x_rr)/v_x_rr;
    float v_x_rl = wheelData.longitudal_speed+0.5*wheelData.angular_velocity*REAR_TRACK_WIDTH;
    float rl_ratio = (wheelData.rl_wheel_speed - v_x_rl)/v_x_rl;
    float v_x_fr = wheelData.longitudal_speed-0.5*wheelData.angular_velocity*FRONT_TRACK_WIDTH;
    float v_y_fr = wheelData.tangential_speed+wheelData.angular_velocity*LONG_DISTANCE;
    float v_delta_x_fr = cos(wheelData.wheel_angle)*v_x_fr - sin(wheelData.wheel_angle)*v_y_fr;
    float fr_ratio = (wheelData.fr_wheel_speed - v_delta_x_fr)/v_delta_x_fr;
    float v_x_fl = wheelData.longitudal_speed+0.5*wheelData.angular_velocity*FRONT_TRACK_WIDTH;
    float v_y_fl = wheelData.tangential_speed+wheelData.angular_velocity*LONG_DISTANCE;
    float v_delta_x_fl = cos(wheelData.wheel_angle)*v_x_fl - sin(wheelData.wheel_angle)*v_y_fl;
    float fl_ratio = (wheelData.fl_wheel_speed - v_delta_x_fl)/v_delta_x_fl;
    //getting messages ready
    std_msgs::msg::Float32 rl_msg;
    std_msgs::msg::Float32 fl_msg;
    std_msgs::msg::Float32 rr_msg;
    std_msgs::msg::Float32 fr_msg;
    fr_msg.data = fr_ratio;
    rr_msg.data = rr_ratio;
    fl_msg.data = fl_ratio;
    rl_msg.data = rl_ratio;
    slip_rl_publisher_->publish(rl_msg);
    slip_fl_publisher_->publish(fl_msg);
    slip_rr_publisher_->publish(rr_msg);
    slip_fr_publisher_->publish(fr_msg);
  }
}

void TakeHome::rawimu_callback(novatel_oem7_msgs::msg::RAWIMU::SharedPtr rawimu_msg) {
  double timestamp = rawimu_msg->header.stamp.sec + rawimu_msg->header.stamp.nanosec / 1e9; 
  
  //removing timestamps older than 1 second
  while (!imu_timestamps.empty() && timestamp - imu_timestamps.front() > 1.0) { 
    imu_timestamps.pop_front();
  }

  imu_timestamps.push_back(timestamp);

  //need at least 3 timestamps to calculate variance
  if (imu_timestamps.size() < 3) {
    //not enough data for variance calculation, I chose to not publish here
    return;
  }

  //calculate sum of squared deviations for sample var
  double sum_squared_deviations = 0.0;

  //get squared deviations for timestamps within a second
  for (auto it = imu_timestamps.begin(); it != imu_timestamps.end() - 1; ++it) {
    double next = *(it + 1); 
    double diff = *it - next;  
    sum_squared_deviations += diff * diff;
  }

  //sample_var_publishing
  float sample_var = static_cast<float>(sum_squared_deviations) / (imu_timestamps.size()-2);
  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = sample_var;
  jitter_publisher_->publish(jitter_msg);
}


void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::SharedPtr curv_msg){
  float curr_pos = curv_msg->data;
  rclcpp::Time now = this->now();
  double curr_time = now.seconds();
  
  //first message
  if (lap_start_pos == -1 && lap_start_time == -1){
    lap_start_pos = curr_pos;
    lap_start_time = curr_time;
  }
  //checking if we returned to the area near where we started the lap, with a margin of error
  else if (abs(curr_pos-lap_start_pos) < 3 && curr_time - lap_start_time > 5.0){
    float lap_time = curr_time - lap_start_time;
    lap_start_pos = curr_pos;
    lap_start_time = curr_time;
    std_msgs::msg::Float32 lap_msg;
    lap_msg.data = lap_time;
    lap_time_publisher_->publish(lap_msg);
  }
}
 
RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)