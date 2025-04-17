#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <queue>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

std::vector<bool> data_populated(3, false);

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_callback, this, std::placeholders::_1));
    
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

    jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);

    lap_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
/*void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
}*/

// TASK A ===================================================================================

struct {
  float pos_x;
  float pos_y;
  float pos_z;
  tf2::Quaternion q;
  float vel_x;
  float vel_y;
  float vel_z;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
} odom;

struct {
  float fl;
  float fr;
  float rl;
  float rr; 
} wheel;

float steering_angle; //deg

const float w_r = 1.523; //width between rear tires [m]
const float w_f = 1.638; //width between front tires [m]
const float l_f = 1.7238; //distance from cog to front wheels


void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  //RCLCPP_INFO(this->get_logger(), "odometry callback");
  odom.pos_x = odom_msg->pose.pose.position.x;
  odom.pos_y = odom_msg->pose.pose.position.y;
  odom.pos_z = odom_msg->pose.pose.position.z;
  odom.q.setX(odom_msg->pose.pose.orientation.x);
  odom.q.setY(odom_msg->pose.pose.orientation.y);
  odom.q.setZ(odom_msg->pose.pose.orientation.z);
  odom.q.setW(odom_msg->pose.pose.orientation.w);

  odom.vel_x = odom_msg->twist.twist.linear.x; //forward linear speed [m/s]
  odom.vel_y = odom_msg->twist.twist.linear.y; //lateral speed [m/s]
  odom.vel_z = odom_msg->twist.twist.linear.z;
  odom.roll_rate = odom_msg->twist.twist.angular.x; //yaw rate [rad/s]
  odom.pitch_rate = odom_msg->twist.twist.angular.y;
  odom.yaw_rate = odom_msg->twist.twist.angular.z; 

  data_populated[0] = true;
  metrics_calculation();
  lap_time_process();
}

void TakeHome::wheel_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg) {
  //Wheel Speeds in rad/sec
  //RCLCPP_INFO(this->get_logger(), "wheel callback");

  wheel.fl = wheel_msg->front_left/3.6; //get values in m/s
  wheel.fr = wheel_msg->front_right/3.6;
  wheel.rl = wheel_msg->rear_left/3.6;
  wheel.rr = wheel_msg->rear_right/3.6;

  data_populated[1] = true;
  metrics_calculation();
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  //RCLCPP_INFO(this->get_logger(), "steering callback");
  steering_angle = (steering_msg->primary_steering_angle_fbk)/15.0;
  data_populated[2] = true;
  metrics_calculation();
}

void TakeHome::metrics_calculation() {

  for(int i = 0; i < 3; i++) { //wait to receive data from all topics at least once
    if(data_populated[i] == false)
      return;
  }
  //RCLCPP_INFO(this->get_logger(), "vel_x %f", odom.vel_x);
  //RCLCPP_INFO(this->get_logger(), "yaw rate %f", odom.yaw_rate);
  //RCLCPP_INFO(this->get_logger(), "wheel speed %f", wheel.rr);
  float v_x_rr = odom.vel_x - 0.5*odom.yaw_rate*w_r;
  float k_rr = (wheel.rr - v_x_rr)/v_x_rr; //wheel slip ratio for rear right
  //RCLCPP_INFO(this->get_logger(), "v_x_rr %f", v_x_rr);

  float v_x_rl = odom.vel_x + 0.5*odom.yaw_rate*w_r;
  float k_rl = (wheel.rl - v_x_rl)/v_x_rl; //wheel slip ratio for rear left

  float v_x_fr = odom.vel_x - 0.5*odom.yaw_rate*w_f;
  float v_y_fr = odom.vel_y + odom.yaw_rate*l_f;
  float v_d_x_fr = cos(steering_angle)*v_x_fr-sin(steering_angle)*v_y_fr;
  float k_fr = (wheel.fr-v_d_x_fr)/v_d_x_fr; //wheel slip ratio for front right

  float v_x_fl = odom.vel_x + 0.5*odom.yaw_rate*w_f;
  float v_y_fl = odom.vel_y + odom.yaw_rate*l_f;
  float v_d_x_fl = cos(steering_angle)*v_x_fl-sin(steering_angle)*v_y_fl;
  float k_fl = (wheel.fl-v_d_x_fl)/v_d_x_fl; //wheel slip ratio for front left


  std_msgs::msg::Float32 rr_msg;
  std_msgs::msg::Float32 rl_msg;
  std_msgs::msg::Float32 fr_msg;
  std_msgs::msg::Float32 fl_msg;
  
  rr_msg.data = k_rr;
  rl_msg.data = k_rl;
  fr_msg.data = k_fr;
  fl_msg.data = k_fl;

  rr_publisher_->publish(rr_msg);
  rl_publisher_->publish(rl_msg);
  fr_publisher_->publish(fr_msg);
  fl_publisher_->publish(fl_msg);
}

// TASK B ===================================================================================

std::queue<long> imu_timestamps;
std::queue<int> imu_dt;
int running_sum;
int init_time_s;
int n;

void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
  //RCLCPP_INFO(this->get_logger(), "imu callback");
  if(n==0) {
    init_time_s = imu_msg->header.stamp.sec;
  }
  int latest_timestamp_s = imu_msg->header.stamp.sec - init_time_s; //essentially a tare
  int latest_timestamp_ns = imu_msg->header.stamp.nanosec;
  long latest_timestamp = latest_timestamp_s*pow(10,9) + latest_timestamp_ns; //convert messages to time in nanoseconds
  int dt;

  //RCLCPP_INFO(this->get_logger(), "timestamp queue: %ld", imu_timestamps.size());
  //RCLCPP_INFO(this->get_logger(), "dt queue: %ld", imu_dt.size());
  n++;
  //dont do anything until two data points
  if(n==2) { //if exactly two data points
    dt = latest_timestamp - imu_timestamps.back();
    imu_timestamps.push(latest_timestamp);
    imu_dt.push(dt);
    running_sum = dt;
  }
  else if(n > 2) { //if more than two data points, process jitter
    dt = latest_timestamp - imu_timestamps.back();
    imu_timestamps.push(latest_timestamp);
    imu_dt.push(dt);
    running_sum += dt;
    jitter_processing();
  }
  else {
      imu_timestamps.push(latest_timestamp);
  }

  //RCLCPP_INFO(this->get_logger(), "dt: %ld", dt);
}

void TakeHome::jitter_processing() {
  //int init_n = n;
  int latest_t = imu_timestamps.back();
  int front_t = 0;
  int front_dt = 0;
  double mean = 0.0;
  double sum_error_squared = 0.0;
  double variance = 0.0;

  front_t = imu_timestamps.front();
  front_dt = imu_dt.front();
  
  while(front_t < (latest_t-pow(10,9))) {//check if front time is more than one second behind, if yes then delete that data since it is outside interval
    running_sum -= front_dt;
    //RCLCPP_INFO(this->get_logger(), "Removing: %d, %d", front_t, front_dt);
    //RCLCPP_INFO(this->get_logger(), "Latest: %d", latest_t);
    imu_timestamps.pop();
    imu_dt.pop();
    front_t = imu_timestamps.front();
    front_dt = imu_dt.front();
    n--;

  }
//queues will only contain values within 1 second of latest time stamp
  mean = double(running_sum)/(n-1);
  for(int i = 0; i < n-1; i++) { //cycles queue, processes sum of squard errors
    front_dt = imu_dt.front();
    sum_error_squared += pow((front_dt - mean),2);
    imu_dt.pop();
    imu_dt.push(front_dt); 
  }
  variance = sum_error_squared/(n-2); //uses n-2 because there are n time stamps, and n-1 dt data, so formula would use n-2

  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = variance/(pow(10,18)); //convert nanoseconds^2 to seconds^2
  jitter_publisher_->publish(jitter_msg);
}

// TASK C ================================================================================================================
float initial_x;
float initial_y;
double init_yaw, init_roll, init_pitch;
float initial_dydx;
bool initial_data_taken = false;

float prev_x;
float prev_y;

float curr_x;
float curr_y;

float eval_curr;
float eval_prev;
float dist;

unsigned long start_time;
unsigned long curr_time;
float lap_time;


void TakeHome::lap_time_process() {

  if(!initial_data_taken) {
    if(imu_timestamps.size() > 0) {
      initial_x = odom.pos_x;
      initial_y = odom.pos_y;

      prev_x = initial_x;
      prev_y = initial_y;
      //prev_yaw = initial_yaw;
      tf2::Matrix3x3(odom.q).getRPY(init_roll,init_pitch,init_yaw);
      initial_dydx = -1/tan(init_yaw);
      RCLCPP_INFO(this->get_logger(), "Initial x: %f", initial_x);
      RCLCPP_INFO(this->get_logger(), "Initial y: %f", initial_y);
      RCLCPP_INFO(this->get_logger(), "Initial dy/dx: %f", initial_dydx);

      start_time = imu_timestamps.back();

      initial_data_taken = true;
    }
  }
  else {
    curr_x = odom.pos_x;
    curr_y = odom.pos_y;
    curr_time = imu_timestamps.back();

    eval_curr = initial_dydx*(curr_x-initial_x)-(curr_y-initial_y);
    eval_prev = initial_dydx*(prev_x-initial_x)-(prev_y-initial_y);
    dist = pow(pow(curr_x-initial_x,2) + pow(curr_y-initial_y,2),0.5);
    //RCLCPP_INFO(this->get_logger(), "Data: %f, %f, %f", eval_curr, eval_prev, dist);

    if(eval_curr >= 0 && eval_prev <= 0 && dist <= 20 /*depends on track width data*/) {
      lap_time = float((curr_time - start_time)/pow(10,9));
      if(lap_time > 1) {
        RCLCPP_INFO(this->get_logger(), "LAP COMPLETE");
        RCLCPP_INFO(this->get_logger(), "Prev: %f, %f, %f", prev_x,prev_y,eval_prev);
        RCLCPP_INFO(this->get_logger(), "Curr: %f, %f, %f", curr_x,curr_y, eval_curr);
        RCLCPP_INFO(this->get_logger(), "Start Time: %ld", start_time);
        RCLCPP_INFO(this->get_logger(), "End Time: %ld", curr_time);
        std_msgs::msg::Float32 lap_msg;
        lap_msg.data = lap_time;
        RCLCPP_INFO(this->get_logger(), "Lap Time: %f", lap_time);
        lap_publisher_->publish(lap_msg);

        start_time = curr_time;
      }
    }
    
    prev_x = curr_x;
    prev_y = curr_y;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
