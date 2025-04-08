#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    // Subscribe to wheel speed reports from Raptor DBW
    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));
      
    // Subscribe to steering reports from Raptor DBW
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    // Subscribe to curvilinear distance for lap time calculation
    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
      
    // Subscribe to vectornav IMU data for jitter calculation
    vectornav_imu_subscriber_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
      "/vectornav/raw/imu", qos_profile,
      std::bind(&TakeHome::vectornav_imu_callback, this, std::placeholders::_1));

    // Original metric publisher
    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    
    // Create publishers for slip ratios
    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
    
    // Create publisher for vectornav jitter
    vectornav_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);
    
    // Create publisher for lap time
    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Store values for slip ratio calculation
  latest_vx_ = odom_msg->twist.twist.linear.x;   // Longitudinal velocity
  latest_vy_ = odom_msg->twist.twist.linear.y;   // Lateral velocity
  latest_omega_ = odom_msg->twist.twist.angular.z;  // Yaw rate (angular velocity around z-axis)

  // Example (original code)
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
  
  calculate_and_publish_slip_ratios();
}

/**
 * Callback for wheel speed reports
 * Stores wheel speeds for all four wheels in radians per second
 */
void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
  // Store wheel speeds in rad/sec for slip ratio calculation
  latest_wheel_speed_fl_ = wheel_speed_msg->front_left;
  latest_wheel_speed_fr_ = wheel_speed_msg->front_right;
  latest_wheel_speed_rl_ = wheel_speed_msg->rear_left;
  latest_wheel_speed_rr_ = wheel_speed_msg->rear_right;
  
  // Calculate and publish slip ratios whenever we receive new wheel speed data
  calculate_and_publish_slip_ratios();
}

/**
 * Callback for steering reports
 * Extracts and converts the steering angle for slip ratio calculations
 */
void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  // Extract the steering angle and convert from degrees to radians
  // The steering angle needs to be divided by the steering ratio to get the wheel angle
  latest_steering_angle_ = (steering_msg->primary_steering_angle_fbk / steering_ratio_) * M_PI / 180.0;
  
  // Calculate and publish slip ratios whenever we receive new steering data
  calculate_and_publish_slip_ratios();
}

/**
 * Callback for curvilinear distance messages
 * Calculates lap time when the vehicle completes a full lap
 */
void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg) {
  float curvilinear_distance = curvilinear_msg->data;
  
  // Check if we're at distance 0.0 (beginning of a lap)
  bool is_zero = std::abs(curvilinear_distance) < 1e-6;
  
  // If we're at 0.0 and the previous value wasn't 0.0, then we've completed a lap
  if (is_zero && !previous_value_was_zero_) {
    // Get the current time
    double current_time = this->now().seconds();
    
    // If we've already seen a starting point (first 0.0), calculate lap time
    if (first_zero_seen_) {
      // Calculate lap time
      double lap_time = current_time - lap_start_time_;
      
      // Publish lap time
      std_msgs::msg::Float32 lap_time_msg;
      lap_time_msg.data = static_cast<float>(lap_time);
      lap_time_publisher_->publish(lap_time_msg);
    } else {
      // This is the first time we've seen 0.0, so mark this point
      first_zero_seen_ = true;
    }
    
    // Update lap start time for next lap
    lap_start_time_ = current_time;
  }
  
  // Update previous value flag
  previous_value_was_zero_ = is_zero;
}

/**
 * Callback for vectornav IMU messages
 * Tracks timestamps and calculates jitter using a sliding window
 */
void TakeHome::vectornav_imu_callback(vectornav_msgs::msg::ImuGroup::ConstSharedPtr imu_msg) {
  // Extract timestamp from the message
  double current_timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9;
  
  // Store timestamp in the deque
  vectornav_timestamps_.push_back(current_timestamp);
  
  // Remove timestamps older than 1 second from the current timestamp
  while (!vectornav_timestamps_.empty() && 
         (current_timestamp - vectornav_timestamps_.front() > 1.0)) {
    vectornav_timestamps_.pop_front();
  }
  
  // Calculate jitter if we have at least 2 timestamps
  if (vectornav_timestamps_.size() >= 2) {
    calculate_and_publish_vectornav_jitter();
  }
}

/**
 * Calculate jitter as the variance of time intervals and publish the result
 */
void TakeHome::calculate_and_publish_vectornav_jitter() {
  if (vectornav_timestamps_.size() < 2) {
    return;  // Need at least 2 timestamps to calculate deltas
  }
  
  // Calculate time deltas between consecutive timestamps
  std::vector<double> deltas;
  for (size_t i = 1; i < vectornav_timestamps_.size(); ++i) {
    deltas.push_back(vectornav_timestamps_[i] - vectornav_timestamps_[i-1]);
  }
  
  // Calculate mean of deltas
  double sum = 0.0;
  for (const double& dt : deltas) {
    sum += dt;
  }
  double mean = sum / deltas.size();
  
  // Calculate variance (jitter)
  double variance = 0.0;
  for (const double& dt : deltas) {
    double diff = dt - mean;
    variance += diff * diff;
  }
  variance /= deltas.size();
  
  // Publish jitter (variance)
  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = static_cast<float>(variance);
  vectornav_jitter_publisher_->publish(jitter_msg);
}

/**
 * Calculate slip ratios for all four wheels and publish them to their respective topics
 * Uses the formulas provided in the assignment description
 */
void TakeHome::calculate_and_publish_slip_ratios() {
  // Create messages for each wheel's slip ratio
  std_msgs::msg::Float32 slip_rr_msg;
  std_msgs::msg::Float32 slip_rl_msg;
  std_msgs::msg::Float32 slip_fr_msg;
  std_msgs::msg::Float32 slip_fl_msg;
  
  // Calculate rear wheel slip ratios
  
  // Rear Right wheel
  float v_x_rr = latest_vx_ - 0.5 * latest_omega_ * w_r_;
  float kappa_rr = 0.0;
  
  // Avoid division by zero
  if (std::abs(v_x_rr) > 1e-6) {
    kappa_rr = (latest_wheel_speed_rr_ - v_x_rr) / v_x_rr;
  }
  
  // Rear Left wheel
  float v_x_rl = latest_vx_ + 0.5 * latest_omega_ * w_r_;
  float kappa_rl = 0.0;
  
  // Avoid division by zero
  if (std::abs(v_x_rl) > 1e-6) {
    kappa_rl = (latest_wheel_speed_rl_ - v_x_rl) / v_x_rl;
  }
  
  // Calculate front wheel slip ratios
  
  // Front Right wheel
  float v_x_fr = latest_vx_ - 0.5 * latest_omega_ * w_f_;
  float v_y_fr = latest_vy_ + latest_omega_ * l_f_;
  float v_x_fr_delta = cos(latest_steering_angle_) * v_x_fr - sin(latest_steering_angle_) * v_y_fr;
  float kappa_fr = 0.0;
  
  // Avoid division by zero
  if (std::abs(v_x_fr_delta) > 1e-6) {
    kappa_fr = (latest_wheel_speed_fr_ - v_x_fr_delta) / v_x_fr_delta;
  }
  
  // Front Left wheel
  float v_x_fl = latest_vx_ + 0.5 * latest_omega_ * w_f_;
  float v_y_fl = latest_vy_ + latest_omega_ * l_f_;
  float v_x_fl_delta = cos(latest_steering_angle_) * v_x_fl - sin(latest_steering_angle_) * v_y_fl;
  float kappa_fl = 0.0;
  
  // Avoid division by zero
  if (std::abs(v_x_fl_delta) > 1e-6) {
    kappa_fl = (latest_wheel_speed_fl_ - v_x_fl_delta) / v_x_fl_delta;
  }
  
  // Assign calculated values to messages
  slip_rr_msg.data = kappa_rr;
  slip_rl_msg.data = kappa_rl;
  slip_fr_msg.data = kappa_fr;
  slip_fl_msg.data = kappa_fl;
  
  // Publish slip ratio messages to their respective topics
  slip_rr_publisher_->publish(slip_rr_msg);
  slip_rl_publisher_->publish(slip_rl_msg);
  slip_fr_publisher_->publish(slip_fr_msg);
  slip_fl_publisher_->publish(slip_fl_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
