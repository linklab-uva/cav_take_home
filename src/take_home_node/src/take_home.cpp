#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto qos_profile_imu = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos_profile,
        std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", qos_profile_imu,
        std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));    

    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/curvilinear_distance", qos_profile,
        std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));    

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
    imu_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/imu_top/jitter", qos_profile_imu);
    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/lap_time", qos_profile);
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

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

  last_odom_ = odom_msg;
}


void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
    last_wheel_speed_ = wheel_speed_msg;
    calculate_wheel_slip_ratio();
}


void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
    last_steering_ = steering_msg;
    calculate_wheel_slip_ratio();
}


void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
    rclcpp::Time now = msg->header.stamp;
    
    imu_timestamps_.push_back(now);

    // removing timestamps older than 1 second
    while (!imu_timestamps_.empty() && (now - imu_timestamps_.front()).seconds() > 1.0) {
        imu_timestamps_.pop_front();
    }

    calculate_imu_jitter();
}


void TakeHome::curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
    float d = msg->data;
    bool is_zero = std::abs(d) < 1e-6;

    // handle bag‑file jumps
    if (!is_zero && last_curvilinear_distance_ >= 0.0) {
    double jump = std::abs(d - last_curvilinear_distance_);
    if (jump > 5.0) {
        first_zero_seen_ = false;
        last_curvilinear_distance_ = -1.0;
    }
    }

    if (is_zero && !previous_value_was_zero_) {
    double now_s = this->now().seconds();
    if (first_zero_seen_) {
        double lap = now_s - lap_start_time_;
        std_msgs::msg::Float32 lap_msg; lap_msg.data = lap;
        lap_time_publisher_->publish(lap_msg);
    } else {
        first_zero_seen_ = true;
    }
    lap_start_time_ = now_s;
    }

    previous_value_was_zero_ = is_zero;
    last_curvilinear_distance_ = d;


}


void TakeHome::calculate_wheel_slip_ratio() {
    if (!last_odom_ || !last_wheel_speed_ || !last_steering_) {
        return;
      } 
    
    const double w_r = 1.523;
    const double w_f = 1.638;
    const double l_f = 1.7238;
    const float kmh_to_ms_factor = 3.6;

    // calculating rear right wheel slip
    double v_x = last_odom_->twist.twist.linear.x;
    double omega = last_odom_->twist.twist.angular.z;
    double v_x_rr = v_x - (0.5 * omega * w_r);
    float v_w_rr = last_wheel_speed_->rear_right / kmh_to_ms_factor; 
    
    double k_rr;
    if (v_x_rr != 0.0) {
        k_rr = (v_w_rr - v_x_rr) / v_x_rr; 
    } else {
        k_rr = 0.0;
    }

    std_msgs::msg::Float32 slip_rr_msg;
    slip_rr_msg.data = k_rr;
    slip_rr_publisher_->publish(slip_rr_msg);
    
    // calculating rear left wheel slip
    double v_x_rl = v_x + (0.5 * omega * w_r);
    float v_w_rl = last_wheel_speed_->rear_left / kmh_to_ms_factor; 
    
    double k_rl;
    if (v_x_rl != 0.0) {
        k_rl = (v_w_rl - v_x_rl) / v_x_rl;
    } else {
        k_rl = 0.0;
    }

    std_msgs::msg::Float32 slip_rl_msg;
    slip_rl_msg.data = k_rl;
    slip_rl_publisher_->publish(slip_rl_msg);

    // calculating front right wheel slip
    double v_x_fr = v_x - (0.5 * omega * w_f);
    double v_y = last_odom_->twist.twist.linear.y; 
    double v_y_fr = v_y + omega * l_f;

    const float STEERING_RATIO = 15.0;
    float steering_angle_msg = last_steering_->primary_steering_angle_fbk;
    double delta_rad = (steering_angle_msg / STEERING_RATIO) * (M_PI / 180.0);
    double v_x_fr_delta = cos(delta_rad) * v_x_fr - sin(delta_rad) * v_y_fr;
    float v_w_fr = last_wheel_speed_->front_right / kmh_to_ms_factor; 

    double k_fr;
    if (v_x_fr_delta != 0.0) {
        k_fr = (v_w_fr - v_x_fr_delta) / v_x_fr_delta;
    } else {
        k_fr = 0.0;
    }

    std_msgs::msg::Float32 slip_fr_msg;
    slip_fr_msg.data = k_fr;
    slip_fr_publisher_->publish(slip_fr_msg);

    // calculating front left wheel slip
    double v_x_fl = v_x + (0.5 * omega * w_f);
    double v_y_fl = v_y + omega * l_f;

    double v_x_fl_delta = cos(delta_rad) * v_x_fl - sin(delta_rad) * v_y_fl;
    float v_w_fl = last_wheel_speed_->front_left / kmh_to_ms_factor; 

    double k_fl;
    if (v_x_fl_delta != 0.0) {
        k_fl = (v_w_fl - v_x_fl_delta) / v_x_fl_delta;
    } else {
        k_fl = 0.0;
    }

    std_msgs::msg::Float32 slip_fl_msg;
    slip_fl_msg.data = k_fl;
    slip_fl_publisher_->publish(slip_fl_msg);
}


void TakeHome::calculate_imu_jitter() {
    if (imu_timestamps_.size() < 2) return;

    std::vector<double> deltas;
    for (size_t i = 1; i < imu_timestamps_.size(); ++i) {
        double dt = (imu_timestamps_[i] - imu_timestamps_[i - 1]).seconds();
        deltas.push_back(dt);
    }

    double mean = 0.0;
    for (double dt : deltas) {
        mean += dt;
    }
    mean /= deltas.size();

    double variance = 0.0;
    for (double dt : deltas) {
        variance += (dt - mean) * (dt - mean);
    }
    variance /= deltas.size();
    double jitter = std::sqrt(variance);

    std_msgs::msg::Float32 jitter_msg;
    jitter_msg.data = static_cast<float>(jitter);
    imu_jitter_publisher_->publish(jitter_msg);

}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
