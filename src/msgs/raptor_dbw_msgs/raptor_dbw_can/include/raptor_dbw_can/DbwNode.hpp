// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RAPTOR_DBW_CAN__DBWNODE_HPP_
#define RAPTOR_DBW_CAN__DBWNODE_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <deep_orange_msgs/msg/brake_temp_report.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pdu_msgs/msg/relay_command.hpp>
#include <pdu_msgs/msg/relay_state.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_extd_report.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <raptor_dbw_msgs/msg/dash_switches.hpp>
#include <raptor_dbw_msgs/msg/diagnostic_report.hpp>
#include <raptor_dbw_msgs/msg/driver_input_report.hpp>
#include <raptor_dbw_msgs/msg/fault_actions_report.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include <raptor_dbw_msgs/msg/gear_report.hpp>
#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>
#include <raptor_dbw_msgs/msg/hmi_global_enable_report.hpp>
#include <raptor_dbw_msgs/msg/low_voltage_system_report.hpp>
#include <raptor_dbw_msgs/msg/misc_cmd.hpp>
#include <raptor_dbw_msgs/msg/misc_report.hpp>
#include <raptor_dbw_msgs/msg/motec_report.hpp>
#include <raptor_dbw_msgs/msg/steering2_report.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/surround_report.hpp>
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
// #include <deep_orange_msgs/msg/pos_time.hpp>
#include <cmath>
#include <string>
#include <vector>

#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <deep_orange_msgs/msg/base_to_car_timing.hpp>
#include <deep_orange_msgs/msg/coordinates.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/tire_report.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "raptor_dbw_can/dispatch.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace raptor_dbw_can {
class DbwNode : public rclcpp::Node {
 public:
  explicit DbwNode(const rclcpp::NodeOptions& options);
  ~DbwNode();

 private:
  void timerTireCallback();
  void timerPtCallback();
  void timerMotecCallback();
  void timerBrakeCallback();
  void timerSteeringCallback();
  void recvCAN0(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCAN1(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCAN2(const can_msgs::msg::Frame::SharedPtr msg);
  void recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg);
  void recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg);
  void recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg);
  void recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg);
  void recvCtReport(const deep_orange_msgs::msg::CtReport::SharedPtr msg);
  void recvDashSwitches(const raptor_dbw_msgs::msg::DashSwitches::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_tire_report_;
  rclcpp::TimerBase::SharedPtr timer_pt_report_;
  rclcpp::TimerBase::SharedPtr timer_motec_report_;
  rclcpp::TimerBase::SharedPtr timer_brake_report_;
  rclcpp::TimerBase::SharedPtr timer_steering_report_;

  deep_orange_msgs::msg::PtReport pt_report_msg;
  deep_orange_msgs::msg::TireReport tire_report_msg;
  raptor_dbw_msgs::msg::MotecReport motec_report_msg;
  raptor_dbw_msgs::msg::BrakeExtdReport brake_report_msg;
  raptor_dbw_msgs::msg::SteeringExtendedReport steering_report_msg;

  // Subscribed topics
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can0_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can1_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can2_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::GearCmd>::SharedPtr sub_gear_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_shift_cmd_;
  rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr sub_ct_report_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::DashSwitches>::SharedPtr sub_dash_switches_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;  // acc pedal report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr pub_steering_;             // steering report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr pub_steering_ext_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr pub_wheel_speeds_;  // wheelspeedreport do
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_engine_kill_;

  rclcpp::Publisher<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr pub_brake_2_report_;        // brake report do
  rclcpp::Publisher<raptor_dbw_msgs::msg::BrakeExtdReport>::SharedPtr pub_brake_extd_report_;  // brake report do
  rclcpp::Publisher<deep_orange_msgs::msg::MiscReport>::SharedPtr pub_misc_do_;
  rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr pub_rc_to_ct_;
  rclcpp::Publisher<deep_orange_msgs::msg::BaseToCarTiming>::SharedPtr pub_base_to_car_timing_;
  rclcpp::Publisher<deep_orange_msgs::msg::BrakeTempReport>::SharedPtr pub_brake_temp_report_;
  rclcpp::Publisher<deep_orange_msgs::msg::TireReport>::SharedPtr pub_tire_report_;
  rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr pub_pt_report_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::DiagnosticReport>::SharedPtr pub_diagnostic_report_;
  rclcpp::Publisher<raptor_dbw_msgs::msg::MotecReport>::SharedPtr pub_motec_report_;

  NewEagle::Dbc dbwDbc_can0_;
  NewEagle::Dbc dbwDbc_can1_;
  NewEagle::Dbc dbwDbc_can2_;
  std::string dbcFile_can0_;
  std::string dbcFile_can1_;
  std::string dbcFile_can2_;
  int ct_state = 0;
  int defender_speed = 80;  // Initialize to 1st round defender speed
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DBWNODE_HPP_
