// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/DbwNode.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace raptor_dbw_can {

DbwNode::DbwNode(const rclcpp::NodeOptions& options) : Node("raptor_dbw_can_node", options) {
  dbcFile_can0_ = this->declare_parameter("dbw_dbc_file_can0", "");
  dbcFile_can1_ = this->declare_parameter("dbw_dbc_file_can1", "");
  dbcFile_can2_ = this->declare_parameter("dbw_dbc_file_can2", "");
  this->declare_parameter("using_marelli", true);

  // Initializing tire report
  for (int i = 0; i < 16; i++) {
    tire_report_msg.fl_tire_temperature.push_back(0.0);
    tire_report_msg.fr_tire_temperature.push_back(0.0);
    tire_report_msg.rl_tire_temperature.push_back(0.0);
    tire_report_msg.rr_tire_temperature.push_back(0.0);
  }

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can0_tx", 20);
  pub_accel_pedal_ =
      this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>("accelerator_pedal_report", 20);
  pub_steering_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringReport>("steering_report", 20);
  pub_steering_ext_ =
      this->create_publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>("steering_extended_report", 20);
  pub_wheel_speeds_ = this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>("wheel_speed_report", 20);
  pub_brake_2_report_ = this->create_publisher<raptor_dbw_msgs::msg::Brake2Report>("brake_2_report", 20);
  pub_brake_extd_report_ = this->create_publisher<raptor_dbw_msgs::msg::BrakeExtdReport>("brake_extd_report", 20);
  pub_misc_do_ = this->create_publisher<deep_orange_msgs::msg::MiscReport>("misc_report_do", 10);
  pub_rc_to_ct_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("rc_to_ct", 10);
  pub_base_to_car_timing_ = this->create_publisher<deep_orange_msgs::msg::BaseToCarTiming>("base_to_car_timing", 10);
  pub_brake_temp_report_ = this->create_publisher<deep_orange_msgs::msg::BrakeTempReport>("brake_temp_report", 10);
  pub_tire_report_ = this->create_publisher<deep_orange_msgs::msg::TireReport>("tire_report", 10);
  pub_pt_report_ = this->create_publisher<deep_orange_msgs::msg::PtReport>("pt_report", 10);
  pub_diagnostic_report_ = this->create_publisher<raptor_dbw_msgs::msg::DiagnosticReport>("diagnostic_report", 10);
  pub_motec_report_ = this->create_publisher<raptor_dbw_msgs::msg::MotecReport>("motec_report", 10);
  pub_engine_kill_ = this->create_publisher<std_msgs::msg::Int32>("ct_input", 10);

  // Set up Subscribers
  sub_can0_ = this->create_subscription<can_msgs::msg::Frame>(
      "can0_rx", 500, std::bind(&DbwNode::recvCAN0, this, std::placeholders::_1));
  sub_can1_ = this->create_subscription<can_msgs::msg::Frame>(
      "can1_rx", 500, std::bind(&DbwNode::recvCAN1, this, std::placeholders::_1));
  // sub_can2_ = this->create_subscription<can_msgs::msg::Frame>(
  //   "can2_rx", 500, std::bind(&DbwNode::recvCAN2, this, std::placeholders::_1));
  sub_brake_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>(
      "brake_cmd", 1, std::bind(&DbwNode::recvBrakeCmd, this, std::placeholders::_1));
  sub_accelerator_pedal_ = this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>(
      "accelerator_pedal_cmd", 1, std::bind(&DbwNode::recvAcceleratorPedalCmd, this, std::placeholders::_1));
  sub_steering_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringCmd>(
      "steering_cmd", 1, std::bind(&DbwNode::recvSteeringCmd, this, std::placeholders::_1));
  sub_gear_shift_cmd_ = this->create_subscription<std_msgs::msg::UInt8>(
      "gear_cmd", 10, std::bind(&DbwNode::recvGearShiftCmd, this, std::placeholders::_1));
  sub_ct_report_ = this->create_subscription<deep_orange_msgs::msg::CtReport>(
      "ct_report", 1, std::bind(&DbwNode::recvCtReport, this, std::placeholders::_1));
  sub_dash_switches_ = this->create_subscription<raptor_dbw_msgs::msg::DashSwitches>(
      "dash_switches", 1, std::bind(&DbwNode::recvDashSwitches, this, std::placeholders::_1));

  dbwDbc_can0_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can0_);
  dbwDbc_can1_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can1_);
  // dbwDbc_can2_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can2_);

  // Set up Timer
  timer_tire_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerTireCallback, this));
  timer_pt_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerPtCallback, this));
  timer_motec_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerMotecCallback, this));
  timer_brake_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerBrakeCallback, this));
  timer_steering_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerSteeringCallback, this));
}

DbwNode::~DbwNode() {}

void DbwNode::recvCAN0(const can_msgs::msg::Frame::SharedPtr msg) {
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      case ID_WHEEL_SPEED_REPORT_DO: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          raptor_dbw_msgs::msg::WheelSpeedReport out;
          out.header.stamp = msg->header.stamp;

          out.front_left = message->GetSignal("wheel_speed_FL")->GetResult();
          out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
          out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
          out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

          pub_wheel_speeds_->publish(out);
        }
      } break;

      case ID_BRAKE_PRESSURE_REPORT_DO: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          raptor_dbw_msgs::msg::Brake2Report out;
          out.header.stamp = msg->header.stamp;

          out.front_brake_pressure = message->GetSignal("brake_pressure_fdbk_front")->GetResult();
          out.rear_brake_pressure = message->GetSignal("brake_pressure_fdbk_rear")->GetResult();
          out.rolling_counter = message->GetSignal("brk_pressure_fdbk_counter")->GetResult();

          pub_brake_2_report_->publish(out);
        }
      } break;

      case ID_BRAKE_REPORT_EXTD: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BRAKE_REPORT_EXTD);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          brake_report_msg.f_brk_pos_cmd = message->GetSignal("F_brk_pos_cmd")->GetResult();
          brake_report_msg.f_brk_pos_fbk = message->GetSignal("F_brk_pos_fbk")->GetResult();
          brake_report_msg.r_brk_pos_cmd = message->GetSignal("R_brk_pos_cmd")->GetResult();
          brake_report_msg.r_brk_pos_fbk = message->GetSignal("R_brk_pos_fbk")->GetResult();
        }
      } break;

      case ID_BRAKE_REPORT_EXTD_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BRAKE_REPORT_EXTD_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          brake_report_msg.f_brake_act_force = message->GetSignal("f_brake_act_force")->GetResult();
          brake_report_msg.r_brake_act_force = message->GetSignal("r_brake_act_force")->GetResult();
        }
      } break;

      case ID_ACCELERATOR_REPORT_DO: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          raptor_dbw_msgs::msg::AcceleratorPedalReport out;
          out.header.stamp = msg->header.stamp;

          out.pedal_output = message->GetSignal("acc_pedal_fdbk")->GetResult();
          out.rolling_counter = message->GetSignal("acc_pedal_fdbk_counter")->GetResult();
          pub_accel_pedal_->publish(out);
        }
      } break;

      case ID_STEERING_REPORT_DO: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_STEERING_REPORT_DO);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          steering_report_msg.header.stamp = msg->header.stamp;
          // steering_report_msg.static_friction_compensation  =
          // message->GetSignal("static_friction_compensation")->GetResult();
          steering_report_msg.primary_steering_angular_rate =
              message->GetSignal("primary_steering_angular_rate")->GetResult();
          steering_report_msg.steering_motor_fdbk_counter =
              message->GetSignal("steering_motor_fdbk_counter")->GetResult();
          steering_report_msg.commanded_steering_rate = message->GetSignal("commanded_steering_rate")->GetResult();
        }
      } break;

      case ID_STEERING_REPORT_EXTD: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_STEERING_REPORT_EXTD);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          steering_report_msg.header.stamp = msg->header.stamp;
          steering_report_msg.average_steering_ang_fdbk = message->GetSignal("average_steering_ang_fdbk")->GetResult();
          steering_report_msg.secondary_steering_ang_fdbk =
              message->GetSignal("secondary_steering_ang_fdbk")->GetResult();
          steering_report_msg.primary_steering_angle_fbk =
              message->GetSignal("primary_steering_angle_fbk")->GetResult();
        }
      } break;

      case ID_STEERING_REPORT_EXTD_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_STEERING_REPORT_EXTD_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          steering_report_msg.motor_duty_cycle_cmd = message->GetSignal("motor_duty_cycle_cmd")->GetResult();
          steering_report_msg.motor_duty_cycle_fbk = message->GetSignal("motor_duty_cycle_fbk")->GetResult();
          steering_report_msg.motor_current_fbk = message->GetSignal("motor_current_fbk")->GetResult();
          steering_report_msg.sbw_ecu_voltage = message->GetSignal("sbw_ecu_voltage")->GetResult();
          steering_report_msg.sbw_ecu_temp = message->GetSignal("sbw_ecu_temp")->GetResult();
          steering_report_msg.sbw_error_code = message->GetSignal("sbw_error_code")->GetResult();
          steering_report_msg.sbw_motor_torque_estimate = message->GetSignal("sbw_motor_torque_estimate")->GetResult();
        }
      } break;

      case ID_MISC_REPORT_DO: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_MISC_REPORT_DO);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          deep_orange_msgs::msg::MiscReport out;

          // out.off_grid_power_connection  = message->GetSignal("off_grid_power_connection")->GetResult();
          out.stamp = msg->header.stamp;
          out.sys_state = message->GetSignal("sys_state")->GetResult();
          out.safety_switch_state = message->GetSignal("safety_switch_state")->GetResult();
          out.mode_switch_state = message->GetSignal("mode_switch_state")->GetResult();
          out.battery_voltage = message->GetSignal("battery_voltage")->GetResult();
          pub_misc_do_->publish(out);
        }
      } break;

      case ID_RC_TO_CT: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_RC_TO_CT);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          deep_orange_msgs::msg::RcToCt out;
          out.stamp = msg->header.stamp;
          // out.current_position  = message->GetSignal("DBW_CurrentPosition")->GetResult();
          out.track_flag = message->GetSignal("track_flag")->GetResult();
          out.veh_flag = message->GetSignal("veh_flag")->GetResult();
          out.lap_count = message->GetSignal("lap_count")->GetResult();
          out.lap_distance = message->GetSignal("lap_distance")->GetResult();
          out.target_speed = message->GetSignal("round_target_speed")->GetResult();
          out.rolling_counter = message->GetSignal("base_to_car_heartbeat")->GetResult();
          if (!this->get_parameter("using_marelli").as_bool()) {
            pub_rc_to_ct_->publish(out);
          }
        }
      } break;

      case ID_BASE_TO_CAR_TIMING: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BASE_TO_CAR_TIMING);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          deep_orange_msgs::msg::BaseToCarTiming out;
          out.stamp = msg->header.stamp;
          // out.current_position  = message->GetSignal("DBW_CurrentPosition")->GetResult();
          out.laps = message->GetSignal("laps")->GetResult();
          out.lap_time = message->GetSignal("lap_time")->GetResult();
          out.time_stamp = message->GetSignal("time_stamp")->GetResult();

          pub_base_to_car_timing_->publish(out);
        }
      } break;

      case ID_MARELLI_REPORT_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_MARELLI_REPORT_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          deep_orange_msgs::msg::RcToCt out;
          out.stamp = msg->header.stamp;
          int track_flag_ = message->GetSignal("marelli_track_flag")->GetResult();
          if (track_flag_ >= 40) {  // Green Flag + Speed
            out.track_flag = 1;
            this->defender_speed = track_flag_;
          } else {
            out.track_flag = track_flag_;
          }
          out.veh_flag = message->GetSignal("marelli_vehicle_flag")->GetResult();
          out.sector_flag = message->GetSignal("marelli_sector_flag")->GetResult();
          out.target_speed = this->defender_speed;
          out.hb_check = message->GetSignal("marelli_rc_base_sync_check")->GetResult();
          if (this->get_parameter("using_marelli").as_bool()) {
            pub_rc_to_ct_->publish(out);
          }
        }
      } break;

      case ID_PT_REPORT_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          pt_report_msg.stamp = msg->header.stamp;
          // pt_report_msg.sys_state = message->GetSignal("engine_state")->GetResult();
          // pt_report_msg.safety_switch_state = message->GetSignal("engine_run_switch")->GetResult();
          pt_report_msg.throttle_position = message->GetSignal("throttle_position")->GetResult();
          pt_report_msg.engine_run_switch_status = message->GetSignal("engine_run_switch")->GetResult();
          pt_report_msg.current_gear = message->GetSignal("current_gear")->GetResult();
          pt_report_msg.engine_rpm = message->GetSignal("engine_speed_rpm")->GetResult();
          pt_report_msg.vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph")->GetResult();
        }
      } break;

      case ID_PT_REPORT_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          pt_report_msg.fuel_pressure = message->GetSignal("fuel_pressure_kPa")->GetResult();
          pt_report_msg.engine_oil_pressure = message->GetSignal("engine_oil_pressure_kPa")->GetResult();
          pt_report_msg.engine_coolant_temperature = message->GetSignal("coolant_temperature")->GetResult();
          pt_report_msg.transmission_oil_temperature = message->GetSignal("transmission_temperature")->GetResult();
          pt_report_msg.transmission_oil_pressure = message->GetSignal("transmission_pressure_kPa")->GetResult();
        }
      } break;

      case ID_TIRE_PRESSURE_FL: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_FL);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_tire_pressure = message->GetSignal("FL_Tire_Pressure")->GetResult();
          tire_report_msg.fl_tire_pressure_gauge = message->GetSignal("FL_Tire_Pressure_Gauge")->GetResult();
        }
      } break;

      case ID_TIRE_PRESSURE_FR: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_FR);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fr_tire_pressure = message->GetSignal("FR_Tire_Pressure")->GetResult();
          tire_report_msg.fr_tire_pressure_gauge = message->GetSignal("FR_Tire_Pressure_Gauge")->GetResult();
        }
      } break;

      case ID_TIRE_PRESSURE_RL: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_RL);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rl_tire_pressure = message->GetSignal("RL_Tire_Pressure")->GetResult();
          tire_report_msg.rl_tire_pressure_gauge = message->GetSignal("RL_Tire_Pressure_Gauge")->GetResult();
        }
      } break;

      case ID_TIRE_PRESSURE_RR: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_RR);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rr_tire_pressure = message->GetSignal("RR_Tire_Pressure")->GetResult();
          tire_report_msg.rr_tire_pressure_gauge = message->GetSignal("RR_Tire_Pressure_Gauge")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FL_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FL_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_tire_temperature[0] = message->GetSignal("FL_Tire_Temp_01")->GetResult();
          tire_report_msg.fl_tire_temperature[1] = message->GetSignal("FL_Tire_Temp_02")->GetResult();
          tire_report_msg.fl_tire_temperature[2] = message->GetSignal("FL_Tire_Temp_03")->GetResult();
          tire_report_msg.fl_tire_temperature[3] = message->GetSignal("FL_Tire_Temp_04")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FL_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FL_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_tire_temperature[4] = message->GetSignal("FL_Tire_Temp_05")->GetResult();
          tire_report_msg.fl_tire_temperature[5] = message->GetSignal("FL_Tire_Temp_06")->GetResult();
          tire_report_msg.fl_tire_temperature[6] = message->GetSignal("FL_Tire_Temp_07")->GetResult();
          tire_report_msg.fl_tire_temperature[7] = message->GetSignal("FL_Tire_Temp_08")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FL_3: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FL_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_tire_temperature[8] = message->GetSignal("FL_Tire_Temp_09")->GetResult();
          tire_report_msg.fl_tire_temperature[9] = message->GetSignal("FL_Tire_Temp_10")->GetResult();
          tire_report_msg.fl_tire_temperature[10] = message->GetSignal("FL_Tire_Temp_11")->GetResult();
          tire_report_msg.fl_tire_temperature[11] = message->GetSignal("FL_Tire_Temp_12")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FL_4: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FL_4);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_tire_temperature[12] = message->GetSignal("FL_Tire_Temp_13")->GetResult();
          tire_report_msg.fl_tire_temperature[13] = message->GetSignal("FL_Tire_Temp_14")->GetResult();
          tire_report_msg.fl_tire_temperature[14] = message->GetSignal("FL_Tire_Temp_15")->GetResult();
          tire_report_msg.fl_tire_temperature[15] = message->GetSignal("FL_Tire_Temp_16")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FR_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FR_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fr_tire_temperature[0] = message->GetSignal("FR_Tire_Temp_01")->GetResult();
          tire_report_msg.fr_tire_temperature[1] = message->GetSignal("FR_Tire_Temp_02")->GetResult();
          tire_report_msg.fr_tire_temperature[2] = message->GetSignal("FR_Tire_Temp_03")->GetResult();
          tire_report_msg.fr_tire_temperature[3] = message->GetSignal("FR_Tire_Temp_04")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FR_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FR_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fr_tire_temperature[4] = message->GetSignal("FR_Tire_Temp_05")->GetResult();
          tire_report_msg.fr_tire_temperature[5] = message->GetSignal("FR_Tire_Temp_06")->GetResult();
          tire_report_msg.fr_tire_temperature[6] = message->GetSignal("FR_Tire_Temp_07")->GetResult();
          tire_report_msg.fr_tire_temperature[7] = message->GetSignal("FR_Tire_Temp_08")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FR_3: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FR_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fr_tire_temperature[8] = message->GetSignal("FR_Tire_Temp_09")->GetResult();
          tire_report_msg.fr_tire_temperature[9] = message->GetSignal("FR_Tire_Temp_10")->GetResult();
          tire_report_msg.fr_tire_temperature[10] = message->GetSignal("FR_Tire_Temp_11")->GetResult();
          tire_report_msg.fr_tire_temperature[11] = message->GetSignal("FR_Tire_Temp_12")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_FR_4: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FR_4);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fr_tire_temperature[12] = message->GetSignal("FR_Tire_Temp_13")->GetResult();
          tire_report_msg.fr_tire_temperature[13] = message->GetSignal("FR_Tire_Temp_14")->GetResult();
          tire_report_msg.fr_tire_temperature[14] = message->GetSignal("FR_Tire_Temp_15")->GetResult();
          tire_report_msg.fr_tire_temperature[15] = message->GetSignal("FR_Tire_Temp_16")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RL_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RL_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rl_tire_temperature[0] = message->GetSignal("RL_Tire_Temp_01")->GetResult();
          tire_report_msg.rl_tire_temperature[1] = message->GetSignal("RL_Tire_Temp_02")->GetResult();
          tire_report_msg.rl_tire_temperature[2] = message->GetSignal("RL_Tire_Temp_03")->GetResult();
          tire_report_msg.rl_tire_temperature[3] = message->GetSignal("RL_Tire_Temp_04")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RL_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RL_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rl_tire_temperature[4] = message->GetSignal("RL_Tire_Temp_05")->GetResult();
          tire_report_msg.rl_tire_temperature[5] = message->GetSignal("RL_Tire_Temp_06")->GetResult();
          tire_report_msg.rl_tire_temperature[6] = message->GetSignal("RL_Tire_Temp_07")->GetResult();
          tire_report_msg.rl_tire_temperature[7] = message->GetSignal("RL_Tire_Temp_08")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RL_3: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RL_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rl_tire_temperature[8] = message->GetSignal("RL_Tire_Temp_09")->GetResult();
          tire_report_msg.rl_tire_temperature[9] = message->GetSignal("RL_Tire_Temp_10")->GetResult();
          tire_report_msg.rl_tire_temperature[10] = message->GetSignal("RL_Tire_Temp_11")->GetResult();
          tire_report_msg.rl_tire_temperature[11] = message->GetSignal("RL_Tire_Temp_12")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RL_4: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RL_4);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rl_tire_temperature[12] = message->GetSignal("RL_Tire_Temp_13")->GetResult();
          tire_report_msg.rl_tire_temperature[13] = message->GetSignal("RL_Tire_Temp_14")->GetResult();
          tire_report_msg.rl_tire_temperature[14] = message->GetSignal("RL_Tire_Temp_15")->GetResult();
          tire_report_msg.rl_tire_temperature[15] = message->GetSignal("RL_Tire_Temp_16")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RR_1: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RR_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rr_tire_temperature[0] = message->GetSignal("RR_Tire_Temp_01")->GetResult();
          tire_report_msg.rr_tire_temperature[1] = message->GetSignal("RR_Tire_Temp_02")->GetResult();
          tire_report_msg.rr_tire_temperature[2] = message->GetSignal("RR_Tire_Temp_03")->GetResult();
          tire_report_msg.rr_tire_temperature[3] = message->GetSignal("RR_Tire_Temp_04")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RR_2: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RR_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rr_tire_temperature[4] = message->GetSignal("RR_Tire_Temp_05")->GetResult();
          tire_report_msg.rr_tire_temperature[5] = message->GetSignal("RR_Tire_Temp_06")->GetResult();
          tire_report_msg.rr_tire_temperature[6] = message->GetSignal("RR_Tire_Temp_07")->GetResult();
          tire_report_msg.rr_tire_temperature[7] = message->GetSignal("RR_Tire_Temp_08")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RR_3: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RR_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rr_tire_temperature[8] = message->GetSignal("RR_Tire_Temp_09")->GetResult();
          tire_report_msg.rr_tire_temperature[9] = message->GetSignal("RR_Tire_Temp_10")->GetResult();
          tire_report_msg.rr_tire_temperature[10] = message->GetSignal("RR_Tire_Temp_11")->GetResult();
          tire_report_msg.rr_tire_temperature[11] = message->GetSignal("RR_Tire_Temp_12")->GetResult();
        }
      } break;

      case ID_TIRE_TEMP_RR_4: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_RR_4);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.rr_tire_temperature[12] = message->GetSignal("RR_Tire_Temp_13")->GetResult();
          tire_report_msg.rr_tire_temperature[13] = message->GetSignal("RR_Tire_Temp_14")->GetResult();
          tire_report_msg.rr_tire_temperature[14] = message->GetSignal("RR_Tire_Temp_15")->GetResult();
          tire_report_msg.rr_tire_temperature[15] = message->GetSignal("RR_Tire_Temp_16")->GetResult();
        }
      } break;

      case ID_WHEEL_STRAIN_GAUGE: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_STRAIN_GAUGE);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.fl_wheel_load = message->GetSignal("wheel_strain_gauge_FL")->GetResult();
          tire_report_msg.fr_wheel_load = message->GetSignal("wheel_strain_gauge_FR")->GetResult();
          tire_report_msg.rl_wheel_load = message->GetSignal("wheel_strain_gauge_RL")->GetResult();
          tire_report_msg.rr_wheel_load = message->GetSignal("wheel_strain_gauge_RR")->GetResult();
        }
      } break;

      case ID_PT_REPORT_3: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          pt_report_msg.engine_oil_temperature = message->GetSignal("engine_oil_temperature")->GetResult();
          pt_report_msg.torque_wheels = message->GetSignal("torque_wheels")->GetResult();
          pt_report_msg.driver_traction_aim_switch_fbk =
              message->GetSignal("driver_traction_aim_swicth_fbk")->GetResult();
          pt_report_msg.driver_traction_range_switch_fbk =
              message->GetSignal("driver_traction_range_switch_fbk")->GetResult();
          pt_report_msg.push2pass_status = message->GetSignal("push2pass_status")->GetResult();
          pt_report_msg.push2pass_budget = message->GetSignal("push2pass_budget_s")->GetResult();
          pt_report_msg.push2pass_active_app_limit = message->GetSignal("push2pass_active_app_limit")->GetResult();
        }
      } break;

      case ID_WHEEL_POTENTIOMETER: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_POTENTIOMETER);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);
          tire_report_msg.stamp = msg->header.stamp;
          tire_report_msg.fl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FL")->GetResult();
          tire_report_msg.fr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FR")->GetResult();
          tire_report_msg.rl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RL")->GetResult();
          tire_report_msg.rr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RR")->GetResult();
        }
      } break;

      case ID_DIAGNOSTIC_REPORT: {
        NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_DIAGNOSTIC_REPORT);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          raptor_dbw_msgs::msg::DiagnosticReport out;
          out.header = msg->header;
          out.sd_system_warning = message->GetSignal("sd_system_warning")->GetResult();
          out.sd_system_failure = message->GetSignal("sd_system_failure")->GetResult();
          out.sd_brake_warning1 = message->GetSignal("sd_brake_warning1")->GetResult();
          out.sd_brake_warning2 = message->GetSignal("sd_brake_warning2")->GetResult();
          out.sd_brake_warning3 = message->GetSignal("sd_brake_warning3")->GetResult();
          out.sd_steer_warning1 = message->GetSignal("sd_steer_warning1")->GetResult();
          out.sd_steer_warning2 = message->GetSignal("sd_steer_warning2")->GetResult();
          out.sd_steer_warning3 = message->GetSignal("sd_steer_warning3")->GetResult();
          out.motec_warning = message->GetSignal("motec_warning")->GetResult();
          out.front_brk_failure = message->GetSignal("est1_oos_front_brk")->GetResult();
          out.rear_brk_failure = message->GetSignal("est2_oos_rear_brk")->GetResult();
          out.low_eng_speed = message->GetSignal("est3_low_eng_speed")->GetResult();
          out.sd_comms_loss = message->GetSignal("est4_sd_comms_loss")->GetResult();
          out.motec_comms_loss = message->GetSignal("est5_motec_comms_loss")->GetResult();
          out.sd_ebrake = message->GetSignal("est6_sd_ebrake")->GetResult();
          out.adlink_hb_lost = message->GetSignal("adlink_hb_lost")->GetResult();
          out.rc_lost = message->GetSignal("rc_lost")->GetResult();

          pub_diagnostic_report_->publish(out);
        }
      } break;
    }
  }
}

void DbwNode::recvCAN1(const can_msgs::msg::Frame::SharedPtr msg) {
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {

      case ID_M1_GENERAL_1: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_1);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.engine_speed = message->GetSignal("Engine_Speed_MoTeC")->GetResult();
          motec_report_msg.inlet_manifold_pressure = message->GetSignal("Inlet_Manifold_Pressure_MoTeC")->GetResult();
          motec_report_msg.inlet_air_temperature = message->GetSignal("Inlet_Air_Temperature_MoTeC")->GetResult();
          motec_report_msg.throttle_position = message->GetSignal("Throttle_Position_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_2: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_2);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.fuel_pressure_sensor = message->GetSignal("Fuel_Pressure_Sensor")->GetResult();
          motec_report_msg.engine_efficiency = message->GetSignal("Engine_Efficiency")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_3: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_3);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.throttle_pedal = message->GetSignal("Throttle_Pedal_MoTeC")->GetResult();
          motec_report_msg.engine_load = message->GetSignal("Engine_Load_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_4: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_4);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.engine_oil_pressure = message->GetSignal("Engine_Oil_Pressure_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_5: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_5);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.boost_pressure = message->GetSignal("Boost_Pressure_MoTeC")->GetResult();
          motec_report_msg.boost_aim = message->GetSignal("Boost_Aim_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_6: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_6);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.wheel_speed_front_left = message->GetSignal("Wheel_Speed_Front_Left_MoTeC")->GetResult();
          motec_report_msg.wheel_speed_front_right = message->GetSignal("Wheel_Speed_Front_Right_MoTeC")->GetResult();
          motec_report_msg.wheel_speed_rear_left = message->GetSignal("Wheel_Speed_Rear_Left_MoTeC")->GetResult();
          motec_report_msg.wheel_speed_rear_right = message->GetSignal("Wheel_Speed_Rear_Right_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_7: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_7);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.coolant_temperature = message->GetSignal("Coolant_Temperature_MoTeC")->GetResult();
          motec_report_msg.engine_oil_temperature = message->GetSignal("Engine_Oil_Temperature_MoTeC")->GetResult();
          motec_report_msg.ambient_temperature = message->GetSignal("Ambient_Temperature_MoTeC")->GetResult();
          motec_report_msg.ecu_battery_voltage = message->GetSignal("ECU_Battery_Voltage_MoTeC")->GetResult();
          motec_report_msg.fuel_used = message->GetSignal("Fuel_Used_MoTeC")->GetResult();
        }
      } break;

        // case ID_M1_GENERAL_8:
        //   {
        //    NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_8);
        //     if (msg->dlc >= message->GetDlc()) {

        //       message->SetFrame(msg);

        //       motec_report_msg.engine_run_time = message->GetSignal("Engine_Run_Time_MoTeC")->GetResult();
        //       motec_report_msg.ecu_up_time = message->GetSignal("ECU_Up_Time_MoTeC")->GetResult();
        //       motec_report_msg.warning_source = message->GetSignal("Warning_Source_MoTeC")->GetResult();
        //       motec_report_msg.coolant_temperature_warning =
        //       message->GetSignal("Coolant_Temperature_Warning")->GetResult();
        //       motec_report_msg.coolant_pressure_warning =
        //       message->GetSignal("Coolant_Pressure_Warning")->GetResult(); motec_report_msg.coolant_temperature =
        //       message->GetSignal("Coolant_Temperature_MoTeC")->GetResult(); motec_report_msg.engine_speed_warning =
        //       message->GetSignal("Engine_Speed_Warning")->GetResult();
        //       motec_report_msg.engine_oil_temperature_warning =
        //       message->GetSignal("Engine_Oil_Temperature_Warning")->GetResult();
        //       motec_report_msg.engine_oil_pressure_warning =
        //       message->GetSignal("Engine_Oil_Pressure_Warning")->GetResult();
        //       motec_report_msg.crankcase_pressure_warning =
        //       message->GetSignal("Crankcase_Pressure_Warning")->GetResult(); motec_report_msg.fuel_pressure_warning =
        //       message->GetSignal("Fuel_Pressure_Warning")->GetResult(); motec_report_msg.knock_warning =
        //       message->GetSignal("Knock_Warning")->GetResult();
        //     }
        //   }
        //   break;

      case ID_M1_GENERAL_9: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_9);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.engine_state = message->GetSignal("Engine_State")->GetResult();
          motec_report_msg.fuel_pump_state = message->GetSignal("Fuel_Pump_State")->GetResult();
          motec_report_msg.launch_state = message->GetSignal("Launch_State")->GetResult();
          motec_report_msg.boost_aim_state = message->GetSignal("Boost_Aim_State")->GetResult();
          motec_report_msg.knock_state = message->GetSignal("Knock_State")->GetResult();
          motec_report_msg.throttle_aim_state = message->GetSignal("Throttle_Aim_State")->GetResult();
          motec_report_msg.engine_speed_reference_state =
              message->GetSignal("Engine_Speed_Reference_State")->GetResult();
          motec_report_msg.gear = message->GetSignal("Gear")->GetResult();
          motec_report_msg.engine_speed_limit_state = message->GetSignal("Engine_Speed_Limit_State")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_10: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_10);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.engine_run_time_total = message->GetSignal("Engine_Run_Time_Total")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_11: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_11);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.driver_switch_1 = message->GetSignal("Driver_Switch_1")->GetResult();
          motec_report_msg.driver_switch_2 = message->GetSignal("Driver_Switch_2")->GetResult();
          motec_report_msg.driver_switch_3 = message->GetSignal("Driver_Switch_3")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_12: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_12);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.exhaust_lambda = message->GetSignal("Exhaust_Lambda")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_13: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_13);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.fuel_pressure_direct_b1 = message->GetSignal("Fuel_Pressure_Direct_B1")->GetResult();
          motec_report_msg.fuel_pressure_direct_b1_aim = message->GetSignal("Fuel_Pressure_Direct_B1_Aim")->GetResult();
          motec_report_msg.fuel_pressure_direct_b1_control =
              message->GetSignal("Fuel_Pressure_Direct_B1_Control")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_14: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_14);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.vehicle_speed = message->GetSignal("Vehicle_Speed_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_15: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_15);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.gearratio = message->GetSignal("GearRatio")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_16: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_16);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.gear_100hz = message->GetSignal("Gear_100Hz_MoTeC")->GetResult();
          motec_report_msg.gear_shift = message->GetSignal("Gear_Shift")->GetResult();
          motec_report_msg.gear_shift_req = message->GetSignal("Gear_Shift_Req")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_17: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_17);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.torque_wheels = message->GetSignal("Torque_Wheels_MoTeC")->GetResult();
        }
      } break;

      case ID_M1_GENERAL_18: {
        NewEagle::DbcMessage* message = dbwDbc_can1_.GetMessageById(ID_M1_GENERAL_18);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          motec_report_msg.fr_wheel_speed_sensor_fault = message->GetSignal("FR_Wheel_Speed_Sensor_Fault")->GetResult();
          motec_report_msg.fl_wheel_speed_sensor_fault = message->GetSignal("FL_Wheel_Speed_Sensor_Fault")->GetResult();
          motec_report_msg.rr_wheel_speed_sensor_fault = message->GetSignal("RR_Wheel_Speed_Sensor_Fault")->GetResult();
          motec_report_msg.rl_wheel_speed_sensor_fault = message->GetSignal("RL_Wheel_Speed_Sensor_Fault")->GetResult();
        }
      } break;
    }
  }
}

void DbwNode::recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg) {
  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("brake_pressure_cmd");
  message->GetSignal("F_brake_pressure_cmd")->SetResult(msg->pedal_cmd);
  message->GetSignal("brk_pressure_cmd_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg) {
  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("accelerator_cmd");

  message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
  message->GetSignal("acc_pedal_cmd_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void DbwNode::recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg) {
  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("steering_cmd");

  message->GetSignal("steering_motor_ang_cmd")->SetResult(msg->angle_cmd);
  message->GetSignal("steering_motor_cmd_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::recvCtReport(const deep_orange_msgs::msg::CtReport::SharedPtr msg) {

  if (this->get_parameter("using_marelli").as_bool()) {
    NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("ct_report_2");
    message->GetSignal("marelli_track_flag_ack")->SetResult(msg->track_flag_ack);
    message->GetSignal("marelli_vehicle_flag_ack")->SetResult(msg->veh_flag_ack);
    message->GetSignal("marelli_sector_flag_ack")->SetResult(msg->sector_flag_ack);

    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
  }

  // This message keeps raptor hb, so need to send it even on marelli
  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("ct_report");
  message->GetSignal("track_cond_ack")->SetResult(msg->track_flag_ack);
  message->GetSignal("veh_sig_ack")->SetResult(msg->veh_flag_ack);
  message->GetSignal("ct_state")->SetResult(msg->ct_state);
  message->GetSignal("ct_state_rolling_counter")->SetResult(msg->rolling_counter);

  can_msgs::msg::Frame frame = message->GetFrame();
  pub_can_->publish(frame);

  this->ct_state = msg->ct_state;
}

void DbwNode::recvDashSwitches(const raptor_dbw_msgs::msg::DashSwitches::SharedPtr msg) {

  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("dash_switches_cmd");
  message->GetSignal("driver_traction_aim_switch")->SetResult(msg->driver_traction_aim_switch);
  message->GetSignal("driver_traction_range_switch")->SetResult(msg->driver_traction_range_switch);
  message->GetSignal("brake_bias_aim_switch")->SetResult(msg->brake_bias_aim_switch);
  // message->GetSignal("driver_steering_gain_cntrl_switch")->SetResult(msg->driver_steering_gain_cntrl_switch);
  message->GetSignal("push2pass_switch")->SetResult(msg->push2pass_switch);

  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg) {

  NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessage("gear_shift_cmd");
  message->GetSignal("desired_gear")->SetResult(msg->data);
  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void DbwNode::timerTireCallback() {
  pub_tire_report_->publish(tire_report_msg);
}

void DbwNode::timerPtCallback() {
  pub_pt_report_->publish(pt_report_msg);
}

void DbwNode::timerMotecCallback() {
  pub_motec_report_->publish(motec_report_msg);
}

void DbwNode::timerBrakeCallback() {
  pub_brake_extd_report_->publish(brake_report_msg);
}

void DbwNode::timerSteeringCallback() {
  pub_steering_ext_->publish(steering_report_msg);
}
}  // namespace raptor_dbw_can

RCLCPP_COMPONENTS_REGISTER_NODE(raptor_dbw_can::DbwNode)
