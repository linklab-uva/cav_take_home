std_msgs/Header header

uint8 steering_motor_fdbk_counter
int8 static_friction_compensation # %
int16 commanded_steering_rate # deg/s

float32 average_steering_ang_fdbk
float32 secondary_steering_ang_fdbk
float32 primary_steering_angle_fbk

int8 motor_duty_cycle_cmd # %
int8 motor_duty_cycle_fbk # %
uint8 motor_current_fbk # A
uint8 sbw_ecu_voltage # V
uint8 sbw_ecu_temp # deg (C)
uint8 sbw_error_code
uint8 sbw_motor_torque_estimate # Nm