#pragma once
#include "my_config.h"

void sense_update_wheel_speeds(robot_state &robot);
void sense_update_attitude(robot_state &robot);
void sense_wel_up_detect(robot_state &robot);
void sense_fall_check(robot_state &robot);
bool sense_fallen_recover_ready(robot_state &robot);
bool sense_no_op(const robot_state &robot);

// 陀螺零偏微校准与应用
void sense_update_gyro_bias(robot_state &robot);

// 传感器连通性检测，失联时返回 true 并可置 fault
bool sense_check_i2c_fault(robot_state &robot);

// 姿态零点自适应：静止时缓慢调整 pitch_zero
void sense_adapt_pitch_zero(robot_state &robot);
