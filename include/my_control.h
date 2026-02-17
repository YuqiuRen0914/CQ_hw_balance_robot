#pragma once
#include "my_config.h"

// 运行时力矩总幅限制（可通过网络配置动态调整）
extern float torque_limit;


void control_reset(robot_state &robot);
void control_update_pid(robot_state &robot);
void control_pitch(robot_state &robot);
void control_yaw(robot_state &robot);
void control_torque_mix(robot_state &robot);

// Fallen → swing-up → soft takeover
void control_swing_up(robot_state &robot);
bool control_soft_takeover_active();
float control_soft_takeover_gain(); // 0~1，接管进度
