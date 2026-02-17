#include <algorithm>
#include <cmath>
#include <SimpleFOC.h>
#include "my_control.h"
#include "my_tool.h"

// PID 控制器（速度环/转向环仍用 SimpleFOC PID）
static PIDController PID_SPD{0, 0, 0, 0, 0};
static PIDController PID_YAW{0, 0, 0, 0, 0};

// 角度环：手写 P+I，支持 back-calculation 抗饱和
static float ang_integral   = 0.0f;
static float ang_err_prev   = 0.0f;
static uint32_t ang_ts_prev = 0;

// 陀螺阻尼低通滤波状态
static float filtered_gyroy  = 0.0f;

// 速度指令前馈状态
static float spd_tar_prev = 0.0f;

// 运行时可调的力矩总幅限制
float torque_limit = TOR_SUM_LIM;
static uint32_t soft_takeover_start = 0;
static bool soft_takeover = false;

void control_update_pid(robot_state &robot)
{
    // 角度环参数由 control_pitch 内手写 P+I 直接读取 robot.ang_pid，无需同步

    PID_SPD.P = robot.spd_pid.p;
    PID_SPD.I = robot.spd_pid.i;
    PID_SPD.D = robot.spd_pid.d;
    PID_SPD.output_ramp = robot.spd_pid.k;
    PID_SPD.limit = robot.spd_pid.l;

    PID_YAW.P = robot.yaw_pid.p;
    PID_YAW.I = robot.yaw_pid.i;
    PID_YAW.D = robot.yaw_pid.d;
    PID_YAW.output_ramp = robot.yaw_pid.k;
    PID_YAW.limit = robot.yaw_pid.l;
}

void control_reset(robot_state &robot)
{
    // 角度环手写状态复位
    ang_integral  = 0.0f;
    ang_err_prev  = 0.0f;
    ang_ts_prev   = 0;
    filtered_gyroy = 0.0f;
    spd_tar_prev  = 0.0f;

    PID_SPD.reset();
    PID_YAW.reset();

    robot.spd.tar = 0.0f;
    robot.tor.base = 0.0f;
    robot.tor.yaw = 0.0f;
    robot.tor.L = 0.0f;
    robot.tor.R = 0.0f;
}

void control_pitch(robot_state &robot)
{
    // ---- 速度环 ----
    if (robot.joy_stop_control)
        robot.spd.tar = 0.0f;
    else
        robot.spd.tar = robot.joy.y * robot.joy.y_coef;

    robot.spd.err = robot.spd.tar - robot.spd.now;
    float pitch_delta = PID_SPD(robot.spd.err);
    pitch_delta = my_lim(pitch_delta, PITCH_TAR_MAX_DEG);

    // 速度指令前馈：摇杆变化率直接前馈到倾角，提升操控响应
    const uint32_t now_us = micros();
    float dt = (ang_ts_prev == 0) ? 0.002f : (now_us - ang_ts_prev) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;
    const float spd_tar_rate = (robot.spd.tar - spd_tar_prev) / dt;
    pitch_delta += ACCEL_FF_GAIN * spd_tar_rate;
    pitch_delta = my_lim(pitch_delta, PITCH_TAR_MAX_DEG);
    spd_tar_prev = robot.spd.tar;

    // ---- 角度环（手写 P+I + 抗饱和） ----
    const float pitch_target = robot.pitch_zero + pitch_delta;
    robot.ang.tar = pitch_target;
    robot.ang.err = robot.ang.tar - robot.ang.now;

    // P 项
    const float p_term = robot.ang_pid.p * robot.ang.err;

    // I 项（梯形积分）
    ang_integral += robot.ang_pid.i * dt * 0.5f * (robot.ang.err + ang_err_prev);
    ang_err_prev = robot.ang.err;
    ang_ts_prev  = now_us;

    float pid_out = p_term + ang_integral;

    // 陀螺阻尼（一阶低通滤波后使用，抑制高频噪声传递到力矩）
    filtered_gyroy = GYRO_DAMP_ALPHA * robot.imu.gyroy
                   + (1.0f - GYRO_DAMP_ALPHA) * filtered_gyroy;
    const float gyro_damping = filtered_gyroy * robot.ang_pid.d;

    // 重力前馈
    const float lean_rad = (robot.ang.now - robot.pitch_zero) * (PI / 180.0f);
    const float gravity_ff = -GRAVITY_FF_GAIN * sinf(lean_rad);

    float tor = pid_out - gyro_damping + gravity_ff;

    // Back-calculation 抗饱和：总力矩超限时回退积分器
    if (tor > torque_limit)
    {
        ang_integral -= (tor - torque_limit);
        tor = torque_limit;
    }
    else if (tor < -torque_limit)
    {
        ang_integral -= (tor + torque_limit);
        tor = -torque_limit;
    }

    robot.tor.base = tor;
}

void control_yaw(robot_state &robot)
{
    robot.yaw.tar = robot.joy.x * robot.joy.x_coef;
    float yaw_err = robot.yaw.tar - robot.yaw.now;
    // wrap [-180,180]
    while (yaw_err > 180.0f)
        yaw_err -= 360.0f;
    while (yaw_err < -180.0f)
        yaw_err += 360.0f;
    robot.yaw.err = yaw_err;
    robot.tor.yaw = PID_YAW(robot.yaw.err);
}

void control_torque_mix(robot_state &robot)
{
    robot.tor.L = robot.tor.base - robot.tor.yaw;
    robot.tor.R = robot.tor.base + robot.tor.yaw;

    auto apply_deadzone = [](float t, float dz) {
        if (t == 0.0f)
            return 0.0f;
        const float sign = (t > 0.0f) ? 1.0f : -1.0f;
        return t + sign * dz;
    };
    robot.tor.L = apply_deadzone(robot.tor.L, robot.tor.dzL);
    robot.tor.R = apply_deadzone(robot.tor.R, robot.tor.dzR);

    const float max_abs = std::max(fabsf(robot.tor.L), fabsf(robot.tor.R));
    if (max_abs > torque_limit)
    {
        const float scale = torque_limit / max_abs;
        robot.tor.L *= scale;
        robot.tor.R *= scale;
    }
}

// 简单摆动起立：sin 波力矩
void control_swing_up(robot_state &robot)
{
    static uint32_t swing_start = 0;
    if (swing_start == 0)
        swing_start = millis();
    const float t = (millis() - swing_start) / 1000.0f;
    const float omega = 2.0f * PI * SWING_FREQ_HZ;
    float amp = SWING_MAX_TORQUE;
    // 随时间略微收敛
    amp = std::max(SWING_MIN_TORQUE, SWING_MAX_TORQUE * (0.4f + 0.6f * expf(-0.2f * t)));
    const float u = amp * sinf(omega * t);
    robot.tor.L = -u;
    robot.tor.R = u;
    robot.tor.base = 0.0f;
    robot.tor.yaw = 0.0f;
    // 摆动成功：pitch 回到阈值内
    if (fabsf(robot.ang.now) < SWING_EXIT_PITCH_DEG)
    {
        soft_takeover = true;
        soft_takeover_start = millis();
        swing_start = 0;
    }
}

bool control_soft_takeover_active()
{
    if (!soft_takeover)
        return false;
    if (millis() - soft_takeover_start > SOFT_TAKEOVER_MS)
    {
        soft_takeover = false;
        return false;
    }
    return true;
}

float control_soft_takeover_gain()
{
    if (!soft_takeover)
        return 1.0f;
    float k = (millis() - soft_takeover_start) / float(SOFT_TAKEOVER_MS);
    if (k >= 1.0f)
    {
        soft_takeover = false;
        return 1.0f;
    }
    return k; // 0 -> 1
}
// 说明：核心平衡控制——PID 更新、力矩混合、摆动起立与软接管
