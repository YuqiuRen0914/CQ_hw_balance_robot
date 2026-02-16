#include <algorithm>
#include <cmath>
#include <SimpleFOC.h>
#include "my_control.h"
#include "my_tool.h"

// PID 控制器
static PIDController PID_ANG{0, 0, 0, 0, 0};
static PIDController PID_SPD{0, 0, 0, 0, 0};
static PIDController PID_YAW{0, 0, 0, 0, 0};
static uint32_t soft_takeover_start = 0;
static bool soft_takeover = false;

void control_update_pid(robot_state &robot)
{
    PID_ANG.P = robot.ang_pid.p;
    PID_ANG.I = robot.ang_pid.i;
    PID_ANG.D = 0; // 使用陀螺仪作为 D
    PID_ANG.output_ramp = robot.ang_pid.k;
    PID_ANG.limit = robot.ang_pid.l;

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
    PID_ANG.reset();
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
    if (robot.joy_stop_control)
        robot.spd.tar = 0.0f;
    else
        robot.spd.tar = robot.joy.y * robot.joy.y_coef;

    robot.spd.err = robot.spd.tar - robot.spd.now;
    float pitch_delta = PID_SPD(robot.spd.err);
    pitch_delta = my_lim(pitch_delta, PITCH_TAR_MAX_DEG);

    const float pitch_target = robot.pitch_zero + pitch_delta;
    robot.ang.tar = pitch_target;
    robot.ang.err = robot.ang.tar - robot.ang.now;
    robot.tor.base = PID_ANG(robot.ang.err);
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
    if (max_abs > TOR_SUM_LIM)
    {
        const float scale = TOR_SUM_LIM / max_abs;
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
