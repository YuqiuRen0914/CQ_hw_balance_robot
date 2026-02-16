#include <Arduino.h>
#include <cmath>
#include "my_calibration.h"
#include "my_storage.h"

// 标定状态
enum class CalibStage
{
    Reset,
    Left,
    Right,
    Done
};

static CalibStage calib_stage = CalibStage::Reset;
static uint32_t calib_stage_start = 0;
static uint32_t calib_last_step = 0;
static float calib_test_torque = 0.0f;
static bool calib_done_flag = false;
static bool force_recalib_flag = false;

// 标定参数
constexpr float CALIB_PITCH_DEG = 8.0f;
constexpr float CALIB_GYRO_DPS = 20.0f;
constexpr float CALIB_VEL_THRESH = 6.0f;
constexpr float CALIB_TORQUE_START = 0.25f;
constexpr float CALIB_TORQUE_STEP = 0.15f;
constexpr float CALIB_TORQUE_MAX = 3.0f;
constexpr uint32_t CALIB_STEP_HOLD_MS = 40;
constexpr uint32_t CALIB_STAGE_TIMEOUT_MS = 1200;
constexpr float CALIB_FALLBACK_DZ = 0.25f;

static inline bool robot_is_quiet(const robot_state &robot)
{
    return (fabsf(robot.ang.now) < CALIB_PITCH_DEG) &&
           (fabsf(robot.imu.gyroy) < CALIB_GYRO_DPS);
}

static void apply_wheel_torque(robot_state &robot, float desired_L, float desired_R)
{
    robot.tor.L = desired_L;
    robot.tor.R = desired_R;
    robot.tor.base = 0.5f * (desired_L + desired_R);
    robot.tor.yaw = 0.5f * (desired_R - desired_L);
}

void calibration_reset(bool force_recalib)
{
    calib_stage = CalibStage::Reset;
    calib_done_flag = false;
    calib_stage_start = 0;
    calib_last_step = 0;
    calib_test_torque = 0.0f;
    force_recalib_flag = force_recalib;
}

void calibration_load_saved(robot_state &robot, bool has_saved, float dzL, float dzR, bool force_recalib)
{
    force_recalib_flag = force_recalib;
    if (has_saved && !force_recalib_flag)
    {
        robot.tor.dzL = dzL;
        robot.tor.dzR = dzR;
        calib_done_flag = true;
    }
    else
    {
        calib_done_flag = false;
    }
}

bool calibration_done()
{
    return calib_done_flag;
}

bool calibration_step(robot_state &robot)
{
    const uint32_t now = millis();

    if (calib_done_flag && !force_recalib_flag)
        return true;

    if (calib_stage == CalibStage::Reset)
    {
        calib_stage = CalibStage::Left;
        calib_stage_start = calib_last_step = now;
        calib_test_torque = CALIB_TORQUE_START;
        robot.tor.dzL = CALIB_FALLBACK_DZ;
        robot.tor.dzR = CALIB_FALLBACK_DZ;
        calib_done_flag = false;
    }

    if (!robot_is_quiet(robot))
    {
        apply_wheel_torque(robot, 0.0f, 0.0f);
        calib_stage_start = calib_last_step = now;
        calib_test_torque = CALIB_TORQUE_START;
        return false;
    }

    auto advance_stage = [&](CalibStage next) {
        calib_stage = next;
        calib_stage_start = calib_last_step = now;
        calib_test_torque = CALIB_TORQUE_START;
        apply_wheel_torque(robot, 0.0f, 0.0f);
    };

    auto tick_torque = [&]() {
        if (now - calib_last_step >= CALIB_STEP_HOLD_MS)
        {
            calib_last_step = now;
            calib_test_torque += CALIB_TORQUE_STEP;
        }
    };

    switch (calib_stage)
    {
    case CalibStage::Left:
        apply_wheel_torque(robot, calib_test_torque, 0.0f);
        if (fabsf(robot.wL) > CALIB_VEL_THRESH)
        {
            robot.tor.dzL = fabsf(calib_test_torque);
            advance_stage(CalibStage::Right);
        }
        else if ((now - calib_stage_start > CALIB_STAGE_TIMEOUT_MS) || (calib_test_torque > CALIB_TORQUE_MAX))
        {
            advance_stage(CalibStage::Right);
        }
        else
        {
            tick_torque();
        }
        break;
    case CalibStage::Right:
        apply_wheel_torque(robot, 0.0f, calib_test_torque);
        if (fabsf(robot.wR) > CALIB_VEL_THRESH)
        {
            robot.tor.dzR = fabsf(calib_test_torque);
            advance_stage(CalibStage::Done);
        }
        else if ((now - calib_stage_start > CALIB_STAGE_TIMEOUT_MS) || (calib_test_torque > CALIB_TORQUE_MAX))
        {
            advance_stage(CalibStage::Done);
        }
        else
        {
            tick_torque();
        }
        break;
    case CalibStage::Done:
        apply_wheel_torque(robot, 0.0f, 0.0f);
        calib_done_flag = true;
        storage_save_calib(robot.tor.dzL, robot.tor.dzR);
        return true;
    case CalibStage::Reset:
    default:
        break;
    }
    return false;
}
