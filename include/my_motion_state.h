#pragma once

#include <stdint.h>

// 运动状态机状态定义
enum class MotionState
{
    Init,
    Calibrating,
    Idle,
    Normal,
    OffGround,
    Test,
    LowBat,
    Fallen,
    EStop,
    Fault,
    Shutdown
};

// 状态机输入
struct MotionInputs
{
    bool run_cmd;
    bool test_cmd;
    bool wel_up;
    bool fallen;
    bool fallen_recover_ready;
    bool calib_done;
    bool estop;
    bool drv_fault;
    float batt_v;
    float batt_warn;
    float batt_empty;
    bool no_op;       // 摇杆是否无操作（供上层零点逻辑使用）
    bool recalib_req; // 运行时重标定请求
};

// 状态机决策输出
struct MotionDecision
{
    MotionState state;
    bool control_allowed;
    bool lowbat_warn;
};

MotionDecision motion_state_step(MotionState current, const MotionInputs &in, uint32_t now_ms);
const char *motion_state_name(MotionState state);
