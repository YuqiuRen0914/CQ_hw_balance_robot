#include "my_motion_state.h"             
#include <cmath>                         

namespace                                    
{
constexpr float kLowbatReleaseDelta = 0.3f;   // 低电恢复滞回阈值

bool is_shutdown(float v, float batt_empty)   // 判定是否触发关机
{
    return v <= batt_empty;                   // 电压低于 empty 阈值即关机
}

bool is_lowbat_enter(float v, float batt_warn) // 判定进入低电告警
{
    return v <= batt_warn;                     // 电压低于 warn 阈值
}

bool is_lowbat_exit(float v, float batt_warn)  // 判定退出低电告警
{
    return v >= batt_warn + kLowbatReleaseDelta; // 电压高于 warn+滞回
}

} // namespace

const char *motion_state_name(MotionState state) 
{
    switch (state)                                // 按状态返回名字
    {
    case MotionState::Init:
        return "Init";
    case MotionState::Calibrating:
        return "Calibrating";
    case MotionState::Idle:
        return "Idle";
    case MotionState::Normal:
        return "Normal";
    case MotionState::OffGround:
        return "OffGround";
    case MotionState::Test:
        return "Test";
    case MotionState::LowBat:
        return "LowBat";
    case MotionState::Fallen:
        return "Fallen";
    case MotionState::EStop:
        return "EStop";
    case MotionState::Fault:
        return "Fault";
    case MotionState::Shutdown:
        return "Shutdown";
    default:
        return "Unknown";                        // 兜底
    }
}

static MotionDecision make_decision(MotionState state, bool control_allowed, bool lowbat_warn) // 构造决策便捷函数
{
    MotionDecision d{state, control_allowed, lowbat_warn}; // 聚合结果
    return d;                                              // 返回
}

MotionDecision motion_state_step(MotionState current, const MotionInputs &in, uint32_t /*now_ms*/) // 状态机单步
{
    // 优先级：EStop > Fault > Shutdown > Fallen > OffGround > LowBat > Normal/Test/Idle

    if (in.estop)                                     // 急停最高优先级
    {
        return make_decision(MotionState::EStop, false, false); // 急停禁用控制
    }
    if (current == MotionState::EStop && !in.estop)   // 急停解除
    {
        return make_decision(MotionState::Idle, false, false);
    }

    if (in.drv_fault)                                 // 驱动/传感器故障
    {
        return make_decision(MotionState::Fault, false, false); // 进入故障态
    }
    if (current == MotionState::Fault && !in.drv_fault) // 故障解除
    {
        return make_decision(MotionState::Calibrating, false, false);
    }

    if (is_shutdown(in.batt_v, in.batt_empty))        // 电量耗尽
    {
        return make_decision(MotionState::Shutdown, false, false); // 关机停控
    }

    if (in.fallen)                                    // 摔倒检测
    {
        return make_decision(MotionState::Fallen, false, false); // 摔倒停控
    }
    if (current == MotionState::Fallen && in.fallen_recover_ready) // 摔倒自动恢复
    {
        return make_decision(MotionState::Calibrating, false, false);
    }

    if (in.wel_up)                                    // 离地检测
    {
        return make_decision(MotionState::OffGround, false, false); // 离地停控
    }
    if (current == MotionState::OffGround && !in.wel_up) // 落地恢复
    {
        return make_decision(in.run_cmd ? MotionState::Normal : MotionState::Idle, in.run_cmd, false);
    }

    if (is_lowbat_enter(in.batt_v, in.batt_warn))     // 低电进入/保持
    {
        return make_decision(MotionState::LowBat, true, true); // 告警但允许控制
    }

    if (current == MotionState::LowBat && !is_lowbat_exit(in.batt_v, in.batt_warn)) // 低电滞回保持
    {
        return make_decision(MotionState::LowBat, true, true); // 继续告警
    }

    // 正常/测试/待机状态转换
    switch (current)
    {
    case MotionState::Init:                           // 上电后
        return make_decision(MotionState::Calibrating, true, false); // 进入校准并允许控制
    case MotionState::Calibrating:                    // 校准完成
        if (in.calib_done)
            return make_decision(MotionState::Idle, false, false); // 校准完成后待机
        return make_decision(MotionState::Calibrating, true, false); // 继续校准
    case MotionState::Idle:                           // 待机逻辑
        if (in.test_cmd)
            return make_decision(MotionState::Test, true, false); // 进入测试态
        if (in.run_cmd)
            return make_decision(MotionState::Normal, true, false); // 进入运行态
        return make_decision(MotionState::Idle, false, false); // 保持待机
    case MotionState::Normal:                         // 运行态
        if (in.test_cmd)
            return make_decision(MotionState::Test, true, false); // 切到测试
        if (!in.run_cmd)
            return make_decision(MotionState::Idle, false, false); // 停止回待机
        return make_decision(MotionState::Normal, true, false); // 继续运行
    case MotionState::Test:                           // 测试态
        if (!in.test_cmd)
            return make_decision(MotionState::Idle, false, false); // 退出测试回待机
        return make_decision(MotionState::Test, true, false); // 继续测试
    case MotionState::OffGround:
    case MotionState::Fallen:
    case MotionState::EStop:
    case MotionState::Fault:
    case MotionState::Shutdown:
        // 已在上方处理各异常态的退出，默认保持当前
        return make_decision(current, false, false);
    case MotionState::LowBat:
        // LowBat 已在上方处理，这里理论不会到达
        return make_decision(MotionState::LowBat, true, true);
    default:                                          // 未知状态兜底
        return make_decision(MotionState::Fault, false, false);
    }
}
