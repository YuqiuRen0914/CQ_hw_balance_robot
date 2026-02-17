#include <algorithm>
#include <cmath>
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_sense.h"
#include "my_control.h"
#include "my_calibration.h"
#include "my_storage.h"
#include "my_foc.h"
#include "my_bat.h"

// 全局机器人状态
robot_state robot = {
    .dt_ms = 2,
    .data_ms = 100,
    .run = false,
    .test_cmd = false,
    .estop = false,
    .drv_fault = false,
    .chart_enable = false,
    .joy_stop_control = false,
    .wel_up = false,
    .lowbat_warn = false,
    .recalib_req = false,
    .imu_recalib_req = false,
    .offground_protect = true,
    .motor_mode = MODE_PWM,
    .state = MotionState::Init,
    .pitch_zero = -2.1f,
    .tor = {.base = 0.0f, .yaw = 0.0f, .L = 0.0f, .R = 0.0f, .dzL = 0.25f, .dzR = 0.25f},
    .wL = 0.0f,
    .wR = 0.0f,
    .gyro_base = {0, 0, 0},
    .gyro_run = {0, 0, 0},
    .imu_zero = {0, 0, 0, 0, 0, 0},
    .imu_l = {0, 0, 0, 0, 0, 0},
    .imu = {0, 0, 0, 0, 0, 0},
    .rgb = {0, 0},
    .joy = {0, 0, 0.1f, 10.0f},
    .joy_l = {0, 0, 0.1f, 10.0f},
    .fallen = {false, 0, false},
    .ang = {0, 0, 0, 0, 0},
    .spd = {0, 0, 0, 0, 0},
    .yaw = {0, 0, 0, 0, 0},
    .ang_pid = {0.6f, 5.0f, 0.016f, 100000, 250},
    .spd_pid = {0.003f, 0.0001f, 0.00f, 100000, 5},
    .yaw_pid = {0.025f, 0.00f, 0.00f, 100000, 5},
};

static MotionState prev_state = MotionState::Init;

// 汇总状态机输入
static MotionInputs collect_motion_inputs()
{
    MotionInputs in{};
    in.run_cmd = robot.run;
    in.test_cmd = robot.test_cmd;
    in.wel_up = robot.offground_protect ? robot.wel_up : false;
    in.fallen = robot.fallen.is;
    in.fallen_recover_ready = sense_fallen_recover_ready(robot);
    in.calib_done = calibration_done();
    in.estop = robot.estop;
    in.drv_fault = robot.drv_fault;
    in.batt_v = battery_voltage;
    in.batt_warn = BAT_WARNING_VOLTAGE;
    in.batt_empty = BAT_EMPTY_VOLTAGE;
    in.no_op = sense_no_op(robot);
    in.recalib_req = robot.recalib_req;
    return in;
}

void my_motion_init()
{
    robot.state = MotionState::Init;
    robot.lowbat_warn = false;
    robot.fallen.enable = true;
    robot.fallen.is = false;
    robot.fallen.count = 0;
    robot.wel_up = false;
    robot.test_cmd = false;
    robot.estop = false;
    robot.drv_fault = false;

    control_reset(robot);

    // 加载存档死区
    float dzL = robot.tor.dzL, dzR = robot.tor.dzR;
    bool has_saved = storage_load_calib(dzL, dzR);
    calibration_reset(robot.recalib_req);
    calibration_load_saved(robot, has_saved, dzL, dzR, robot.recalib_req);

    // 加载陀螺基准零偏
    float gx = robot.gyro_base.gx, gy = robot.gyro_base.gy, gz = robot.gyro_base.gz;
    if (storage_load_gyro_bias(gx, gy, gz))
    {
        robot.gyro_base = {gx, gy, gz};
        robot.gyro_run = robot.gyro_base;
    }

    prev_state = MotionState::Init;
}

void my_motion_update()
{
    // 传感与估计
    sense_update_wheel_speeds(robot);
    robot.imu_l = robot.imu; // 备份上一帧 IMU（外部更新已有）
    sense_update_attitude(robot);
    sense_update_gyro_bias(robot);

    // I2C 存活检测降频：避免每 2ms 做 3 次 I2C ping 引入控制环抖动
    {
        static uint32_t last_i2c_check = 0;
        const uint32_t now = millis();
        if (now - last_i2c_check >= I2C_FAULT_CHECK_MS)
        {
            last_i2c_check = now;
            sense_check_i2c_fault(robot);
        }
    }

    sense_adapt_pitch_zero(robot);
    sense_wel_up_detect(robot);
    sense_fall_check(robot);

    // 标定阶段
    if (prev_state != MotionState::Calibrating && robot.state == MotionState::Calibrating)
    {
        calibration_reset(robot.recalib_req);
        robot.recalib_req = false; // 消费请求，防止循环触发
    }
    if (robot.state == MotionState::Calibrating)
    {
        calibration_step(robot);
    }

    // 状态机
    MotionInputs inputs = collect_motion_inputs();
    MotionDecision decision = motion_state_step(robot.state, inputs, millis());
    robot.state = decision.state;
    robot.lowbat_warn = decision.lowbat_warn;

    // Calibrating 保持在校准逻辑，控制环不运行
    if (robot.state == MotionState::Calibrating)
    {
        prev_state = robot.state;
        return;
    }

    // 测试模式：允许外部 set_motor 直接写入力矩，不运行 PID
    if (robot.state == MotionState::Test)
    {
        // 在测试模式下，robot.tor.L/R 可能承载 PWM值(±1000)、速度(rad/s)或位置(deg)
        // 因此不再进行统一的力矩幅值限制(torque_limit=25.0)，避免误截断
        prev_state = robot.state;
        return;
    }

    if (robot.state == MotionState::Fallen)
    {
        // 摆动起立，不使用常规 PID
        control_swing_up(robot);
    }
    else if (decision.control_allowed)
    {
        control_update_pid(robot);
        control_pitch(robot);
        control_yaw(robot);
        control_torque_mix(robot);
        // 软接管：按进度放大输出
        if (control_soft_takeover_active())
        {
            const float k = control_soft_takeover_gain(); // 0~1
            robot.tor.L *= k;
            robot.tor.R *= k;
        }
    }
    else
    {
        // Shutdown/EStop/Fault：力矩清零
        control_reset(robot);
    }

    prev_state = robot.state;
}
// 说明：协调传感、状态机、标定挂钩与控制环集成
