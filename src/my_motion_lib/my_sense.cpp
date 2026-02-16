#include <Arduino.h>
#include <cmath>
#include "my_sense.h"
#include "my_foc.h"
#include "my_config.h"
#include "my_storage.h"
#include "my_mpu6050.h"
#include "my_i2c.h"

// 单次刷新左右轮角速度并缓存
void sense_update_wheel_speeds(robot_state &robot)
{
    sensor_1.update();
    sensor_2.update();
    robot.wL = sensor_1.getVelocity();
    robot.wR = sensor_2.getVelocity();
}

// 更新姿态/航向/速度估计
void sense_update_attitude(robot_state &robot)
{
    robot.ang.last = robot.ang.now;
    robot.spd.last = robot.spd.now;
    robot.yaw.last = robot.yaw.now;

    robot.ang.now = robot.imu.angley;
    // wrap heading
    float yaw = robot.imu.anglez;
    while (yaw > 180.0f)
        yaw -= 360.0f;
    while (yaw < -180.0f)
        yaw += 360.0f;
    robot.yaw.now = yaw;

    robot.spd.now = 0.5f * (robot.wL + robot.wR);
}

void sense_wel_up_detect(robot_state &robot)
{
    const float wL = fabsf(robot.wL);
    const float wR = fabsf(robot.wR);
    const float pitch_abs = fabsf(robot.ang.now);
    const bool high_w = (wL > OFFGROUND_W_HIGH_RAD) || (wR > OFFGROUND_W_HIGH_RAD);
    const bool low_w = (wL < OFFGROUND_W_LOW_RAD) && (wR < OFFGROUND_W_LOW_RAD);
    const bool torque_enough = (fabsf(robot.tor.L) > OFFGROUND_T_MIN_NM) || (fabsf(robot.tor.R) > OFFGROUND_T_MIN_NM);

    static uint32_t off_enter_ms = 0;
    static uint32_t off_exit_ms = 0;
    const uint32_t now = millis();

    if (!robot.wel_up && high_w && torque_enough && pitch_abs < 6.0f)
    {
        if (off_enter_ms == 0)
            off_enter_ms = now;
        if (now - off_enter_ms >= OFFGROUND_T_SPIN_MS)
        {
            robot.wel_up = true;
            off_exit_ms = 0;
        }
    }
    else
    {
        off_enter_ms = 0;
    }

    if (robot.wel_up && low_w && pitch_abs < 6.0f)
    {
        if (off_exit_ms == 0)
            off_exit_ms = now;
        if (now - off_exit_ms >= OFFGROUND_T_DOWN_MS)
        {
            robot.wel_up = false;
            off_enter_ms = 0;
        }
    }
    else
    {
        off_exit_ms = 0;
    }
}

void sense_fall_check(robot_state &robot)
{
    if (!robot.fallen.enable)
        return;

    const float pitch = robot.ang.now;
    if (pitch > FALL_MAX_PITCH || pitch < FALL_MIN_PITCH)
    {
        robot.fallen.count++;
        if (robot.fallen.count >= COUNT_FALL_MAX)
            robot.fallen.is = true;
    }
    else
    {
        robot.fallen.count = 0;
        robot.fallen.is = false;
    }
}

bool sense_fallen_recover_ready(robot_state &robot)
{
    static uint32_t fallen_rec_ms = 0;
    const float pitch_abs = fabsf(robot.ang.now);
    const float gyroY_abs = fabsf(robot.imu.gyroy);
    const uint32_t now = millis();

    if (pitch_abs < FALLEN_REC_PITCH_DEG && gyroY_abs < FALLEN_REC_GYRO_DPS)
    {
        if (fallen_rec_ms == 0)
            fallen_rec_ms = now;
        if (now - fallen_rec_ms >= FALLEN_REC_MS)
            return true;
    }
    else
    {
        fallen_rec_ms = 0;
    }
    return false;
}

bool sense_no_op(const robot_state &robot)
{
    constexpr float kJoyDeadband = 0.05f;
    return (fabsf(robot.joy.x) < kJoyDeadband) && (fabsf(robot.joy.y) < kJoyDeadband);
}

// ------------- 陀螺零偏微校准 -------------
void sense_update_gyro_bias(robot_state &robot)
{
    static bool base_loaded = false;
    static bool base_calibrated = false;
    static uint32_t boot_ms = millis();
    static uint32_t accum_start = 0;
    static float acc_gx = 0, acc_gy = 0, acc_gz = 0;
    static uint16_t acc_cnt = 0;

    // 首次加载持久化基准
    if (!base_loaded)
    {
        float gx = robot.gyro_base.gx, gy = robot.gyro_base.gy, gz = robot.gyro_base.gz;
        base_calibrated = storage_load_gyro_bias(gx, gy, gz);
        if (base_calibrated)
        {
            robot.gyro_base = {gx, gy, gz};
            robot.gyro_run = robot.gyro_base;
        }
        base_loaded = true;
        boot_ms = millis();
        accum_start = 0;
        acc_cnt = 0;
        acc_gx = acc_gy = acc_gz = 0;
    }

    // 条件：静止且姿态平稳
    const bool quiet = (fabsf(robot.ang.now) < 8.0f) && (fabsf(robot.imu.gyroy) < 20.0f) && sense_no_op(robot);
    const uint32_t now = millis();

    bool want_calib = false;
    // 1) 上电后 1s 内自动微校准
    if (now - boot_ms < 1000U)
        want_calib = true;
    // 2) 外部请求 IMU 重新校准
    if (robot.imu_recalib_req)
        want_calib = true;

    if (want_calib && quiet)
    {
        if (accum_start == 0)
            accum_start = now;
        acc_gx += robot.imu.gyrox;
        acc_gy += robot.imu.gyroy;
        acc_gz += robot.imu.gyroz;
        acc_cnt++;
        if (now - accum_start >= 500) // 累计 0.5s
        {
            float gx = acc_gx / acc_cnt;
            float gy = acc_gy / acc_cnt;
            float gz = acc_gz / acc_cnt;
            robot.gyro_run = {gx, gy, gz};

            // 如果是手动重校，则更新基准并存储
            if (robot.imu_recalib_req)
            {
                robot.gyro_base = robot.gyro_run;
                storage_save_gyro_bias(robot.gyro_base.gx, robot.gyro_base.gy, robot.gyro_base.gz);
                robot.imu_recalib_req = false;
            }
            // reset accumulator
            accum_start = 0;
            acc_cnt = 0;
            acc_gx = acc_gy = acc_gz = 0;
        }
    }
    else
    {
        accum_start = 0;
        acc_cnt = 0;
        acc_gx = acc_gy = acc_gz = 0;
    }

    // 应用运行偏置到当前读数
    robot.imu.gyrox -= robot.gyro_run.gx;
    robot.imu.gyroy -= robot.gyro_run.gy;
    robot.imu.gyroz -= robot.gyro_run.gz;
}

// 静止自适应 pitch 零点：缓慢逼近当前姿态
void sense_adapt_pitch_zero(robot_state &robot)
{
    const bool quiet = (fabsf(robot.ang.now) < ZERO_ADAPT_DEADBAND_DEG) &&
                       (fabsf(robot.imu.gyroy) < ZERO_ADAPT_GYRO_DPS) &&
                       sense_no_op(robot);
    if (!quiet)
        return;
    const float err = robot.ang.now - robot.pitch_zero;
    robot.pitch_zero += err * ZERO_ADAPT_RATE;
}

// I2C 设备应答检测：MPU6050 与左右 AS5600（两路总线）
bool sense_check_i2c_fault(robot_state &robot)
{
    auto ping = [](TwoWire &w, uint8_t addr) {
        w.beginTransmission(addr);
        return w.endTransmission() == 0;
    };

    bool ok_mpu = ping(Wire0, ADDR_MPU6050);
    bool ok_as_l = ping(Wire0, ADDR_AS5600);
    bool ok_as_r = ping(Wire1, ADDR_AS5600);

    robot.drv_fault = !(ok_mpu && ok_as_l && ok_as_r);
    return robot.drv_fault;
}
