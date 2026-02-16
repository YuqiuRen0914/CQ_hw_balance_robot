#pragma once

#include <Arduino.h>
#include "my_motion_state.h"

/********** I2C **********/
#define I2C_SDA_1 4     //mpu6050+左as5600
#define I2C_SCL_1 5     
#define I2C_SDA_2 1     //screen+右as5600
#define I2C_SCL_2 2
#define I2C_FREQUENCY 400000

/********** SCREEN (SSD1306) **********/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_REFRESH_TIME 100 // ms
#define SCREEN_I2C_ADDRESS 0x3C // 标签0x78，7bit地址0x3C

/********** Power **********/
#define BAT_PIN 6
#define BAT_CHECK_TIME 2
#define BAT_FULL_VOLTAGE 12.6f
#define BAT_WARNING_VOLTAGE 11.1f
#define BAT_EMPTY_VOLTAGE 10.0f
#define BAT_DIVIDER_RATIO 5.7f  // 外部分压比：Vbat ≈ Vadc * BAT_DIVIDER_RATIO

/********** Motor **********/
#define DRIVER_EN 13

#define DRIVER1_IN1 10
#define DRIVER1_IN2 11
#define DRIVER1_IN3 12

#define DRIVER2_IN1 9
#define DRIVER2_IN2 8
#define DRIVER2_IN3 7

/********** RGB **********/
#define RGB_PIN 3
#define RGB_COUNT 5

/********** 摔倒检测参数 **********/
#define COUNT_FALL_MAX 3 // 连续3次采样超限才算倒地
#define FALL_MAX_PITCH +30.0f
#define FALL_MIN_PITCH -30.0f

/********** 速度环 → 倾角映射 **********/
#define PITCH_TAR_MAX_DEG 8.0f


/********** 力矩参数配置 **********/
#define TOR_SUM_LIM 25.0f



/********** 硬件外设结构体 **********/
struct imu_data
{
    float anglex;
    float angley;
    float anglez;
    float gyrox;
    float gyroy;
    float gyroz;
};

struct rgb_state
{
    int rgb_count; // 灯珠数量
    int mode;      // 灯光模式
};

struct motor_tor
{
    float base;
    float yaw;
    float L;
    float R;
    float dzL;  //死区补偿
    float dzR;  //死区补偿
};

/********** 控制结构体 **********/
struct pid_config
{
    float p, i, d;
    float k; // 斜率限制
    float l; // 最值限制
};

struct pid_state
{
    float now;
    float last;
    float tar;
    float err;
    float tor;
};

struct joy_state
{
    float x;
    float y;
    float x_coef;
    float y_coef;
};

struct fallen_state
{
    bool is;     // 是否摔倒
    int count;   // 计数
    bool enable; // 检测是否启用
};

struct robot_state
{
    int dt_ms;
    int data_ms;

    bool run;
    bool test_cmd;
    bool estop;
    bool drv_fault;
    bool chart_enable;
    bool joy_stop_control;
    bool wel_up;
    bool lowbat_warn;
    bool recalib_req; // 外部指令要求重新校准
    bool imu_recalib_req; // 外部指令要求IMU重新校准

    MotionState state;

    float pitch_zero;

    motor_tor tor;
    float wL; // 左轮角速度 (rad/s)
    float wR; // 右轮角速度 (rad/s)

    // 陀螺零偏：base 为长期基准，run 为本次上电微校准
    struct
    {
        float gx, gy, gz;
    } gyro_base, gyro_run;

    imu_data imu_zero;
    imu_data imu_l;
    imu_data imu;

    rgb_state rgb;

    joy_state joy;
    joy_state joy_l;

    fallen_state fallen;

    pid_state ang;
    pid_state spd;
    pid_state yaw;
    pid_config ang_pid;
    pid_config spd_pid;
    pid_config yaw_pid;
};

/********** 离地与恢复判定参数（基于 2804 电机） **********/
#define OFFGROUND_T_MIN_NM 0.015f   // 判定离地所需最小力矩 (Nm)
#define OFFGROUND_W_HIGH_RAD 150.0f // 进入离地的轮速阈值 (rad/s)
#define OFFGROUND_W_LOW_RAD 50.0f   // 落地恢复的轮速阈值 (rad/s)
#define OFFGROUND_T_SPIN_MS 120     // 满足高轮速的持续时间 (ms)
#define OFFGROUND_T_DOWN_MS 400     // 落地稳定时间 (ms)

/********** 摔倒自动恢复判定 **********/
#define FALLEN_REC_PITCH_DEG 5.0f
#define FALLEN_REC_GYRO_DPS 5.0f
#define FALLEN_REC_MS 1000

/********** 摆动起立参数 **********/
#define SWING_MAX_TORQUE 6.0f         // 摆动最大力矩
#define SWING_MIN_TORQUE 2.5f         // 起摆最小力矩
#define SWING_FREQ_HZ 1.2f            // 摆动频率
#define SWING_EXIT_PITCH_DEG 25.0f    // 小于该角度认为摆动成功
#define SOFT_TAKEOVER_MS 600          // 摆动成功后 PID 渐进时间

/********** 传感器地址 **********/
#define ADDR_MPU6050 0x68
#define ADDR_AS5600  0x36

/********** 姿态零点自适应 **********/
#define ZERO_ADAPT_DEADBAND_DEG 2.0f   // 静止判定俯仰阈值
#define ZERO_ADAPT_GYRO_DPS     8.0f   // 静止判定角速度阈值
#define ZERO_ADAPT_RATE         0.0005f // 每周期零点缓动比例
