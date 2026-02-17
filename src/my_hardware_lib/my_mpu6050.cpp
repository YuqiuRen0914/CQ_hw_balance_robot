#include "my_motion.h"
#include "my_mpu6050.h"
#include "my_i2c.h"
#include "my_config.h"
#include "Arduino.h"
#include <cmath>

MPU6050 mpu6050 = MPU6050(Wire0);

// ==================== Mahony AHRS 滤波器 ====================
static float mq0 = 1.0f, mq1 = 0.0f, mq2 = 0.0f, mq3 = 0.0f;
static float meIx = 0.0f, meIy = 0.0f, meIz = 0.0f;
static uint32_t mahony_last_us = 0;
static bool mahony_inited = false;

static void mahony_init_from_accel(float ax, float ay, float az)
{
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    mq0 =  cr * cp;
    mq1 =  sr * cp;
    mq2 =  cr * sp;
    mq3 = -sr * sp;
    meIx = meIy = meIz = 0.0f;
}

static void mahony_update(float ax, float ay, float az,
                          float gx_dps, float gy_dps, float gz_dps)
{
    uint32_t now_us = micros();
    float dt = (mahony_last_us == 0) ? 0.002f : (now_us - mahony_last_us) * 1e-6f;
    mahony_last_us = now_us;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

    constexpr float D2R = PI / 180.0f;
    float gx = gx_dps * D2R;
    float gy = gy_dps * D2R;
    float gz = gz_dps * D2R;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.01f) goto integrate;
    {
        float inv = 1.0f / norm;
        ax *= inv; ay *= inv; az *= inv;

        float vx = 2.0f * (mq1 * mq3 - mq0 * mq2);
        float vy = 2.0f * (mq0 * mq1 + mq2 * mq3);
        float vz = mq0 * mq0 - mq1 * mq1 - mq2 * mq2 + mq3 * mq3;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        meIx += ex * MAHONY_KI * dt;
        meIy += ey * MAHONY_KI * dt;
        meIz += ez * MAHONY_KI * dt;

        gx += MAHONY_KP * ex + meIx;
        gy += MAHONY_KP * ey + meIy;
        gz += MAHONY_KP * ez + meIz;
    }
integrate:
    {
        float hdt = 0.5f * dt;
        float dq0 = (-mq1 * gx - mq2 * gy - mq3 * gz) * hdt;
        float dq1 = ( mq0 * gx + mq2 * gz - mq3 * gy) * hdt;
        float dq2 = ( mq0 * gy - mq1 * gz + mq3 * gx) * hdt;
        float dq3 = ( mq0 * gz + mq1 * gy - mq2 * gx) * hdt;
        mq0 += dq0; mq1 += dq1; mq2 += dq2; mq3 += dq3;

        float qn = 1.0f / sqrtf(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);
        mq0 *= qn; mq1 *= qn; mq2 *= qn; mq3 *= qn;
    }
}

static float mahony_pitch_deg()
{
    float sinp = 2.0f * (mq0 * mq2 - mq3 * mq1);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    return asinf(sinp) * (180.0f / PI);
}

static float mahony_roll_deg()
{
    return atan2f(2.0f * (mq0 * mq1 + mq2 * mq3),
                  1.0f - 2.0f * (mq1 * mq1 + mq2 * mq2)) * (180.0f / PI);
}

static float mahony_yaw_deg()
{
    return atan2f(2.0f * (mq0 * mq3 + mq1 * mq2),
                  1.0f - 2.0f * (mq2 * mq2 + mq3 * mq3)) * (180.0f / PI);
}

// ==================== 公共接口 ====================

void my_mpu6050_init()
{
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    delay(1000);
    mahony_inited = false;
    mahony_last_us = 0;
    Serial.println("MPU6050初始化完成");
}

void my_mpu6050_update()
{
    mpu6050.update();

    float ax = mpu6050.getAccX();
    float ay = mpu6050.getAccY();
    float az = mpu6050.getAccZ();
    float gx = mpu6050.getGyroX();
    float gy = mpu6050.getGyroY();
    float gz = mpu6050.getGyroZ();

    if (!mahony_inited)
    {
        mahony_init_from_accel(ax, ay, az);
        mahony_inited = true;
    }

    mahony_update(ax, ay, az, gx, gy, gz);

    robot.imu.anglex = mahony_roll_deg();
    robot.imu.angley = mahony_pitch_deg();
    robot.imu.anglez = mahony_yaw_deg();
    robot.imu.gyrox  = gx;
    robot.imu.gyroy  = gy;
    robot.imu.gyroz  = gz;
}
// 说明：MPU6050 IMU 初始化 + Mahony AHRS 姿态融合，将姿态数据写入机器人状态
