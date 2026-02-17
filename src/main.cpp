#include <Arduino.h>
#include "my_i2c.h"
#include "my_mpu6050.h"
#include "my_bat.h"
#include "my_motion.h"
#include "my_foc.h"
#include "my_screen.h"
#include "my_net.h"

// FreeRTOS 任务句柄
static TaskHandle_t control_task_handle = nullptr;
static TaskHandle_t screen_task_handle = nullptr;

// 控制周期：从 robot.dt_ms 读取（单位 ms）
static inline TickType_t control_period_ticks()
{
    return pdMS_TO_TICKS(robot.dt_ms);
}

// 屏幕刷新周期：使用 SCREEN_REFRESH_TIME（单位 ms）
static inline TickType_t screen_period_ticks()
{
    return pdMS_TO_TICKS(SCREEN_REFRESH_TIME);
}

// 控制任务：高优先级，绑定核心 0
void control_task(void *)
{
    // 初始化时间基准
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        // 传感器更新
        my_mpu6050_update();
        my_bat_update();

        // 运动学与状态机
        my_motion_update();

        // 力矩输出到电机
        my_motor_update();

        // 周期调度
        vTaskDelayUntil(&last_wake, control_period_ticks());
    }
}

// 屏幕任务：中优先级，绑定核心 1
void screen_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        my_screen_update();
        vTaskDelayUntil(&last_wake, screen_period_ticks());
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    // 硬件初始化
    my_i2c_init();
    my_mpu6050_init();
    my_bat_init();
    my_motor_init();
    my_motion_init();
    my_screen_init();
    my_net_init();

    // 创建控制任务（核心0，高优先级）
    xTaskCreatePinnedToCore(
        control_task,         // 任务函数
        "control",            // 名称
        8192,                 // 栈
        nullptr,              // 参数
        configMAX_PRIORITIES - 1, // 优先级：最高
        &control_task_handle, // 句柄
        0);                   // 绑定 core0

    // 创建屏幕任务（核心1，中等优先级）
    xTaskCreatePinnedToCore(
        screen_task,
        "screen",
        4096,
        nullptr,
        3,                    // 中等优先级
        &screen_task_handle,
        1);                   // 绑定 core1
}

void loop()
{
    // 主循环空转，让出 CPU
    vTaskDelay(pdMS_TO_TICKS(1000));
}
// 说明：ESP32 平衡车入口，初始化硬件并创建控制/屏幕任务
