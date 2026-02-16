// Battery monitor for ESP32-S3
#include <Arduino.h>
#include <esp32-hal-adc.h>
#include <algorithm>
#include <cmath>
#include "my_bat.h"
#include "my_config.h"

float battery_voltage = 0.0f;

namespace
{
constexpr float divider_ratio = BAT_DIVIDER_RATIO; 

constexpr uint8_t adc_resolution_bits = 12;
constexpr uint8_t oversample = 4;       // 单次更新时的过采样次数
constexpr uint8_t median_window = 3;    // 前端中值窗口
constexpr float outlier_delta = 0.50f;  // 夹逼去极值阈值 (V)
constexpr float fast_iir_alpha = 0.35f; // 一阶 IIR 系数，越大响应越快
constexpr uint32_t update_interval_ms = BAT_CHECK_TIME * 1000UL; // 采样周期

float median_buf[median_window];
uint8_t median_head = 0;
uint8_t median_fill = 0;

float last_spike_free = 0.0f;
bool has_spike_free = false;

float fast_iir = 0.0f;
bool has_fast = false;

uint32_t last_sample_ms = 0;

// 单次读取电池电压：多次采样求平均，并按分压比换算为电池端电压
float read_battery_once()
{
    uint32_t acc_mv = 0;
    for (uint8_t i = 0; i < oversample; ++i)
    {
        acc_mv += static_cast<uint32_t>(analogReadMilliVolts(BAT_PIN));
    }
    const float pin_v = static_cast<float>(acc_mv) / (oversample * 1000.0f); // GPIO 电压
    return pin_v * divider_ratio;                                            // 反推电池端电压
}

// 针对偶发跳变做夹逼，限制到最近平稳值 ± outlier_delta
float clamp_outlier(float sample)
{
    if (!has_spike_free)
    {
        last_spike_free = sample;
        has_spike_free = true;
        return sample;
    }

    const float delta = sample - last_spike_free;
    const float limited = (fabsf(delta) > outlier_delta)
                              ? last_spike_free + (delta > 0 ? outlier_delta : -outlier_delta)
                              : sample;
    last_spike_free = limited;
    return limited;
}

// 中值滤波抑制尖峰
float push_median(float sample)
{
    median_buf[median_head] = sample;
    median_head = (median_head + 1) % median_window;
    if (median_fill < median_window)
        ++median_fill;

    float tmp[median_window];
    for (uint8_t i = 0; i < median_fill; ++i)
        tmp[i] = median_buf[i];
    std::sort(tmp, tmp + median_fill);
    return tmp[median_fill / 2];
}

// 低延迟通道：一阶 IIR 平滑
float push_fast_iir(float sample)
{
    if (!has_fast)
    {
        fast_iir = sample;
        has_fast = true;
        return fast_iir;
    }
    fast_iir += fast_iir_alpha * (sample - fast_iir);
    return fast_iir;
}

} // namespace

// 初始化 ADC 参数与滤波状态
void my_bat_init()
{
    analogReadResolution(adc_resolution_bits);
    analogSetPinAttenuation(BAT_PIN, ADC_11db); // 量程约 0~3.6V，对应 3S 分压后端

    median_head = median_fill = 0;
    has_spike_free = false;
    has_fast = false;
    battery_voltage = 0.0f;
    last_sample_ms = 0;
}

// 周期采样电池电压并更新 battery_voltage（已平滑）
void my_bat_update()
{
    const uint32_t now = millis();
    // 首次尚未建立滤波时立即采样，之后按周期采样
    if (has_fast && (now - last_sample_ms < update_interval_ms))
        return;
    last_sample_ms = now;

    float v = read_battery_once();
    v = clamp_outlier(v);
    v = push_median(v);
    v = push_fast_iir(v);

    // 合理范围裁剪，避免偶发异常值
    const float max_reasonable = BAT_FULL_VOLTAGE * 1.3f;
    if (v < 0.0f)
        v = 0.0f;
    else if (v > max_reasonable)
        v = max_reasonable;

    battery_voltage = v;
}
