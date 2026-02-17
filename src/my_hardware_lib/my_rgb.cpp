#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <algorithm>
#include "my_rgb.h"
#include "my_config.h"

// 全局灯带对象（单路）
namespace
{
Adafruit_NeoPixel strip(RGB_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);
int led_count = RGB_COUNT;

void show_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < led_count; ++i)
        strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
}
} // namespace

void my_rgb_init(int count_default)
{
    led_count = count_default > 0 ? count_default : RGB_COUNT;
    strip.updateLength(led_count);
    strip.setPin(RGB_PIN);
    strip.begin();
    strip.setBrightness(255);
    show_all(0, 0, 0);
}

void my_rgb_set(const uint8_t *rgb, int len, uint8_t brightness)
{
    if (!rgb || len <= 0)
        return;
    strip.setBrightness(brightness);
    const int count = min(led_count, len / 3);
    for (int i = 0; i < count; ++i)
    {
        const int idx = i * 3;
        strip.setPixelColor(i, strip.Color(rgb[idx], rgb[idx + 1], rgb[idx + 2]));
    }
    strip.show();
}

void my_rgb_preset(int mode, int count)
{
    if (count > 0)
    {
        led_count = count;
        strip.updateLength(led_count);
        strip.begin();
    }

    switch (mode)
    {
    case 0: // off
        show_all(0, 0, 0);
        break;
    case 1: // 彩虹流光
    {
        for (int i = 0; i < led_count; ++i)
        {
            uint8_t hue = (i * 256 / led_count) & 0xFF;
            uint32_t c = strip.gamma32(strip.ColorHSV(hue * 256)); // ColorHSV uses 0-65535
            strip.setPixelColor(i, c);
        }
        strip.show();
        break;
    }
    case 2: // 呼吸（柔和白光）
    {
        for (int i = 0; i < led_count; ++i)
            strip.setPixelColor(i, strip.Color(80, 80, 100));
        strip.show();
        break;
    }
    case 3: // 流星拖尾（白头+蓝尾）
    {
        const int tail = std::max(1, led_count / 3);
        for (int i = 0; i < led_count; ++i)
        {
            int head = i;
            float t = float(tail - (i % tail)) / tail;
            uint8_t r = (uint8_t)(180 * t);
            uint8_t g = (uint8_t)(200 * t);
            uint8_t b = (uint8_t)(255 * t);
            strip.setPixelColor(head, strip.Color(r, g, b));
        }
        strip.show();
        break;
    }
    case 4: // 心跳律动（红色常亮为主）
    {
        for (int i = 0; i < led_count; ++i)
            strip.setPixelColor(i, strip.Color(200, 20, 20));
        strip.show();
        break;
    }
    default:
        show_all(0, 0, 0);
        break;
    }
}
// 说明：WS2812 RGB 控制（初始化、直接写入、四种网页端预设灯效）
