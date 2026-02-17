#pragma once

#include <stdint.h>

// 初始化 RGB 灯（WS2812），count_default 可覆盖默认 5 颗
void my_rgb_init(int count_default = 5);

// 直接设置 RGB 数组（长度 = count*3），brightness 0~255
void my_rgb_set(const uint8_t *rgb, int len, uint8_t brightness = 255);

// 预设效果：0 关闭；1 呼吸；2 彩虹；其他可扩展
void my_rgb_preset(int mode, int count = 5);
