#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "my_config.h"
#include "my_bat.h"
#include "my_i2c.h"

void my_screen_init();
void my_screen_update();

// 接收外部屏幕数据，mode: "mono" (1bpp) 或 "rgb565"
// data 指向编码后的字节数组，len 需匹配 width*height/8 或 width*height*2
void my_screen_draw_buffer(const uint8_t *data, size_t len, uint16_t width, uint16_t height, const char *mode);
