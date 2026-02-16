#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "my_config.h"
#include "my_bat.h"
#include "my_i2c.h"

void my_screen_init();
void my_screen_update();