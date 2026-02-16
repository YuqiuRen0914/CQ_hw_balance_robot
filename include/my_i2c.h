#pragma once
#include <Wire.h>
#include "SimpleFOC.h"

// 双 I2C 总线：Wire0 映射到核心默认 Wire，Wire1 使用核心的第二路 I2C
#define Wire0 Wire

void my_i2c_init();
