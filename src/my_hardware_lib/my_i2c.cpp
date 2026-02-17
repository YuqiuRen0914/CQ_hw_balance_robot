#include <Arduino.h>
#include "my_i2c.h"
#include "my_config.h"

void my_i2c_init()
{
    Wire0.begin(I2C_SDA_1, I2C_SCL_1, I2C_FREQUENCY);
    Wire1.begin(I2C_SDA_2, I2C_SCL_2, I2C_FREQUENCY);
}
// 说明：初始化双 I2C 总线（Wire0/Wire1）供传感器与外设使用
