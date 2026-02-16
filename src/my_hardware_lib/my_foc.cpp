#include "Arduino.h"
#include "my_foc.h"
#include "my_i2c.h"
#include "my_config.h"
#include "my_motion.h"
#include "my_bat.h"

BLDCMotor motor_1 = BLDCMotor(7);
BLDCMotor motor_2 = BLDCMotor(7);

MagneticSensorI2C sensor_1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_2 = MagneticSensorI2C(AS5600_I2C);

BLDCDriver3PWM driver_1(DRIVER1_IN1, DRIVER1_IN2, DRIVER1_IN3, DRIVER_EN);
BLDCDriver3PWM driver_2(DRIVER2_IN1, DRIVER2_IN2, DRIVER2_IN3, DRIVER_EN);

// 根据实时电池电压调整驱动供电参数，避免电量高低导致力感变化
static inline void update_supply_from_battery()
{
    // 未初始化时用满电作为兜底
    const float supply = (battery_voltage > 1.0f) ? battery_voltage : BAT_FULL_VOLTAGE;

    driver_1.voltage_power_supply = supply;
    driver_2.voltage_power_supply = supply;

    // 将输出上限跟随供电，预留 15% 余量防止触顶
    const float limit = supply * 0.85f;
    motor_1.voltage_limit = limit;
    motor_2.voltage_limit = limit;
}
void my_motor_init() { 
    // 初始化传感器
    sensor_1.init(&Wire0);
    sensor_2.init(&Wire1);
    // 传感器与电机关联
    motor_1.linkSensor(&sensor_1);
    motor_2.linkSensor(&sensor_2);
    // 初始化驱动器
    driver_1.init();
    driver_2.init();
    // 电机与驱动器关联
    motor_1.linkDriver(&driver_1);
    motor_2.linkDriver(&driver_2);

    motor_1.controller = torque;
    motor_2.controller = torque;

    motor_1.torque_controller = voltage;
    motor_2.torque_controller = voltage;

    motor_1.voltage_sensor_align = 3.0f;
    motor_2.voltage_sensor_align = 3.0f;

    update_supply_from_battery();

    motor_1.useMonitoring(Serial);
    motor_2.useMonitoring(Serial);

    motor_1.init();
    motor_1.initFOC();
    motor_2.init();
    motor_2.initFOC();
    Serial.println("电机初始化完成");
}

void my_motor_update() {
    update_supply_from_battery();

    motor_1.target = robot.tor.L;
    motor_2.target = robot.tor.R;

    motor_1.loopFOC();
    motor_2.loopFOC();

    motor_1.move();
    motor_2.move();
}
