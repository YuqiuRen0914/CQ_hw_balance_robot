#pragma once

#include "SimpleFOC.h"

extern BLDCMotor motor_1;
extern BLDCMotor motor_2;
extern MagneticSensorI2C sensor_1;
extern MagneticSensorI2C sensor_2;

void my_motor_init();
void my_motor_update();
