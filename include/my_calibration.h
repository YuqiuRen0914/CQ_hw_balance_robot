#pragma once
#include "my_config.h"

void calibration_reset(bool force_recalib);
void calibration_load_saved(robot_state &robot, bool has_saved, float dzL, float dzR, bool force_recalib);
bool calibration_step(robot_state &robot);
bool calibration_done();
