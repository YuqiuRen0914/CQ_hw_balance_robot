#pragma once

// 简单的 NVS 存取封装
bool storage_load_calib(float &dzL, float &dzR);
void storage_save_calib(float dzL, float dzR);

bool storage_load_gyro_bias(float &gx, float &gy, float &gz);
void storage_save_gyro_bias(float gx, float gy, float gz);
