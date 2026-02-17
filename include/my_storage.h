#pragma once

#include <Arduino.h>

// 简单的 NVS 存取封装
bool storage_load_calib(float &dzL, float &dzR);
void storage_save_calib(float dzL, float dzR);

bool storage_load_gyro_bias(float &gx, float &gy, float &gz);
void storage_save_gyro_bias(float gx, float gy, float gz);

// 通用键值存取（字符串 / 浮点），便于网络配置、命名等功能复用
bool storage_load_string(const char *key, String &out);
void storage_save_string(const char *key, const String &value);

bool storage_load_float(const char *key, float &out);
void storage_save_float(const char *key, float value);
