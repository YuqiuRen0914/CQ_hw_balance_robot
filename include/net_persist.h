#pragma once

#include <Arduino.h>

struct NetPersist
{
    String ws_password;
    String robot_name;
    String wifi_ssid;
    String wifi_pass;
    float pitch_zero;
};

void net_persist_load(NetPersist &out, float pitch_zero_default);
void net_persist_save_password(const String &pwd);
void net_persist_save_name(const String &name);
void net_persist_save_wifi(const String &ssid, const String &pass);
void net_persist_save_pitch_zero(float v);
