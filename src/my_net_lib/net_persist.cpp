// 说明：网络相关的持久化存取（WS 密码、设备名、WiFi、pitch 零点）
#include <Arduino.h>
#include "net_persist.h"
#include "my_storage.h"

namespace
{
constexpr const char *NVS_KEY_WS_PASSWORD = "ws_pwd";
constexpr const char *NVS_KEY_ROBOT_NAME = "robot_name";
constexpr const char *NVS_KEY_WIFI_SSID = "wifi_ssid";
constexpr const char *NVS_KEY_WIFI_PASS = "wifi_pass";
constexpr const char *NVS_KEY_PITCH_ZERO = "pitch_zero";
} // namespace

void net_persist_load(NetPersist &out, float pitch_zero_default)
{
    // 加载 WebSocket 密码
    storage_load_string(NVS_KEY_WS_PASSWORD, out.ws_password);
    
    // 加载设备名
    storage_load_string(NVS_KEY_ROBOT_NAME, out.robot_name);
    
    // 加载 WiFi 配置
    storage_load_string(NVS_KEY_WIFI_SSID, out.wifi_ssid);
    storage_load_string(NVS_KEY_WIFI_PASS, out.wifi_pass);
    
    // 加载 pitch 零点
    storage_load_float(NVS_KEY_PITCH_ZERO, out.pitch_zero);
    if (out.pitch_zero == 0.0f) {
        out.pitch_zero = pitch_zero_default;
    }
}

void net_persist_save_password(const String &pwd)
{
    storage_save_string(NVS_KEY_WS_PASSWORD, pwd);
}

void net_persist_save_name(const String &name)
{
    storage_save_string(NVS_KEY_ROBOT_NAME, name);
}

void net_persist_save_wifi(const String &ssid, const String &pass)
{
    storage_save_string(NVS_KEY_WIFI_SSID, ssid);
    storage_save_string(NVS_KEY_WIFI_PASS, pass);
}

void net_persist_save_pitch_zero(float v)
{
    storage_save_float(NVS_KEY_PITCH_ZERO, v);
}
