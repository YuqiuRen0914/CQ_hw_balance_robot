#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <vector>

#include "net_persist.h"
#include "my_config.h"

// 全局网络状态
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern volatile uint32_t telem_ms;
extern volatile uint32_t ext_ms;
extern bool charts_send_on;
extern bool attitude_send_on;
extern NetPersist persist;

extern const char *NET_AP_SSID;
extern const char *NET_AP_PASS;
extern const char *NET_FW_VERSION;

// 基础工具
float battery_pct();
String current_ip();
bool decode_base64(const String &in, std::vector<uint8_t> &out);

// 下行消息辅助
void send_json(AsyncWebSocketClient *client, const JsonDocument &doc);
void send_state(AsyncWebSocketClient *client = nullptr);
void send_auth_status(AsyncWebSocketClient *client, const char *status);
void send_sys_info(AsyncWebSocketClient *client);
void send_wifi_config(AsyncWebSocketClient *client);
void send_wifi_save_status(AsyncWebSocketClient *client, const char *status, const char *message = "");
void send_pid(AsyncWebSocketClient *client);
void send_pitch_zero(AsyncWebSocketClient *client);
void send_torque_limit(AsyncWebSocketClient *client);
void send_deadzone(AsyncWebSocketClient *client);
void send_schema(AsyncWebSocketClient *client);
void broadcast_telemetry();
void broadcast_extended();

// 背景任务启动
void net_start_tasks();
