#pragma once

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

bool handle_auth_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc);
bool handle_control_cmd(const char *type, JsonDocument &doc);
bool handle_freq_cmd(const char *type, JsonDocument &doc);
bool handle_pid_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc);
bool handle_motion_cmd(const char *type, JsonDocument &doc);
bool handle_rgb_cmd(const char *type, JsonDocument &doc);
bool handle_screen_cmd(const char *type, JsonDocument &doc);
bool handle_wifi_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc);
bool handle_info_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc);
