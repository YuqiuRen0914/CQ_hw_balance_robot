// 说明：网络全局状态与下行消息封装，含 telemetry/extended 与后台任务
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>

#include "net_state.h"
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_bat.h"
#include "my_screen.h"
#include "my_rgb.h"
#include "my_control.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile uint32_t telem_ms = 500;
volatile uint32_t ext_ms = 100; // 10Hz
bool charts_send_on = false;
bool attitude_send_on = true;

NetPersist persist{
    .ws_password = "",
    .robot_name = "BalBot",
    .wifi_ssid = "",
    .wifi_pass = "",
    .pitch_zero = -2.1f,
};

const char *NET_AP_SSID = "BalBot";
const char *NET_AP_PASS = "balbot123";
const char *NET_FW_VERSION = "v0.1.0";

float battery_pct()
{
    float v = battery_voltage;
    if (v < BAT_PCT_MIN_V)
        return 0.0f;
    if (v > BAT_PCT_MAX_V)
        return 100.0f;
    return (v - BAT_PCT_MIN_V) * 100.0f / (BAT_PCT_MAX_V - BAT_PCT_MIN_V);
}

String sta_ip()
{
    return WiFi.localIP().toString();
}

String ap_ip()
{
    return WiFi.softAPIP().toString();
}

bool ip_valid(const String &ip)
{
    return ip != "0.0.0.0";
}

String current_ip()
{
    String ip = sta_ip();
    if (ip_valid(ip))
        return ip;
    ip = ap_ip();
    if (ip_valid(ip))
        return ip;
    return "0.0.0.0";
}

void send_json(AsyncWebSocketClient *client, const JsonDocument &doc)
{
    String out;
    serializeJson(doc, out);
    if (client)
        client->text(out);
    else
        ws.textAll(out);
}

void send_state(AsyncWebSocketClient *client)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "state";
    doc["auth_required"] = !persist.ws_password.isEmpty();
    doc["name"] = persist.robot_name;
    doc["fw_version"] = NET_FW_VERSION;
    doc["ip"] = current_ip();
    doc["battery"] = battery_pct();
    doc["voltage"] = battery_voltage;
    doc["state"] = motion_state_name(robot.state);
    send_json(client, doc);
}

void send_auth_status(AsyncWebSocketClient *client, const char *status)
{
    if (!client)
        return;
    StaticJsonDocument<96> doc;
    doc["type"] = "auth";
    doc["status"] = status;
    send_json(client, doc);
}

void send_sys_info(AsyncWebSocketClient *client)
{
    if (!client)
        return;
    StaticJsonDocument<192> doc;
    doc["type"] = "sys_info";
    doc["name"] = persist.robot_name;
    doc["fw_version"] = NET_FW_VERSION;
    doc["ip"] = current_ip();
    send_json(client, doc);
}

void send_wifi_config(AsyncWebSocketClient *client)
{
    if (!client)
        return;
    StaticJsonDocument<192> doc;
    doc["type"] = "wifi_config";
    doc["ssid"] = persist.wifi_ssid;
    doc["ip"] = current_ip();
    send_json(client, doc);
}

void send_wifi_save_status(AsyncWebSocketClient *client, const char *status, const char *message)
{
    if (!client)
        return;
    StaticJsonDocument<160> doc;
    doc["type"] = "wifi_save_status";
    doc["status"] = status;
    if (message && message[0])
        doc["message"] = message;
    send_json(client, doc);
}

void send_pid(AsyncWebSocketClient *client)
{
    if (!client)
        return;
    StaticJsonDocument<256> doc;
    doc["type"] = "pid";
    JsonObject p = doc.createNestedObject("param");
    p["key01"] = robot.ang_pid.p;
    p["key02"] = robot.ang_pid.i;
    p["key03"] = robot.ang_pid.d;
    p["key04"] = robot.spd_pid.p;
    p["key05"] = robot.spd_pid.i;
    p["key06"] = robot.spd_pid.d;
    p["key07"] = 0.0f;
    p["key08"] = 0.0f;
    p["key09"] = 0.0f;
    p["key10"] = robot.yaw_pid.p;
    p["key11"] = robot.yaw_pid.i;
    p["key12"] = robot.yaw_pid.d;
    send_json(client, doc);
}

void send_pitch_zero(AsyncWebSocketClient *client)
{
    if (!client)
        return;
    StaticJsonDocument<96> doc;
    doc["type"] = "pitch_zero_state";
    doc["value"] = robot.pitch_zero;
    send_json(client, doc);
}

void send_torque_limit(AsyncWebSocketClient *client)
{
    if (!client)
        return;
    StaticJsonDocument<96> doc;
    doc["type"] = "torque_limit_state";
    doc["value"] = torque_limit;
    send_json(client, doc);
}

void send_deadzone(AsyncWebSocketClient *client)
{
    StaticJsonDocument<128> doc;
    doc["type"] = "deadzone";
    doc["dzL"] = robot.tor.dzL;
    doc["dzR"] = robot.tor.dzR;
    send_json(client, doc);
}

void send_schema(AsyncWebSocketClient *client)
{
    StaticJsonDocument<256> doc;
    doc["type"] = "schema";
    JsonObject s = doc.createNestedObject("schema");
    s["ang_tar"] = "Angle target";
    s["pitch"] = "Angle";
    s["spd_tar"] = "Speed target";
    s["spd_now"] = "Speed";
    s["torque_l"] = "Torque L";
    s["torque_r"] = "Torque R";
    s["speed_l"] = "Wheel L";
    s["speed_r"] = "Wheel R";
    s["gyro_y"] = "Gyro Y";
    s["acc_y"] = "Acc Y";
    s["battery"] = "Battery %";
    send_json(client, doc);
}

void broadcast_telemetry()
{
    StaticJsonDocument<384> doc;
    doc["type"] = "telemetry";
    doc["pitch"] = robot.ang.now;
    doc["roll"] = robot.imu.anglex;
    doc["yaw"] = robot.yaw.now;
    doc["fallen"] = robot.fallen.is;
    doc["battery"] = battery_pct();
    doc["voltage"] = battery_voltage;
    doc["state"] = motion_state_name(robot.state);
    doc["wel_up"] = robot.wel_up;
    doc["drv_fault"] = robot.drv_fault;
    doc["speed_l"] = robot.wL;
    doc["speed_r"] = robot.wR;
    doc["torque_l"] = robot.tor.L;
    doc["torque_r"] = robot.tor.R;
    doc["dzL"] = robot.tor.dzL;
    doc["dzR"] = robot.tor.dzR;
    send_json(nullptr, doc);
}

void broadcast_extended()
{
    if (!charts_send_on)
        return;
    StaticJsonDocument<384> doc;
    doc["type"] = "extended";
    JsonObject d = doc.createNestedObject("data");
    d["ang_tar"] = robot.ang.tar;
    d["pitch"] = robot.ang.now;
    d["spd_tar"] = robot.spd.tar;
    d["spd_now"] = robot.spd.now;
    d["torque_l"] = robot.tor.L;
    d["torque_r"] = robot.tor.R;
    d["speed_l"] = robot.wL;
    d["speed_r"] = robot.wR;
    d["gyro_y"] = robot.imu.gyroy;
    d["acc_y"] = robot.imu.angley;
    d["battery"] = battery_pct();
    send_json(nullptr, doc);
}

bool decode_base64(const String &in, std::vector<uint8_t> &out)
{
    size_t out_len = 0;
    int ret = mbedtls_base64_decode(nullptr, 0, &out_len, (const unsigned char *)in.c_str(), in.length());
    if (ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL && ret != 0)
        return false;
    out.resize(out_len);
    ret = mbedtls_base64_decode(out.data(), out_len, &out_len, (const unsigned char *)in.c_str(), in.length());
    if (ret != 0)
        return false;
    out.resize(out_len);
    return true;
}

// telemetry / extended tasks
void telem_task(void *)
{
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        if (attitude_send_on)
            broadcast_telemetry();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(telem_ms));
    }
}

void ext_task(void *)
{
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        broadcast_extended();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ext_ms));
    }
}

void net_start_tasks()
{
    static TaskHandle_t telem_handle = nullptr;
    static TaskHandle_t ext_handle = nullptr;
    xTaskCreatePinnedToCore(telem_task, "telem", 4096, nullptr, 1, &telem_handle, 1);
    xTaskCreatePinnedToCore(ext_task, "ext", 4096, nullptr, 1, &ext_handle, 1);
}
