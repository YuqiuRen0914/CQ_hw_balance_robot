#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "my_net.h"
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_config.h"
#include "my_bat.h"

namespace
{
constexpr const char *AP_SSID = "BalBot";
constexpr const char *AP_PASS = "balbot123"; // 简单默认，可在网页后续配置

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 遥测周期
volatile uint32_t telem_ms = 500;

String json_state()
{
    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry";
    doc["battery"] = battery_voltage;
    doc["state"] = motion_state_name(robot.state);
    doc["fault"] = robot.drv_fault;
    doc["lowbat"] = robot.lowbat_warn;
    doc["pitch"] = robot.ang.now;
    doc["tor"] = robot.tor.base;
    doc["wel_up"] = robot.wel_up;
    doc["fallen"] = robot.fallen.is;
    String out;
    serializeJson(doc, out);
    return out;
}

void broadcast_state()
{
    const String payload = json_state();
    ws.textAll(payload);
}

void handle_ws_message(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT))
        return;

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err)
        return;

    const char *type = doc["type"] | "";
    if (strcmp(type, "robot_run") == 0)
    {
        robot.run = doc["running"] | false;
    }
    else if (strcmp(type, "fallen_protect") == 0)
    {
        robot.fallen.enable = doc["enable"] | true;
    }
    else if (strcmp(type, "offground_protect") == 0)
    {
        robot.offground_protect = doc["enable"] | true;
    }
    else if (strcmp(type, "imu_recalib") == 0)
    {
        robot.imu_recalib_req = true;
    }
    else if (strcmp(type, "recalib") == 0)
    {
        robot.recalib_req = true;
    }
    else if (strcmp(type, "estop") == 0)
    {
        robot.estop = doc["active"] | false;
    }
    else if (strcmp(type, "system_restart") == 0)
    {
        ESP.restart();
    }
    else if (strcmp(type, "telem_hz") == 0)
    {
        uint32_t ms = doc["ms"] | 500;
        telem_ms = ms == 0 ? 500 : ms;
    }
    broadcast_state();
}

void ws_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        client->text(json_state());
        break;
    case WS_EVT_DATA:
        handle_ws_message(arg, data, len);
        break;
    case WS_EVT_DISCONNECT:
    default:
        break;
    }
}

void telem_task(void *)
{
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        broadcast_state();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(telem_ms));
    }
}
TaskHandle_t telem_handle = nullptr;
} // namespace

void my_net_init()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "OK");
    });
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });

    ws.onEvent(ws_event);
    server.addHandler(&ws);
    server.begin();

    xTaskCreatePinnedToCore(telem_task, "telem", 4096, nullptr, 1, &telem_handle, 1);
}
