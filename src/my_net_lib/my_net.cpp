// 说明：网络入口与路由，负责 WS 客户端管理、HTTP/OTA 注册与 WiFi 启动
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <Update.h>
#include <unordered_map>
#include <string.h>
#include "my_net.h"
#include "my_config.h"
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_control.h"
#include "my_foc.h"
#include "my_mpu6050.h"
#include "my_i2c.h"
#include "net_state.h"
#include "net_handlers.h"
#include "net_persist.h"
#include "my_rgb.h"

namespace
{
struct ClientMeta
{
    bool authed;
};

// ------------- Helpers -------------
std::unordered_map<uint32_t, ClientMeta> client_meta;

void wifi_start_ap()
{
    WiFi.softAP(NET_AP_SSID, NET_AP_PASS);
}

void wifi_start_sta()
{
    if (persist.wifi_ssid.isEmpty())
        return;
    WiFi.begin(persist.wifi_ssid.c_str(), persist.wifi_pass.c_str());
}

void wifi_restart()
{
    WiFi.disconnect(true);
    delay(100);
    wifi_start_ap();
    wifi_start_sta();
}

ClientMeta *ensure_meta(AsyncWebSocketClient *c)
{
    if (!c)
        return nullptr;
    auto &meta = client_meta[c->id()]; // default {false}
    return &meta;
}

bool client_authed(AsyncWebSocketClient *c)
{
    ClientMeta *m = ensure_meta(c);
    if (!m)
        return false;
    if (persist.ws_password.isEmpty())
        m->authed = true;
    return m->authed;
}

void free_meta(AsyncWebSocketClient *c)
{
    if (!c)
        return;
    client_meta.erase(c->id());
}

bool ensure_auth(AsyncWebSocketClient *client, const char *type)
{
    if (persist.ws_password.isEmpty())
        return true;
    if (client_authed(client))
        return true;
    if (strcmp(type, "auth") == 0 || strcmp(type, "state") == 0)
        return true;
    // 未认证，要求先 auth
    send_state(client);
    return false;
}

void handle_command(AsyncWebSocketClient *client, JsonDocument &doc)
{
    const char *type = doc["type"] | "";
    if (!ensure_auth(client, type))
        return;

    if (handle_auth_cmd(client, type, doc))
    {
        // 认证成功时标记客户端已认证
        const char *pwd = doc["password"] | "";
        if (persist.ws_password.isEmpty() || persist.ws_password == pwd)
        {
            ClientMeta *m = ensure_meta(client);
            if (m)
                m->authed = true;
        }
        return;
    }
    if (handle_control_cmd(type, doc))
        return;
    if (handle_freq_cmd(type, doc))
        return;
    if (handle_pid_cmd(client, type, doc))
        return;
    if (handle_motion_cmd(type, doc))
        return;
    if (handle_rgb_cmd(type, doc))
        return;
    if (handle_screen_cmd(type, doc))
        return;
    if (handle_wifi_cmd(client, type, doc))
        return;
    if (handle_info_cmd(client, type, doc))
        return;
    // 未识别则忽略
}

void handle_ws_message(AsyncWebSocketClient *client, void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT))
        return;

    StaticJsonDocument<768> doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err)
        return;

    handle_command(client, doc);
}

void ws_event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        ensure_meta(client);
        send_state(client);
        break;
    case WS_EVT_DATA:
        handle_ws_message(client, arg, data, len);
        break;
    case WS_EVT_DISCONNECT:
        free_meta(client);
        break;
    default:
        break;
    }
}

// OTA upload handler
void handle_update_post(AsyncWebServerRequest *request)
{
    bool shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
    request->send(response);
    if (shouldReboot)
    {
        delay(200);
        ESP.restart();
    }
}

void handle_update_upload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        {
            Update.printError(Serial);
        }
    }
    if (!Update.hasError())
    {
        if (Update.write(data, len) != len)
            Update.printError(Serial);
    }
    if (final)
    {
        if (!Update.end(true))
            Update.printError(Serial);
    }
}

} // namespace

void my_net_push_state()
{
    send_state(nullptr);
}

void my_net_init()
{
    net_persist_load(persist, robot.pitch_zero);
    robot.pitch_zero = persist.pitch_zero;

    WiFi.mode(WIFI_AP_STA);
    wifi_start_ap();
    wifi_start_sta();

    ws.onEvent(ws_event);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *resp = request->beginResponseStream("application/json");
        StaticJsonDocument<128> doc;
        doc["fw"] = NET_FW_VERSION;
        doc["name"] = persist.robot_name;
        serializeJson(doc, *resp);
        request->send(resp);
    });

    server.on("/update", HTTP_POST, handle_update_post, handle_update_upload);

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });

    server.begin();
    my_rgb_init();
    net_start_tasks();
}
