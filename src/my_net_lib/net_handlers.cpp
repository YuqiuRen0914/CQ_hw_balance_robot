// 说明：按领域拆分的 WebSocket 指令处理器
#include "net_handlers.h"
#include "net_state.h"
#include "net_persist.h"
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_screen.h"
#include "my_rgb.h"
#include "my_control.h"
#include "my_foc.h"
#include "my_mpu6050.h"

bool handle_auth_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc)
{
    if (strcmp(type, "auth") != 0)
        return false;
    const char *pwd = doc["password"] | "";
    if (persist.ws_password == "" || persist.ws_password == pwd)
    {
        // auth success handled in caller (ensure_meta set there)
        send_auth_status(client, "ok");
        send_state(client);
        send_pid(client);
        send_pitch_zero(client);
        send_torque_limit(client);
        send_deadzone(client);
        send_schema(client);
        send_wifi_config(client);
    }
    else
    {
        send_auth_status(client, "fail");
    }
    return true;
}

bool handle_control_cmd(const char *type, JsonDocument &doc)
{
    if (strcmp(type, "robot_run") == 0)
    {
        robot.run = doc["running"] | false;
        send_state(nullptr);
        return true;
    }
    if (strcmp(type, "fall_check") == 0)
    {
        robot.fallen.enable = doc["enable"] | true;
        return true;
    }
    if (strcmp(type, "suspension_stop") == 0)
    {
        robot.offground_protect = doc["enable"] | true;
        return true;
    }
    if (strcmp(type, "estop") == 0)
    {
        robot.estop = doc["active"] | false;
        return true;
    }
    if (strcmp(type, "system_restart") == 0)
    {
        ESP.restart();
        return true;
    }
    if (strcmp(type, "restart_imu") == 0)
    {
        my_mpu6050_init();
        return true;
    }
    if (strcmp(type, "restart_motor") == 0)
    {
        my_motor_init();
        return true;
    }
    if (strcmp(type, "calib_imu") == 0)
    {
        robot.imu_recalib_req = true;
        return true;
    }
    if (strcmp(type, "calib_deadzone") == 0)
    {
        robot.run = false;
        robot.recalib_req = true;
        return true;
    }
    if (strcmp(type, "restart_wifi") == 0)
    {
        WiFi.disconnect(true);
        delay(100);
        WiFi.softAP(NET_AP_SSID, NET_AP_PASS);
        if (!persist.wifi_ssid.isEmpty())
            WiFi.begin(persist.wifi_ssid.c_str(), persist.wifi_pass.c_str());
        return true;
    }
    return false;
}

bool handle_freq_cmd(const char *type, JsonDocument &doc)
{
    if (strcmp(type, "telem_hz") == 0)
    {
        uint32_t ms = doc["ms"] | telem_ms;
        telem_ms = ms == 0 ? 500 : ms;
        return true;
    }
    if (strcmp(type, "ext_hz") == 0)
    {
        uint32_t ms = doc["ms"] | ext_ms;
        ext_ms = ms == 0 ? 100 : ms;
        return true;
    }
    if (strcmp(type, "attitude_send") == 0)
    {
        attitude_send_on = doc["on"] | true;
        return true;
    }
    if (strcmp(type, "charts_send") == 0)
    {
        charts_send_on = doc["on"] | false;
        return true;
    }
    return false;
}

bool handle_pid_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc)
{
    if (strcmp(type, "get_pid") == 0)
    {
        send_pid(client);
        return true;
    }
    if (strcmp(type, "set_pid") == 0)
    {
        JsonObject p = doc.as<JsonObject>();
        robot.ang_pid.p = p["key01"] | robot.ang_pid.p;
        robot.ang_pid.i = p["key02"] | robot.ang_pid.i;
        robot.ang_pid.d = p["key03"] | robot.ang_pid.d;
        robot.spd_pid.p = p["key04"] | robot.spd_pid.p;
        robot.spd_pid.i = p["key05"] | robot.spd_pid.i;
        robot.spd_pid.d = p["key06"] | robot.spd_pid.d;
        robot.yaw_pid.p = p["key10"] | robot.yaw_pid.p;
        robot.yaw_pid.i = p["key11"] | robot.yaw_pid.i;
        robot.yaw_pid.d = p["key12"] | robot.yaw_pid.d;
        send_pid(client);
        return true;
    }
    if (strcmp(type, "get_pitch_zero") == 0)
    {
        send_pitch_zero(client);
        return true;
    }
    if (strcmp(type, "pitch_zero_set") == 0)
    {
        float v = doc["value"] | robot.pitch_zero;
        robot.pitch_zero = v;
        net_persist_save_pitch_zero(v);
        send_pitch_zero(client);
        return true;
    }
    if (strcmp(type, "get_deadzone") == 0)
    {
        send_deadzone(client);
        return true;
    }
    if (strcmp(type, "set_torque_limit") == 0)
    {
        float v = doc["value"] | torque_limit;
        torque_limit = v;
        return true;
    }
    if (strcmp(type, "get_torque_limit") == 0)
    {
        send_torque_limit(client);
        return true;
    }
    return false;
}

bool handle_motion_cmd(const char *type, JsonDocument &doc)
{
    if (strcmp(type, "joy") == 0)
    {
        robot.joy.x = doc["x"] | robot.joy.x;
        robot.joy.y = doc["y"] | robot.joy.y;
        robot.joy_stop_control = false;
        return true;
    }
    if (strcmp(type, "test_mode") == 0)
    {
        bool enable = doc["enable"] | false;
        robot.test_cmd = enable;
        if (!enable)
            control_reset(robot);
        return true;
    }
    if (strcmp(type, "set_motor_mode") == 0)
    {
        const char *m = doc["mode"] | "pwm";
        if (strcmp(m, "speed") == 0)
            robot.motor_mode = MODE_SPEED;
        else if (strcmp(m, "pos") == 0)
            robot.motor_mode = MODE_POS;
        else
            robot.motor_mode = MODE_PWM;
        return true;
    }
    if (strcmp(type, "set_motor") == 0)
    {
        if (robot.test_cmd)
        {
            robot.tor.L = doc["l"] | 0.0f;
            robot.tor.R = doc["r"] | 0.0f;
        }
        return true;
    }
    if (strcmp(type, "set_servo") == 0)
    {
        StaticJsonDocument<64> resp;
        resp["type"] = "info";
        resp["text"] = "servo unsupported";
        send_json(nullptr, resp);
        return true;
    }
    return false;
}

bool handle_rgb_cmd(const char *type, JsonDocument &doc)
{
    if (strcmp(type, "set_leds") == 0)
    {
        uint8_t brightness = doc["brightness"] | 255;
        JsonArray arr = doc["leds"].as<JsonArray>();
        std::vector<uint8_t> rgb(arr.size());
        for (size_t i = 0; i < arr.size(); ++i)
            rgb[i] = arr[i];
        my_rgb_set(rgb.data(), rgb.size(), brightness);
        return true;
    }
    if (strcmp(type, "set_rgb") == 0)
    {
        int mode = doc["mode"] | 0;
        int count = doc["count"] | RGB_COUNT;
        my_rgb_preset(mode, count);
        return true;
    }
    return false;
}

bool handle_screen_cmd(const char *type, JsonDocument &doc)
{
    if (strcmp(type, "screen_data_v2") != 0)
        return false;
    uint16_t w = doc["width"] | 0;
    uint16_t h = doc["height"] | 0;
    const char *mode = doc["mode"] | "";
    const char *encoding = doc["encoding"] | "";
    if (strcmp(encoding, "base64") != 0)
        return true;
    String data_b64 = doc["data"] | "";
    std::vector<uint8_t> buf;
    if (decode_base64(data_b64, buf))
        my_screen_draw_buffer(buf.data(), buf.size(), w, h, mode);
    return true;
}

bool handle_wifi_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc)
{
    if (strcmp(type, "get_wifi_config") == 0)
    {
        send_wifi_config(client);
        return true;
    }
    if (strcmp(type, "set_wifi_config") == 0)
    {
        String ssid = doc["ssid"] | "";
        String pass = doc["password"] | "";
        if (ssid.length() == 0)
        {
            send_wifi_save_status(client, "fail", "empty ssid");
        }
        else
        {
            net_persist_save_wifi(ssid, pass);
            persist.wifi_ssid = ssid;
            persist.wifi_pass = pass;
            WiFi.disconnect(true);
            delay(100);
            WiFi.softAP(NET_AP_SSID, NET_AP_PASS);
            if (!persist.wifi_ssid.isEmpty())
                WiFi.begin(persist.wifi_ssid.c_str(), persist.wifi_pass.c_str());
            send_wifi_save_status(client, "ok");
        }
        return true;
    }
    return false;
}

bool handle_info_cmd(AsyncWebSocketClient *client, const char *type, JsonDocument &doc)
{
    if (strcmp(type, "get_sys_info") == 0)
    {
        send_sys_info(client);
        return true;
    }
    if (strcmp(type, "set_name") == 0)
    {
        String name = doc["name"] | "";
        net_persist_save_name(name);
        persist.robot_name = name;
        send_sys_info(client);
        return true;
    }
    if (strcmp(type, "set_password") == 0)
    {
        String pwd = doc["password"] | "";
        persist.ws_password = pwd;
        net_persist_save_password(pwd);
        send_auth_status(client, "ok");
        send_state(nullptr);
        return true;
    }
    return false;
}
