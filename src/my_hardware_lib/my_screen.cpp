#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "my_screen.h"
#include "my_motion.h"
#include "my_motion_state.h"
#include "my_config.h"
#include "my_bat.h"
#include "my_i2c.h"
#include "my_tool.h"

namespace
{
constexpr uint32_t FRAME_INTERVAL_MS = SCREEN_REFRESH_TIME;
constexpr uint8_t HEADER_H = 12;

// 公司 Logo 位图数据 (55x55 像素)
constexpr uint8_t COMPANY_LOGO_WIDTH = 55;
constexpr uint8_t COMPANY_LOGO_HEIGHT = 55;
static const uint8_t PROGMEM company_logo_bitmap[] = {
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0x81, 0xff, 0xff, 0xfe, 0xff, 0xff, 
0xf8, 0x61, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xc7, 0x87, 0xff, 0xff, 0xfe, 0xff, 0xff, 0x3e, 0x00, 
0x7f, 0xff, 0xfe, 0xff, 0xfc, 0xfc, 0x00, 0x0f, 0xff, 0xfe, 0xff, 0xf1, 0xf8, 0x00, 0x03, 0xfc, 
0x7e, 0xff, 0xe7, 0xf0, 0x00, 0x00, 0xe3, 0x7e, 0xff, 0xcf, 0xe0, 0x00, 0x40, 0x4f, 0x7e, 0xff, 
0x9f, 0xc0, 0x00, 0x40, 0x3f, 0x3e, 0xff, 0x3f, 0x80, 0x00, 0x00, 0xff, 0x3e, 0xfe, 0x7f, 0x80, 
0x60, 0x01, 0xff, 0x3e, 0xfc, 0xff, 0x00, 0x60, 0x03, 0xff, 0x7e, 0xfd, 0xff, 0x01, 0xf8, 0x03, 
0x8f, 0x7e, 0xf9, 0xee, 0x00, 0x60, 0x07, 0x07, 0x7e, 0xfb, 0xfe, 0x00, 0x60, 0x0f, 0x07, 0x7e, 
0xf3, 0xfe, 0x00, 0x00, 0x0f, 0x8e, 0x7e, 0xf7, 0xfc, 0x00, 0x00, 0x1f, 0xfe, 0x7e, 0xe7, 0xfc, 
0x00, 0x00, 0xdf, 0xfc, 0x3e, 0xef, 0xfc, 0x00, 0x03, 0xbf, 0xfc, 0x3e, 0xef, 0xfc, 0x08, 0x03, 
0xbf, 0xf8, 0x3e, 0xcf, 0xfc, 0x1c, 0x07, 0x7f, 0xf0, 0x1e, 0xce, 0x7c, 0x08, 0x0f, 0x7f, 0xe0, 
0x1e, 0xcc, 0xfc, 0x00, 0x00, 0xff, 0xc0, 0x1e, 0xde, 0x7c, 0x00, 0x00, 0xff, 0x80, 0x1e, 0xdf, 
0xfc, 0x00, 0x00, 0x3f, 0x20, 0x1e, 0xdf, 0xfc, 0x00, 0x00, 0x1e, 0x60, 0x1e, 0xdf, 0xfc, 0x00, 
0x01, 0x84, 0xe0, 0x1e, 0xdf, 0xfe, 0x03, 0x03, 0xe0, 0xe0, 0x1e, 0xdf, 0xfe, 0x02, 0x07, 0xe0, 
0xc0, 0x1e, 0xdf, 0xfe, 0x00, 0x0f, 0xe0, 0x80, 0x1e, 0xcf, 0xff, 0x00, 0x0f, 0xc0, 0x04, 0x12, 
0xcf, 0xff, 0x00, 0x1f, 0x80, 0x04, 0x16, 0xcf, 0xcf, 0x80, 0x3f, 0x00, 0x00, 0x26, 0xef, 0xb7, 
0xc0, 0x7f, 0x00, 0x00, 0x06, 0xe7, 0xb7, 0xc1, 0xfe, 0x00, 0x00, 0x06, 0xe7, 0xbf, 0xff, 0xfc, 
0x00, 0x80, 0x2e, 0xf3, 0xc7, 0xff, 0xfc, 0x00, 0x00, 0x6e, 0xf3, 0xff, 0xff, 0xfc, 0x00, 0x00, 
0xde, 0xf9, 0xff, 0xff, 0xfc, 0x00, 0x01, 0x9e, 0xf9, 0xff, 0xff, 0xfc, 0x00, 0x07, 0xbe, 0xfc, 
0xff, 0xff, 0xfe, 0x00, 0x1f, 0x3e, 0xfe, 0x7f, 0xff, 0xff, 0x00, 0xfe, 0x7e, 0xfe, 0x7f, 0x9f, 
0xff, 0xff, 0xfe, 0xfe, 0xff, 0x3f, 0x5f, 0xff, 0xff, 0xfd, 0xfe, 0xff, 0x9f, 0x9f, 0xff, 0xff, 
0xf9, 0xfe, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xf3, 0xfe, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xc7, 0xfe, 
0xff, 0xf9, 0xfe, 0xff, 0xff, 0x9f, 0xfe, 0xff, 0xfc, 0x7f, 0xff, 0xfe, 0x3f, 0xfe, 0xff, 0xff, 
0x1f, 0xff, 0xf8, 0xff, 0xfe, 0xff, 0xff, 0xc3, 0xff, 0xc3, 0xff, 0xfe, 0xff, 0xff, 0xf8, 0x00, 
0x1f, 0xff, 0xfe, 0xff, 0xff, 0xff, 0x87, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xfe
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);
bool screen_ready = false;
uint32_t last_frame = 0;

// 五次缓入缓出
static float ease_in_out_quint(float t)
{
    if (t < 0.5f)
        return 16.0f * t * t * t * t * t;
    const float inv = (2.0f * t) - 2.0f;
    return 0.5f * inv * inv * inv * inv * inv + 1.0f;
}

const char *state_text(MotionState s)
{
    switch (s)
    {
    case MotionState::Init: return "INIT";
    case MotionState::Calibrating: return "CAL";
    case MotionState::Idle: return "IDLE";
    case MotionState::Normal: return "RUN";
    case MotionState::OffGround: return "AIR";
    case MotionState::Test: return "TEST";
    case MotionState::LowBat: return "LOW";
    case MotionState::Fallen: return "FALL";
    case MotionState::EStop: return "STOP";
    case MotionState::Fault: return "FAULT";
    case MotionState::Shutdown: return "SHDN";
    default: return "UNK";
    }
}

void draw_header(uint8_t phase)
{
    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.drawFastHLine(0, HEADER_H, SCREEN_WIDTH, SSD1306_WHITE);
    // 简单扫描动画
    uint8_t sweep = (phase * 3) % (SCREEN_WIDTH - 20);
    display.drawFastHLine(4 + sweep, HEADER_H - 1, 16, SSD1306_WHITE);
}

void draw_status(uint8_t phase)
{
    (void)phase;
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // 左侧：状态与电池
    display.setCursor(4, 2);
    display.print(state_text(robot.state));

    display.setCursor(50, 2);
    display.print(robot.lowbat_warn ? "BAT! " : "BAT ");
    display.print(battery_voltage, 1);

    // 中部：角度与扭矩
    display.setCursor(4, HEADER_H + 3);
    display.print("P:");
    display.print(robot.ang.now, 1);
    display.setCursor(64, HEADER_H + 3);
    display.print("T:");
    display.print(robot.tor.base, 1);

    // 底部：离地/摔倒提示
    uint8_t y = HEADER_H + 15;
    if (robot.drv_fault)
    {
        display.setCursor(4, y);
        display.print("I2C FAULT");
    }
    else if (robot.wel_up)
    {
        display.setCursor(4, y);
        display.print("OFF-GROUND");
    }
    else if (robot.fallen.is)
    {
        display.setCursor(4, y);
        display.print("FALLEN");
    }
    else
    {
        display.setCursor(4, y);
        display.print("READY");
    }
}

// 中心 Logo：使用内置位图 company_logo_bitmap 居中绘制
void draw_center_logo(uint8_t frame, float alpha)
{
    (void)frame;
    const int16_t cx = SCREEN_WIDTH / 2;
    const int16_t cy = SCREEN_HEIGHT / 2;

    // 背景光晕
    if (alpha > 0.2f)
    {
        const uint8_t r = static_cast<uint8_t>(10 * alpha);
        display.drawCircle(cx, cy, r, SSD1306_WHITE);
        if (r > 6)
            display.drawCircle(cx, cy, r - 3, SSD1306_WHITE);
    }

    // 位图居中
    const int16_t logo_x = cx - COMPANY_LOGO_WIDTH / 2;
    const int16_t logo_y = cy - COMPANY_LOGO_HEIGHT / 2;
    display.drawBitmap(logo_x, logo_y, company_logo_bitmap, COMPANY_LOGO_WIDTH, COMPANY_LOGO_HEIGHT, SSD1306_WHITE);
}

// 细线网格与斜向扫描线，增加层次感
void draw_grid_and_scan(uint8_t frame)
{
    // 轻量网格
    for (int x = 0; x < SCREEN_WIDTH; x += 16)
        display.drawFastVLine(x, 0, SCREEN_HEIGHT, SSD1306_WHITE);
    for (int y = 0; y < SCREEN_HEIGHT; y += 8)
        display.drawFastHLine(0, y, SCREEN_WIDTH, SSD1306_WHITE);

    // 斜向扫描
    for (int x = -SCREEN_HEIGHT; x < SCREEN_WIDTH; x += 10)
    {
        int x0 = x + (frame % 10);
        int y0 = 0;
        int x1 = x0 + SCREEN_HEIGHT;
        int y1 = SCREEN_HEIGHT - 1;
        display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
    }
}

// 粒子闪烁
void draw_particles(uint8_t frame)
{
    for (uint8_t i = 0; i < 10; ++i)
    {
        int16_t x = (frame * (i + 2) + i * 7) % SCREEN_WIDTH;
        int16_t y = (frame * (i + 3) + i * 11) % SCREEN_HEIGHT;
        if ((frame + i) % 3 == 0)
            display.drawPixel(x, y, SSD1306_WHITE);
    }
}

// 更丰富的启动动画（约 2.8s），Logo 居中
void play_boot()
{
    constexpr uint8_t FRAME_COUNT = 80;
    constexpr uint8_t FRAME_DELAY = 35; // 80*35≈2800ms
    for (uint8_t f = 0; f < FRAME_COUNT; ++f)
    {
        const float t = f / float(FRAME_COUNT - 1);
        const float alpha = ease_in_out_quint(t); // Logo 缓入

        display.clearDisplay();

        // 背景网格 + 斜向扫描
        if (t > 0.1f)
            draw_grid_and_scan(f);

        // 流动下划线
        draw_header(f);

        // 粒子层
        if (t > 0.2f)
            draw_particles(f);

        // 中心 Logo
        draw_center_logo(f, alpha);

        display.display();
        delay(FRAME_DELAY);
    }
}

} // namespace

void my_screen_init()
{
    screen_ready = false;
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDRESS))
    {
        return;
    }
    display.clearDisplay();
    play_boot();
    screen_ready = true;
    last_frame = millis();
}

void my_screen_update()
{
    if (!screen_ready)
        return;
    const uint32_t now = millis();
    if (now - last_frame < FRAME_INTERVAL_MS)
        return;
    last_frame = now;

    static uint8_t phase = 0;
    phase++;

    display.clearDisplay();
    draw_header(phase);
    draw_status(phase);
    display.display();
}
