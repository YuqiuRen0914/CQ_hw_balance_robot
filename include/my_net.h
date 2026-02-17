#pragma once

void my_net_init();

// 主动推送一次 state/telemetry（用于重要状态变化后即刻同步）
void my_net_push_state();

