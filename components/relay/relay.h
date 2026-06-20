#ifndef RELAY_H_
#define RELAY_H_

#include "driver/gpio.h"

// Định nghĩa chân điều khiển Relay
#define RELAY_PIN         GPIO_NUM_10

// --- CẤU HÌNH LOGIC KÍCH (Thay đổi ở đây nếu bị ngược trạng thái) ---
#define RELAY_ON_LEVEL    1  // 1 nếu là Relay Active-High, 0 nếu là Active-Low
#define RELAY_OFF_LEVEL   0  // 0 nếu là Relay Active-High, 1 nếu là Active-Low

// --- CÁC HÀM CÔNG KHAI ---
void relay_init(void);
void relay_on(void);
void relay_off(void);
void relay_toggle(void);

#endif /* RELAY_H_ */