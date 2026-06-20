#ifndef BUZZER_H_
#define BUZZER_H_

#include "driver/gpio.h"

// Định nghĩa chân số 7 để điều khiển còi
#define BUZZER_PIN    GPIO_NUM_7

// --- CÁC HÀM CÔNG KHAI ---
void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint32_t duration_ms);

#endif /* BUZZER_H_ */

