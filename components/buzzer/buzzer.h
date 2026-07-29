#ifndef BUZZER_H_
#define BUZZER_H_

#include "driver/gpio.h"


#define BUZZER_PIN    GPIO_NUM_1


void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint32_t duration_ms);

#endif /* BUZZER_H_ */

