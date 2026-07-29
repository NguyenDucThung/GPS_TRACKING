#ifndef RELAY_H_
#define RELAY_H_

#include "driver/gpio.h"


#define RELAY_PIN         GPIO_NUM_10


#define RELAY_ON_LEVEL    1  
#define RELAY_OFF_LEVEL   0  

// --- CÁC HÀM CÔNG KHAI ---
void relay_init(void);
void relay_on(void);
void relay_off(void);
void relay_toggle(void);

#endif /* RELAY_H_ */