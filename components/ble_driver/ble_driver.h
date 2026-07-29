#ifndef BLE_DRIVER_H
#define BLE_DRIVER_H

#include <stdbool.h>

// Các hàm 
void ble_driver_init(void);
void ble_driver_start_advertising(void);
bool ble_is_app_paired_successfully(void);


void ble_driver_stop_advertising(void);
#endif // BLE_DRIVER_H