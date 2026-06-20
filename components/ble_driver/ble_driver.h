#ifndef BLE_DRIVER_H
#define BLE_DRIVER_H

#include <stdbool.h>

// Các hàm cũ đã có sẵn của bạn
void ble_driver_init(void);
void ble_driver_start_advertising(void);
bool ble_is_app_paired_successfully(void);

// 🔥 THÊM CHÍNH XÁC DÒNG NÀY VÀO ĐÂY:
void ble_driver_stop_advertising(void);
#endif // BLE_DRIVER_H