#ifndef SIM_A7600E_H_
#define SIM_A7600E_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define SIM_UART_NUM       UART_NUM_1
#define SIM_TX_PIN         GPIO_NUM_4  // ESP TX -> SIM RX
#define SIM_RX_PIN         GPIO_NUM_5  // ESP RX -> SIM TX
#define SIM_PWR_PIN        GPIO_NUM_3  // Chân điều khiển nguồn PWRKEY
#define BUF_SIZE           1024

// Link Firebase Realtime Database của bạn
#define FIREBASE_URL       "https://gps-tracking-a01d3-default-rtdb.asia-southeast1.firebasedatabase.app/vehicle.json?x-http-method-override=PATCH"

// --- BIẾN TOÀN CỤC CHỨA CHUỖI JSON GPS (Xuất khẩu sang main.c dùng) ---
extern char g_gps_json_payload[256];

// --- NGUYÊN MẪU CÁC HÀM (PROTOTYPES) ---
void sim_a7600e_power_on(void);
void sim_a7600e_init(void);
esp_err_t sim_send_cmd(const char *cmd, const char *expected_resp, uint32_t timeout_ms);
esp_err_t sim_gps_enable(void);
void sim_gps_get_info(void);
esp_err_t sim_make_call(const char *phone_number);
void sim_hang_up(void);
esp_err_t sim_send_to_firebase(const char *json_payload);
esp_err_t sim_set_airplane_mode(bool enable);

/**
 * @brief Trả về con trỏ trỏ tới chuỗi JSON chứa tọa độ GPS mới nhất
 * @return char* Chuỗi JSON dạng {"latitude":xx.xxxx,"longitude":xx.xxxx,"status":"..."}
 */
char *sim_get_gps_location(void);

#endif /* SIM_A7600E_H_ */