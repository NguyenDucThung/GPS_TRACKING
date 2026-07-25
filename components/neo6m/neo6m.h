#ifndef NEO6M_H
#define NEO6M_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

// ==========================================
// CẤU HÌNH PHẦN CỨNG MẶC ĐỊNH CHO ESP32-C3
// ==========================================
#define CONFIG_NEO6M_UART_PORT   UART_NUM_0
#define CONFIG_NEO6M_TX_PIN      7     // ESP32-C3 TX (GPIO 7) -> RX NEO-6M
#define CONFIG_NEO6M_RX_PIN      6     // ESP32-C3 RX (GPIO 6) <- TX NEO-6M
#define CONFIG_NEO6M_BAUDRATE    9600

#define NEO6M_BUF_SIZE           1024

/**
 * @brief Cấu trúc lưu trữ dữ liệu GPS đã giải mã
 */
typedef struct {
    float latitude;      // Vĩ độ (Decimal Degrees)
    float longitude;     // Kinh độ (Decimal Degrees)
    float altitude;      // Độ cao (m)
    uint8_t satellites;  // Số vệ tinh kết nối
    bool valid;          // true nếu đã Fix GPS, false nếu chưa
} neo6m_gps_data_t;

/**
 * @brief Khởi tạo UART với thông số cấu hình mặc định (Khai báo ở header)
 * @return esp_err_t 
 */
esp_err_t neo6m_init_default(void);

/**
 * @brief Khởi tạo UART tùy chỉnh tham số (Nếu cần)
 */
esp_err_t neo6m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);

/**
 * @brief Đọc bộ đệm UART và parse dữ liệu NMEA
 */
bool neo6m_read_gps(neo6m_gps_data_t *out_data);

#endif // NEO6M_H