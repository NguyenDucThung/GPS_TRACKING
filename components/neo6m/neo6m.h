#ifndef NEO6M_H
#define NEO6M_H

#include "esp_err.h"
#include "driver/uart.h"

#define CONFIG_NEO6M_UART_PORT   UART_NUM_0
#define CONFIG_NEO6M_TX_PIN      7
#define CONFIG_NEO6M_RX_PIN      6
#define CONFIG_NEO6M_BAUDRATE    9600
#define CONFIG_NEO6M_PWR_PIN     GPIO_NUM_18  // Chân kích Transistor C1815
#define NEO6M_BUF_SIZE           256

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int satellites;
    bool valid;
} neo6m_gps_data_t;

// Khai báo các hàm Driver
esp_err_t neo6m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);
esp_err_t neo6m_init_default(void);

// Các hàm Bật/Tắt nguồn bổ sung mới
esp_err_t neo6m_power_init(void);
esp_err_t neo6m_set_power(bool enable);
bool neo6m_is_powered(void);

bool neo6m_read_gps(neo6m_gps_data_t *out_data);

#endif