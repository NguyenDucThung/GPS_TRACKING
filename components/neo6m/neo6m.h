#ifndef NEO6M_H
#define NEO6M_H

#include "esp_err.h"
#include "driver/uart.h"

// Dùng UART_NUM_1 để không đụng hàng với UART_NUM_0 (Log Console)
#define CONFIG_NEO6M_UART_PORT   UART_NUM_0 
#define CONFIG_NEO6M_TX_PIN      7
#define CONFIG_NEO6M_RX_PIN      6
#define CONFIG_NEO6M_BAUDRATE    9600
#define NEO6M_BUF_SIZE           256

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int satellites;
    bool valid;
} neo6m_gps_data_t;

// Khai báo các hàm Driver rút gọn
esp_err_t neo6m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);
esp_err_t neo6m_init_default(void);
bool neo6m_read_gps(neo6m_gps_data_t *out_data);

#endif