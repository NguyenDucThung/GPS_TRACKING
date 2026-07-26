#include "neo6m.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "NEO6M_DRIVER";
static uart_port_t g_uart_num = CONFIG_NEO6M_UART_PORT;
static char line_buffer[128];

static float nmea_to_decimal(const char *raw, const char *dir)
{
    if (!raw || strlen(raw) == 0)
        return 0.0f;

    float raw_val = atof(raw);
    int degrees = (int)(raw_val / 100);
    float minutes = raw_val - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);

    if (dir && (dir[0] == 'S' || dir[0] == 'W'))
    {
        decimal = -decimal;
    }
    return decimal;
}

static bool parse_gga_line(char *line, neo6m_gps_data_t *out_data)
{
    if (strstr(line, "$GPGGA") == NULL && strstr(line, "$GNGGA") == NULL)
    {
        return false;
    }

    char *tokens[15];
    int token_count = 0;
    char *ptr = line;

    while (ptr && token_count < 15)
    {
        tokens[token_count++] = ptr;
        char *comma = strchr(ptr, ',');
        if (comma)
        {
            *comma = '\0';
            ptr = comma + 1;
        }
        else
        {
            break;
        }
    }

    if (token_count < 10)
        return false;

    int fix_quality = atoi(tokens[6]);
    out_data->satellites = atoi(tokens[7]);

    if (fix_quality == 0)
    {
        out_data->valid = false;
        return false;
    }

    out_data->latitude = nmea_to_decimal(tokens[2], tokens[3]);
    out_data->longitude = nmea_to_decimal(tokens[4], tokens[5]);
    out_data->altitude = atof(tokens[9]);
    out_data->valid = true;

    return true;
}

esp_err_t neo6m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate)
{
    g_uart_num = uart_num;

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(g_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    esp_err_t err = uart_driver_install(g_uart_num, NEO6M_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Init NEO-6M UART%d (TX:%d, RX:%d, Baud:%d) -> READY 24/24", g_uart_num, tx_pin, rx_pin, baud_rate);
    }
    return err;
}

esp_err_t neo6m_init_default(void)
{
    return neo6m_init(CONFIG_NEO6M_UART_PORT, CONFIG_NEO6M_TX_PIN, CONFIG_NEO6M_RX_PIN, CONFIG_NEO6M_BAUDRATE);
}

// Hàm đọc tọa độ GPS trực tiếp từ UART
bool neo6m_read_gps(neo6m_gps_data_t *out_data)
{
    int line_idx = 0;
    TickType_t start_tick = xTaskGetTickCount();
    
    // Quét UART trong tối đa 1 giây để bốc dòng GPGGA
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(1000))
    {
        uint8_t ch;
        if (uart_read_bytes(g_uart_num, &ch, 1, pdMS_TO_TICKS(10)) > 0)
        {
            if (ch == '\n' || ch == '\r')
            {
                if (line_idx > 0)
                {
                    line_buffer[line_idx] = '\0';
                    if (parse_gga_line(line_buffer, out_data))
                    {
                        if (out_data->valid)
                        {
                            return true;
                        }
                    }
                    line_idx = 0;
                }
            }
            else
            {
                if (line_idx < sizeof(line_buffer) - 1)
                {
                    line_buffer[line_idx++] = (char)ch;
                }
                else
                {
                    line_idx = 0;
                }
            }
        }
    }
    return false;
}