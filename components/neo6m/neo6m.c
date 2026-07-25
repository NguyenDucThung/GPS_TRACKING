#include "neo6m.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Macro chân GPIO (Lấy từ header hoặc định nghĩa mặc định)
#ifndef CONFIG_NEO6M_PWR_PIN
#define CONFIG_NEO6M_PWR_PIN GPIO_NUM_18
#endif

#ifndef CONFIG_NEO6M_TX_PIN
#define CONFIG_NEO6M_TX_PIN GPIO_NUM_7
#endif

#ifndef CONFIG_NEO6M_RX_PIN
#define CONFIG_NEO6M_RX_PIN GPIO_NUM_6
#endif

static const char *TAG = "NEO6M_DRIVER";
static uart_port_t g_uart_num = CONFIG_NEO6M_UART_PORT;

static char line_buffer[128];
static int line_idx = 0;
static bool s_is_powered = false; // Trạng thái hoạt động của GPS

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

/* =========================================================================
   1. KHỞI TẠO UART
   ========================================================================= */

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
        ESP_LOGI(TAG, "Init NEO-6M UART%d (TX:%d, RX:%d, Baud:%d)", g_uart_num, tx_pin, rx_pin, baud_rate);
    }
    return err;
}

esp_err_t neo6m_init_default(void)
{
    return neo6m_init(CONFIG_NEO6M_UART_PORT, CONFIG_NEO6M_TX_PIN, CONFIG_NEO6M_RX_PIN, CONFIG_NEO6M_BAUDRATE);
}

/* =========================================================================
   2. QUẢN LÝ BẬT / TẮT BẰNG TRANSISTOR C1815 (GND SWITCH)
   ========================================================================= */

// Khởi tạo chân GPIO18 (Mặc định LOW = TẮT C1815)
esp_err_t neo6m_power_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_NEO6M_PWR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Pull-down để C1815 tắt mặc định
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err == ESP_OK)
    {
        gpio_set_level(CONFIG_NEO6M_PWR_PIN, 0); // Ngắt GND GPS lúc mới lên nguồn
        s_is_powered = false;
        ESP_LOGI(TAG, "GPIO %d (C1815 Transistor) initialized: OFF by default.", CONFIG_NEO6M_PWR_PIN);
    }
    return err;
}

// Hàm Bật / Tắt GPS chính
esp_err_t neo6m_set_power(bool enable)
{
    if (enable && !s_is_powered)
    {
        // Step 1: Kéo GPIO18 lên HIGH -> Bật C1815 cấp GND cho GPS
        gpio_set_level(CONFIG_NEO6M_PWR_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100)); // Trễ 100ms chờ điện áp ổn định

        // Step 2: Khởi tạo lại Driver UART
        esp_err_t err = neo6m_init_default();
        if (err == ESP_OK)
        {
            s_is_powered = true;
            ESP_LOGI(TAG, "NEO-6M: POWER ON (C1815 Active - GND Connected)");
        }
        return err;
    }
    else if (!enable && s_is_powered)
    {
        // Step 1: Hủy Driver UART0
        uart_driver_delete(g_uart_num);

        // Step 2: Triệt tiêu dòng rò (Phantom Power) qua chân TX/RX
        gpio_reset_pin(CONFIG_NEO6M_TX_PIN);
        gpio_reset_pin(CONFIG_NEO6M_RX_PIN);
        gpio_set_direction(CONFIG_NEO6M_TX_PIN, GPIO_MODE_INPUT);
        gpio_set_direction(CONFIG_NEO6M_RX_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode(CONFIG_NEO6M_TX_PIN, GPIO_FLOATING);
        gpio_set_pull_mode(CONFIG_NEO6M_RX_PIN, GPIO_FLOATING);

        // Step 3: Kéo GPIO18 xuống LOW -> Ngắt C1815 dập nguồn GPS
        gpio_set_level(CONFIG_NEO6M_PWR_PIN, 0);
        s_is_powered = false;
        line_idx = 0; // Reset buffer

        ESP_LOGI(TAG, "NEO-6M: POWER OFF (C1815 Cut Off + Anti-Leakage Active)");
        return ESP_OK;
    }
    return ESP_OK;
}

bool neo6m_is_powered(void)
{
    return s_is_powered;
}

/* =========================================================================
   3. HÀM ĐỌC TỌA ĐỘ GPS (CÓ FLUSH BUFFER VÀ TIMEOUT)
   ========================================================================= */

bool neo6m_get_latest_fix(neo6m_gps_data_t *out_data, uint32_t timeout_ms)
{
    if (!s_is_powered)
    {
        return false;
    }

    // Xóa sạch đệm UART cũ tích tụ
    uart_flush_input(g_uart_num);
    line_idx = 0;

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    // Lặp đọc UART đến khi gặp dòng $GPGGA/$GNGGA hợp lệ hoặc hết timeout
    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        uint8_t ch;
        if (uart_read_bytes(g_uart_num, &ch, 1, pdMS_TO_TICKS(20)) > 0)
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

bool neo6m_read_gps(neo6m_gps_data_t *out_data)
{
    return neo6m_get_latest_fix(out_data, 1000);
}