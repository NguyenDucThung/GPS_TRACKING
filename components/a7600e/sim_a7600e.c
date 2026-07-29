#include "sim_a7600e.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SIM_A7600E";

// Hàm kích nguồn cứng 
void sim_a7600e_power_on(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIM_PWR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 1. Thả nút nguồn ban đầu
    gpio_set_level(SIM_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 2. Nhấn giữ nút nguồn
    ESP_LOGI(TAG, "Bat dau kich nguon: Keo PWRKEY xuong GND (Kich Transistor)...");
    gpio_set_level(SIM_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 3. Buông nút nguồn (Trả về trạng thái nghỉ)
    ESP_LOGI(TAG, "Buong nut nguon: Tha nổi chân PWRKEY an toan...");
    gpio_set_level(SIM_PWR_PIN, 0); 

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Xung kich nguon hoan tat. Cho module on dinh dinh vao he thong.");
}

// Khởi tạo cấu hình UART
void sim_a7600e_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(SIM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SIM_UART_NUM, SIM_TX_PIN, SIM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(SIM_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Khoi tao UART1 (TX=%d, RX=%d) thanh cong.", SIM_TX_PIN, SIM_RX_PIN);
}

// Hàm gửi lệnh AT và kiểm tra phản hồi
esp_err_t sim_send_cmd(const char *cmd, const char *expected_resp, uint32_t timeout_ms)
{
    char data[BUF_SIZE];
    memset(data, 0, BUF_SIZE);

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_NUM, "\r\n", 2);

    int len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)data, BUF_SIZE - 1, pdMS_TO_TICKS(timeout_ms));

    if (len > 0)
    {
        data[len] = '\0';
        ESP_LOGI(TAG, "Sent: %s -> Received:\n%s", cmd, data);
        if (strstr(data, expected_resp) != NULL)
        {
            return ESP_OK;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Sent: %s -> [TIMEOUT] Khong nhan duoc phan hoi trong %d ms", cmd, (int)timeout_ms);
    }
    return ESP_FAIL;
}

// Hàm thực hiện cuộc gọi
esp_err_t sim_make_call(const char *phone_number)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ATD%s;", phone_number);
    return sim_send_cmd(cmd, "OK", 3000);
}

// Hàm dập máy
void sim_hang_up(void)
{
    sim_send_cmd("ATH", "OK", 2000);
}

// Gửi payload JSON lên Firebase qua SIM A7600E
esp_err_t sim_send_to_firebase(const char *json_payload)
{
    char cmd[256];
    char rx_buf[BUF_SIZE];
    int len;

    sim_send_cmd("AT+HTTPTERM", "OK", 500);

    if (sim_send_cmd("AT+HTTPINIT", "OK", 2000) != ESP_OK) return ESP_FAIL;

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", FIREBASE_URL);
    if (sim_send_cmd(cmd, "OK", 2000) != ESP_OK) {
        sim_send_cmd("AT+HTTPTERM", "OK", 1000);
        return ESP_FAIL;
    }

    if (sim_send_cmd("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000) != ESP_OK) {
        sim_send_cmd("AT+HTTPTERM", "OK", 1000);
        return ESP_FAIL;
    }

    int payload_len = strlen(json_payload);
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,5000", payload_len);

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_NUM, "\r\n", 2);

    len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)rx_buf, BUF_SIZE - 1, pdMS_TO_TICKS(2000));
    if (len > 0) {
        rx_buf[len] = '\0';
        if (strstr(rx_buf, "DOWNLOAD") != NULL || strstr(rx_buf, ">") != NULL) {
            uart_write_bytes(SIM_UART_NUM, json_payload, payload_len);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            sim_send_cmd("AT+HTTPTERM", "OK", 1000);
            return ESP_FAIL;
        }
    } else {
        sim_send_cmd("AT+HTTPTERM", "OK", 1000);
        return ESP_FAIL;
    }

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, "AT+HTTPACTION=1\r\n", 17);

    len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)rx_buf, BUF_SIZE - 1, pdMS_TO_TICKS(8000));
    if (len > 0) {
        rx_buf[len] = '\0';
        if (strstr(rx_buf, ",200,") != NULL || strstr(rx_buf, ",201,") != NULL) {
            ESP_LOGI(TAG, "==> FIREBASE: PUSH DATA SUCCESS!");
            sim_send_cmd("AT+HTTPTERM", "OK", 2000);
            return ESP_OK;
        }
    }

    sim_send_cmd("AT+HTTPTERM", "OK", 2000);
    return ESP_FAIL;
}