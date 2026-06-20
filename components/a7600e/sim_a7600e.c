#include "sim_a7600e.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SIM_A7600E";

// Định nghĩa mảng lưu chuỗi JSON tọa độ mặc định ban đầu
char g_gps_json_payload[256] = "{\"latitude\":0.0,\"longitude\":0.0,\"status\":\"searching\"}";

// Hàm kích nguồn cứng
void sim_a7600e_power_on(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIM_PWR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    gpio_set_level(SIM_PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Bat dau kich nguon: Keo xuong LOW trong 2.5s...");
    gpio_set_level(SIM_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGI(TAG, "Buong ra: Tra ve muc HIGH (Giu nguyen cau hinh OUTPUT)...");
    gpio_set_level(SIM_PWR_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "Module SIM da boot xong voi nguon 5.5V on dinh!");
}

// Khởi tạo ngoại vi UART
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

    ESP_LOGI(TAG, "Khoi tao UART cho SIM A7600E thanh cong.");
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
        ESP_LOGE(TAG, "Sent: %s -> Timeout khoong nhan duoc phan hoi", cmd);
    }
    return ESP_FAIL;
}

// Hàm bật nguồn module GPS
esp_err_t sim_gps_enable(void)
{
    ESP_LOGI(TAG, "Dang bat nguon khoi GPS/GNSS...");
    if (sim_send_cmd("AT+CGNSSPWR=1", "OK", 2000) == ESP_OK)
    {
        ESP_LOGI(TAG, "Da bat nguon GPS thanh cong.");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Khong bat duoc GPS!");
    return ESP_FAIL;
}

// Hàm đọc và tự động bóc tách tọa độ GPS từ NMEA sang JSON
void sim_gps_get_info(void)
{
    char data[BUF_SIZE];
    memset(data, 0, BUF_SIZE);

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, "AT+CGNSSINFO\r\n", 14);

    int len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)data, BUF_SIZE - 1, pdMS_TO_TICKS(1500));

    if (len > 0)
    {
        data[len] = '\0';
        
        // Trường hợp 1: Chưa định vị được vệ tinh (chuỗi chứa nhiều dấu phẩy trống)
        if (strstr(data, ",,,,,,,") != NULL)
        {
            ESP_LOGW(TAG, "GPS dang tim ve tinh... (Hay ra cho thoang dang)");
            snprintf(g_gps_json_payload, sizeof(g_gps_json_payload), 
                     "{\"latitude\":0.0,\"longitude\":0.0,\"status\":\"searching\"}");
        }
        // Trường hợp 2: Đã bắt được sóng vệ tinh thành công
        else if (strstr(data, "+CGNSSINFO:") != NULL)
        {
            char *ptr = strstr(data, "+CGNSSINFO:");
            char lat_raw[20] = {0};
            char lon_raw[20] = {0};
            char n_s = 0, e_w = 0;

            // Bóc tách chuỗi thô bằng sscanf bảo mật độ rộng %15 để tránh tràn bộ đệm
            // Định dạng AT+CGNSSINFO phản hồi: +CGNSSINFO: [Vĩ độ],N/S,[Kinh độ],E/W,...
            int parsed = sscanf(ptr, "+CGNSSINFO: %15[^,],%c,%15[^,],%c", lat_raw, &n_s, lon_raw, &e_w);

            if (parsed == 4 && strlen(lat_raw) > 0 && strlen(lon_raw) > 0)
            {
                double raw_latitude = atof(lat_raw);
                double raw_longitude = atof(lon_raw);

                // --- CHUYỂN ĐỔI ĐỊNH DẠNG NMEA (DDMM.MMMMMM) SANG ĐỘ THẬP PHÂN (DD.DDDDDD) ---
                // Xử lý Vĩ độ (Latitude)
                int lat_degrees = (int)(raw_latitude / 100);
                double lat_minutes = raw_latitude - (lat_degrees * 100);
                double final_latitude = lat_degrees + (lat_minutes / 60.0);
                if (n_s == 'S') final_latitude = -final_latitude;

                // Xử lý Kinh độ (Longitude)
                int lon_degrees = (int)(raw_longitude / 100);
                double lon_minutes = raw_longitude - (lon_degrees * 100);
                double final_longitude = lon_degrees + (lon_minutes / 60.0);
                if (e_w == 'W') final_longitude = -final_longitude;

                // Đóng gói thành chuỗi JSON hoàn chỉnh lưu vào biến toàn cục
                snprintf(g_gps_json_payload, sizeof(g_gps_json_payload),
                         "{\"latitude\":%.6f,\"longitude\":%.6f,\"status\":\"fixed\"}",
                         final_latitude, final_longitude);

                ESP_LOGI(TAG, "==> Gps Parse JSON Success: %s", g_gps_json_payload);
            }
            else
            {
                ESP_LOGE(TAG, "Loi dinh dang chuoi CGNSSINFO, khong sscanf duoc");
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "Loi timeout khi doc toa do GPS");
    }
}

// Hàm thực hiện cuộc gọi
esp_err_t sim_make_call(const char *phone_number)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ATD%s;", phone_number);
    ESP_LOGI(TAG, "Dang goi dien den so: %s...", phone_number);

    if (sim_send_cmd(cmd, "OK", 3000) == ESP_OK)
    {
        ESP_LOGI(TAG, "Cuoc goi da duoc khoi tao thanh cong!");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Khoi tao cuoc goi that bai!");
    return ESP_FAIL;
}

// Hàm dập máy
void sim_hang_up(void)
{
    ESP_LOGI(TAG, "Dang tien hanh dap may...");
    sim_send_cmd("ATH", "OK", 2000);
}

// Hàm truyền dữ liệu JSON lên Firebase
esp_err_t sim_send_to_firebase(const char *json_payload)
{
    char cmd[256];
    char rx_buf[BUF_SIZE];
    int len;

    ESP_LOGI(TAG, "=== Bat dau gui len Firebase ===");

    sim_send_cmd("AT+HTTPTERM", "OK", 500);

    if (sim_send_cmd("AT+HTTPINIT", "OK", 2000) != ESP_OK)
    {
        ESP_LOGE(TAG, "Loi khoi tao HTTP");
        return ESP_FAIL;
    }

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", FIREBASE_URL);
    sim_send_cmd(cmd, "OK", 2000);
    sim_send_cmd("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000);

    int payload_len = strlen(json_payload);
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,5000", payload_len);

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_NUM, "\r\n", 2);

    len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)rx_buf, BUF_SIZE - 1, pdMS_TO_TICKS(2000));
    if (len > 0)
    {
        rx_buf[len] = '\0';
        if (strstr(rx_buf, "DOWNLOAD") != NULL || strstr(rx_buf, ">") != NULL)
        {
            uart_write_bytes(SIM_UART_NUM, json_payload, payload_len);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    else
    {
        ESP_LOGE(TAG, "SIM khong phan hoi lenh HTTPDATA");
        sim_send_cmd("AT+HTTPTERM", "OK", 1000);
        return ESP_FAIL;
    }

    uart_flush_input(SIM_UART_NUM);
    uart_write_bytes(SIM_UART_NUM, "AT+HTTPACTION=1\r\n", 17);

    len = uart_read_bytes(SIM_UART_NUM, (uint8_t *)rx_buf, BUF_SIZE - 1, pdMS_TO_TICKS(6000));
    if (len > 0)
    {
        rx_buf[len] = '\0';
        ESP_LOGI(TAG, "Ket qua tu mang: %s", rx_buf);
        if (strstr(rx_buf, ",200,") != NULL || strstr(rx_buf, ",201,") != NULL)
        {
            ESP_LOGI(TAG, "==> FIREBASE: GUI DATA OK!");
        }
        else
        {
            ESP_LOGE(TAG, "==> FIREBASE: Loi ma HTTP (Khong phai 200/201)");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Timeout HTTPACTION");
    }

    sim_send_cmd("AT+HTTPTERM", "OK", 2000);
    return ESP_OK;
}

// Hàm quản lý chế độ máy bay
esp_err_t sim_set_airplane_mode(bool enable) {
    // Luôn dọn sạch bộ đệm UART trước khi ra lệnh
    uart_flush_input(SIM_UART_NUM);

    if (enable) {
        ESP_LOGI(TAG, "==> Chuyen SIM sang che do may bay (Tiet kiem pin)...");
        return sim_send_cmd("AT+CFUN=4", "OK", 3000);
    } else {
        ESP_LOGI(TAG, "==> Tat che do may bay, dang bat lai song di dong...");
        
        // BỔ SUNG CHIẾN THUẬT: Bơm 3 lần lệnh AT rỗng để xóa ký tự nhiễu do sụt áp chân TX khi ngủ
        for (int i = 0; i < 3; i++) {
            uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        // Xóa sạch đống phản hồi thừa thu được từ 3 lệnh AT rỗng phía trên
        uart_flush_input(SIM_UART_NUM); 

        // Tiến hành bật lại sóng mạng an toàn
        esp_err_t ret = sim_send_cmd("AT+CFUN=1", "OK", 3000);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(3000)); 
        }
        return ret;
    }
}
// Triển khai hàm trả về con trỏ chuỗi JSON
char *sim_get_gps_location(void) {
    return g_gps_json_payload;
}