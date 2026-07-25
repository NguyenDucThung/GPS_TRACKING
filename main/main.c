#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "nvs_flash.h"

// CẤU HÌNH SỐ ĐIỆN THOẠI
#define OWNER_PHONE_NUMBER "0854383970"
#define SIM_SLEEP_PIN 3 // Chân điều khiển PWR

// Include các driver độc lập từ thư mục components
#include "mpu6050.h"
#include "sim_a7600e.h"
#include "neo6m.h"
#include "ble_driver.h"
#include "buzzer.h"
#include "relay.h"

static const char *TAG = "MAIN_SYSTEM";

typedef enum
{
    STATE_SLEEPING,
    STATE_VERIFYING,
    STATE_OWNER_CONNECTED,
    STATE_REMOTE_FINDING,
    STATE_ALARM
} system_state_t;

// Biến trạng thái toàn cục
static system_state_t g_system_state = STATE_SLEEPING;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xWakeSemaphore;
SemaphoreHandle_t xAuthSemaphore;
SemaphoreHandle_t xCallSemaphore;
SemaphoreHandle_t xSimMutex;

// CỜ LỆNH CHO TASK 5
SemaphoreHandle_t xRemoteWakeSemaphore;
SemaphoreHandle_t xSleepAgainSemaphore;
SemaphoreHandle_t xFirebaseDoneSemaphore; // Cờ báo Task 3 đã push Firebase xong

// Khai báo các Task
void mpu_monitor_task(void *pvParameters);
void central_control_task(void *pvParameters);
void sim_gps_network_task(void *pvParameters);
void ble_auth_task(void *pvParameters);
void remote_find_task(void *pvParameters);

// HÀM TIẾP NHẬN TÍN HIỆU XÁC THỰC THÀNH CÔNG TỪ BLE
void main_system_auth_success(void)
{
    if (xAuthSemaphore != NULL)
    {
        xSemaphoreGive(xAuthSemaphore);
        ESP_LOGI(TAG, "Đã tiếp nhận tín hiệu từ BLE! Mở khóa xe cấp tốc...");
    }
}

// HÀM ỔN ĐỊNH UART VÀ LÀM SẠCH RÁC CHO SIM SAU KHI THỨC GIẤC
void sim_stabilize_uart(void)
{
    uart_flush_input(SIM_UART_NUM);

    for (int i = 0; i < 3; i++)
    {
        uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    uart_flush_input(SIM_UART_NUM);
    ESP_LOGI("SIM_UTIL", "Đường truyền UART với SIM đã sạch sẽ và sẵn sàng!");
}

// CHU TRÌNH CẤP NGUỒN
void power_on_sequence(void)
{
    ESP_LOGI(TAG, "===== KHỞI ĐỘNG CHU TRÌNH KÍCH NGUỒN MODULE SIM =====");

    // 1. Đưa về trạng thái tĩnh ban đầu
    gpio_set_level(SIM_SLEEP_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 2. Kéo lên HIGH trong 3 giây
    ESP_LOGI(TAG, "-> Chân PWR (GPIO%d) lên HIGH (Giữ 3s)...", SIM_SLEEP_PIN);
    gpio_set_level(SIM_SLEEP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 3. Thả về LOW và đợi 5 giây cho module bung sóng ổn định
    gpio_set_level(SIM_SLEEP_PIN, 0);
    ESP_LOGI(TAG, "-> Đã thả chân PWR về LOW. Chờ SIM bám mạng ổn định trong 5s...");
    vTaskDelay(pdMS_TO_TICKS(5000));
}

void app_main(void)
{

    // 1. BẮT BỘC: Khởi tạo NVS Flash cho Bluetooth & PHY Calibration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Khởi tạo cơ chế RTOS
    xStateMutex = xSemaphoreCreateMutex();
    xSimMutex = xSemaphoreCreateMutex();

    xWakeSemaphore = xSemaphoreCreateBinary();
    xAuthSemaphore = xSemaphoreCreateBinary();
    xCallSemaphore = xSemaphoreCreateBinary();
    xRemoteWakeSemaphore = xSemaphoreCreateBinary();
    xSleepAgainSemaphore = xSemaphoreCreateBinary();
    xFirebaseDoneSemaphore = xSemaphoreCreateBinary();

    // 2. Khởi tạo cấu hình GPIO ngoại vi
    buzzer_init();
    relay_init();

    // Cấu hình chân ngắt từ cảm biến nghiêng MPU6050
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL};
    gpio_config(&io_conf);
    gpio_wakeup_enable(WAKEUP_GPIO_PIN, GPIO_INTR_HIGH_LEVEL);

    // Cấu hình chân nguồn GPIO 3 làm Output
    gpio_config_t sleep_pin_conf = {
        .pin_bit_mask = (1ULL << SIM_SLEEP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&sleep_pin_conf);

    // Bật tính năng thức giấc bằng GPIO cho MPU6050
    esp_sleep_enable_gpio_wakeup();

    // 3. THỰC THI CHU TRÌNH CẤP NGUỒN VÀ KHỞI ĐỘNG DRIVER PHẦN CỨNG
    power_on_sequence();
    mpu6050_init();
    ble_driver_init();
    sim_a7600e_init();

    // Khởi tạo GPS NEO-6M (Cấu hình mặc định UART_NUM_1, RX=6, TX=7)
    ESP_ERROR_CHECK(neo6m_init_default());

    ESP_LOGI(TAG, "--- HE THONG TASK HOAN THANH ---");

    // 4. Khởi chạy 5 Luồng
    xTaskCreate(remote_find_task, "Find_Task", 4096, NULL, 5, NULL);
    xTaskCreate(mpu_monitor_task, "MPU_Task", 3072, NULL, 5, NULL);
    xTaskCreate(central_control_task, "Control_Task", 4096, NULL, 4, NULL);
    xTaskCreate(sim_gps_network_task, "SIM_Task", 4096, NULL, 4, NULL);
    xTaskCreate(ble_auth_task, "BLE_Task", 4096, NULL, 3, NULL);
}

// TASK 1: QUẢN LÝ NGỦ TIMER 3S
void mpu_monitor_task(void *pvParameters)
{
    float pitch = 0.0;
    int consecutive_tilt_count = 0;
    const int TILT_THRESHOLD_COUNT = 13;

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        if (mpu6050_get_pitch(&pitch) == ESP_OK)
        {
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            system_state_t current_state = g_system_state;
            xSemaphoreGive(xStateMutex);

            if (fabs(pitch) > 10.0)
            {
                consecutive_tilt_count++;
                ESP_LOGW(TAG, "Xe nghieng! Dem thoi gian: %d/%d (Pitch: %.2f)", consecutive_tilt_count, TILT_THRESHOLD_COUNT, pitch);

                if (consecutive_tilt_count >= TILT_THRESHOLD_COUNT)
                {
                    if (xSemaphoreTake(xSimMutex, 0) == pdTRUE)
                    {
                        xSemaphoreGive(xSimMutex);

                        xSemaphoreTake(xStateMutex, portMAX_DELAY);
                        g_system_state = STATE_SLEEPING;
                        xSemaphoreGive(xStateMutex);

                        relay_off();
                        buzzer_off();
                        ble_driver_stop_advertising();

                        // Cài đặt báo thức Timer 3 giây cho giấc ngủ Light Sleep
                        esp_sleep_enable_timer_wakeup(3 * 1000000ULL);
                        gpio_set_level(SIM_SLEEP_PIN, 0);

                        ESP_LOGE(TAG, "HỆ THỐNG VÀO GIẤC NGỦ LIGHT SLEEP (TIMER 3S)...");

                        esp_light_sleep_start();

                        // CHỦ ĐỘNG HỎI TRẠNG THÁI CUỘC GỌI BẰNG AT+CPAS
                        xSemaphoreTake(xSimMutex, portMAX_DELAY);
                        uart_flush_input(SIM_UART_NUM);
                        uart_write_bytes(SIM_UART_NUM, "AT+CPAS\r\n", 9);

                        uint8_t cpas_buf[64] = {0};
                        int len = uart_read_bytes(SIM_UART_NUM, cpas_buf, sizeof(cpas_buf) - 1, pdMS_TO_TICKS(200));
                        xSemaphoreGive(xSimMutex);

                        if (len > 0)
                        {
                            cpas_buf[len] = '\0';
                            if (strstr((char *)cpas_buf, "+CPAS: 3") != NULL || strstr((char *)cpas_buf, "RING") != NULL)
                            {
                                ESP_LOGW(TAG, "[CALL DETECTED] Phát hiện nháy máy tìm xe! Kích hoạt Task 5...");

                                xSemaphoreGive(xRemoteWakeSemaphore);
                                xSemaphoreTake(xSleepAgainSemaphore, portMAX_DELAY);

                                consecutive_tilt_count = TILT_THRESHOLD_COUNT;
                                continue;
                            }
                        }

                        consecutive_tilt_count = TILT_THRESHOLD_COUNT;
                        continue;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "[MPU] SIM đang bận, hoãn giấc ngủ...");
                        consecutive_tilt_count = TILT_THRESHOLD_COUNT - 3;
                    }
                }
            }
            else
            {
                consecutive_tilt_count = 0;
                if (current_state == STATE_SLEEPING)
                {
                    xSemaphoreTake(xStateMutex, portMAX_DELAY);
                    g_system_state = STATE_VERIFYING;
                    xSemaphoreGive(xStateMutex);

                    xSemaphoreGive(xWakeSemaphore);
                    ESP_LOGI(TAG, "Xe dựng thẳng! Kích hoạt TIẾN TRÌNH XÁC THỰC...");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

// TASK 5: DẬP MÁY & CHUYỂN STATE_REMOTE_FINDING ĐỂ TASK 3 PUSH FIREBASE
void remote_find_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xRemoteWakeSemaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "[TASK 5] Nhận lệnh cập nhật vị trí từ xa (Chế độ âm thầm)...");

            // 1. Dập cuộc gọi ngắt cước
            if (xSemaphoreTake(xSimMutex, pdMS_TO_TICKS(2000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "[ATH] Tiến hành dập cuộc gọi...");
                uart_write_bytes(SIM_UART_NUM, "ATH\r\n", 5);
                vTaskDelay(pdMS_TO_TICKS(500));
                uart_flush_input(SIM_UART_NUM);
                xSemaphoreGive(xSimMutex);
            }

            vTaskDelay(pdMS_TO_TICKS(1500));

            // 2. Xóa sạch cờ Push Firebase cũ
            xSemaphoreTake(xFirebaseDoneSemaphore, 0);

            // Chuyển trạng thái sang STATE_REMOTE_FINDING dành riêng cho tìm xe bãi
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_REMOTE_FINDING;
            xSemaphoreGive(xStateMutex);

            // 4. Đợi Task 3 push Firebase xong (Timeout 50s)
            ESP_LOGI(TAG, "Đang đợi Task 3 lấy GPS và gửi lên Firebase (Tối đa 50s)...");
            if (xSemaphoreTake(xFirebaseDoneSemaphore, pdMS_TO_TICKS(50000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "Task 3 đã xác nhận Push Firebase THÀNH CÔNG!");
            }
            else
            {
                ESP_LOGE(TAG, "Timeout 50s! Mạng quá yếu hoặc không lấy được GPS.");
            }

            ESP_LOGI(TAG, " Chu kỳ hoàn tất! Đưa hệ thống trở lại giấc ngủ...");

            // 5. Trả trạng thái về SLEEPING
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_SLEEPING;
            xSemaphoreGive(xStateMutex);

            // 6. Cho Task 1 đi ngủ tiếp
            xSemaphoreGive(xSleepAgainSemaphore);
        }
    }
}

// TASK 2: TRUNG TÂM ĐIỀU KHIỂN
void central_control_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xWakeSemaphore, portMAX_DELAY) == pdTRUE)
        {
            relay_off();
            ESP_LOGW(TAG, "Mach thuc giac! Da khoa cung he thong danh lua de cho xac thuc...");

            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_VERIFYING;
            xSemaphoreGive(xStateMutex);

            ESP_LOGI(TAG, "Dang cho App Flutter ket noi xac thuc trong 15s...");

            if (xSemaphoreTake(xAuthSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "Xac thuc thanh cong! Chao mung chu xe.");
                relay_on();

                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                g_system_state = STATE_OWNER_CONNECTED;
                xSemaphoreGive(xStateMutex);
            }
            else
            {
                ESP_LOGE(TAG, "CANH BAO: PAIRED THAT BAI! KICH HOAT BAO DONG.");
                relay_off();

                ESP_LOGW(TAG, "Dang hu coi canh bao trong 5 giay...");
                buzzer_on();
                vTaskDelay(pdMS_TO_TICKS(5000));
                buzzer_off();

                ESP_LOGE(TAG, "Da hu coi xong 5s. Chuyen sang STATE_ALARM va quay so khan cap...");

                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                g_system_state = STATE_ALARM;
                xSemaphoreGive(xStateMutex);

                xSemaphoreGive(xCallSemaphore);
            }
        }
    }
}

// TASK 3: THAO TÁC SIM VÀ PUSH FIREBASE (DÙNG NEO-6M LẤY GPS)
void sim_gps_network_task(void *pvParameters)
{
    char gps_payload[128];
    uint8_t net_buf[64];
    neo6m_gps_data_t gps_data = {0};

    while (1)
    {
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        system_state_t current_state = g_system_state;
        xSemaphoreGive(xStateMutex);

        if (current_state == STATE_VERIFYING)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // 1. BÃI XE: STATE_REMOTE_FINDING -> PUSH "PARKING_FIND"
        else if (current_state == STATE_REMOTE_FINDING)
        {
            // Đọc tọa độ GPS từ module NEO-6M
            neo6m_read_gps(&gps_data);
            double real_lat = gps_data.valid ? (double)gps_data.latitude : 0.0;
            double real_lng = gps_data.valid ? (double)gps_data.longitude : 0.0;

            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            // Khôi phục 4G nếu bị ngắt sau ATH
            uart_flush_input(SIM_UART_NUM);
            uart_write_bytes(SIM_UART_NUM, "AT+NETOPEN?\r\n", 13);
            memset(net_buf, 0, sizeof(net_buf));
            int len = uart_read_bytes(SIM_UART_NUM, net_buf, sizeof(net_buf) - 1, pdMS_TO_TICKS(500));

            if (len <= 0 || strstr((char *)net_buf, "+NETOPEN: 1") == NULL)
            {
                sim_send_cmd("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 1000);
                sim_send_cmd("AT+NETOPEN", "OK", 2000);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            snprintf(gps_payload, sizeof(gps_payload),
                     "{\"latitude\":%.6f,\"longitude\":%.6f,\"status\":\"PARKING_FIND\"}",
                     real_lat, real_lng);

            ESP_LOGI(TAG, "[BÃI XE] Push JSON: %s", gps_payload);

            sim_send_to_firebase(gps_payload);
            xSemaphoreGive(xSimMutex);

            if (xFirebaseDoneSemaphore != NULL)
            {
                xSemaphoreGive(xFirebaseDoneSemaphore);
            }

            vTaskDelay(pdMS_TO_TICKS(10000));
        }

        // 2. CHỦ LÁI XE: STATE_OWNER_CONNECTED -> PUSH "OWNER_DRIVING"
        else if (current_state == STATE_OWNER_CONNECTED)
        {
            // Đọc tọa độ GPS từ module NEO-6M
            neo6m_read_gps(&gps_data);
            double real_lat = gps_data.valid ? (double)gps_data.latitude : 0.0;
            double real_lng = gps_data.valid ? (double)gps_data.longitude : 0.0;

            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            // Khôi phục 4G nếu bị ngắt kết nối
            uart_flush_input(SIM_UART_NUM);
            uart_write_bytes(SIM_UART_NUM, "AT+NETOPEN?\r\n", 13);
            memset(net_buf, 0, sizeof(net_buf));
            int len = uart_read_bytes(SIM_UART_NUM, net_buf, sizeof(net_buf) - 1, pdMS_TO_TICKS(500));

            if (len <= 0 || strstr((char *)net_buf, "+NETOPEN: 1") == NULL)
            {
                sim_send_cmd("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 1000);
                sim_send_cmd("AT+NETOPEN", "OK", 2000);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            snprintf(gps_payload, sizeof(gps_payload),
                     "{\"latitude\":%.6f,\"longitude\":%.6f,\"status\":\"OWNER_DRIVING\"}",
                     real_lat, real_lng);

            ESP_LOGI(TAG, "🏍️ [CHỦ XE LÁI] Push JSON: %s", gps_payload);

            sim_send_to_firebase(gps_payload);
            xSemaphoreGive(xSimMutex);

            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        // 3. TRỘM DẮT XE: STATE_ALARM -> PUSH "THEFT_ALARM"
        else if (current_state == STATE_ALARM)
        {
            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            if (xSemaphoreTake(xCallSemaphore, 0) == pdTRUE)
            {
                ESP_LOGE(TAG, "[TRỘM] GỌI ĐIỆN BÁO ĐỘNG CHO CHỦ XE!");
                sim_make_call(OWNER_PHONE_NUMBER);

                // Đổ chuông báo động trong 6 giây
                vTaskDelay(pdMS_TO_TICKS(6000));

                ESP_LOGI(TAG, "[TRỘM] Dập cuộc gọi thoại (ATH) để giải phóng mạng Data...");
                sim_hang_up();
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            // Khôi phục 4G sau cuộc gọi
            uart_flush_input(SIM_UART_NUM);
            uart_write_bytes(SIM_UART_NUM, "AT+NETOPEN?\r\n", 13);
            memset(net_buf, 0, sizeof(net_buf));
            int len = uart_read_bytes(SIM_UART_NUM, net_buf, sizeof(net_buf) - 1, pdMS_TO_TICKS(500));

            if (len <= 0 || strstr((char *)net_buf, "+NETOPEN: 1") == NULL)
            {
                sim_send_cmd("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 1000);
                sim_send_cmd("AT+NETOPEN", "OK", 2000);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            // Đọc tọa độ GPS từ module NEO-6M
            neo6m_read_gps(&gps_data);
            double real_lat = gps_data.valid ? (double)gps_data.latitude : 0.0;
            double real_lng = gps_data.valid ? (double)gps_data.longitude : 0.0;

            snprintf(gps_payload, sizeof(gps_payload),
                     "{\"latitude\":%.6f,\"longitude\":%.6f,\"status\":\"THEFT_ALARM\"}",
                     real_lat, real_lng);

            ESP_LOGE(TAG, "🚨 [TRỘM DẮT XE] Push JSON: %s", gps_payload);

            sim_send_to_firebase(gps_payload);
            xSemaphoreGive(xSimMutex);

            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// TASK 4: QUẢN LÝ PHÁT BLE
void ble_auth_task(void *pvParameters)
{
    system_state_t last_state = STATE_SLEEPING;

    while (1)
    {
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        system_state_t current_state = g_system_state;
        xSemaphoreGive(xStateMutex);

        if (current_state == STATE_VERIFYING)
        {
            if (last_state != STATE_VERIFYING)
            {
                ble_driver_start_advertising();
                ESP_LOGI(TAG, "Anten BLE đang phát quảng bá tìm điện thoại...");
            }
        }

        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}