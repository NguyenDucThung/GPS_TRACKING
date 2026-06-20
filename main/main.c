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
#include "driver/uart.h" // Thư viện dùng hàm xả rác UART chống reset

// --- CẤU HÌNH SỐ ĐIỆN THOẠI CHỦ XE VÀ CHÂN CỦA BẠN ---
#define OWNER_PHONE_NUMBER "xxxxxxxxxx"
#define SIM_SLEEP_PIN 3 // Chân điều khiển ngủ DTR/SLEEP của module SIM (Thay bằng chân bạn nối)

// Include các driver độc lập từ thư mục components
#include "mpu6050.h"
#include "sim_a7600e.h" // Nhận diện SIM_RX_PIN (GPIO_NUM_5) và SIM_UART_NUM từ đây
#include "ble_driver.h"
#include "buzzer.h"
#include "relay.h"

static const char *TAG = "MAIN_SYSTEM";

typedef enum
{
    STATE_SLEEPING,
    STATE_VERIFYING,
    STATE_OWNER_CONNECTED,
    STATE_ALARM
} system_state_t;

// Biến trạng thái toàn cục và các công cụ đồng bộ RTOS
static system_state_t g_system_state = STATE_SLEEPING;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xWakeSemaphore;
SemaphoreHandle_t xAuthSemaphore;
SemaphoreHandle_t xCallSemaphore;
SemaphoreHandle_t xSimMutex;

// 🔥 CỜ LỆNH ĐỒNG BỘ DÀNH RIÊNG CHO TASK 5 TÌM XE TỪ XA
SemaphoreHandle_t xRemoteWakeSemaphore;
SemaphoreHandle_t xSleepAgainSemaphore;

// Khai báo nguyên mẫu các Task
void mpu_monitor_task(void *pvParameters);
void central_control_task(void *pvParameters);
void sim_gps_network_task(void *pvParameters);
void ble_auth_task(void *pvParameters);
void remote_find_task(void *pvParameters); // 🔥 Khai báo Task 5 xử lý cuộc gọi tìm xe

// ====================================================================
// HÀM CẦU NỐI: TIẾP NHẬN TÍN HIỆU XÁC THỰC THÀNH CÔNG TỪ LUỒNG BLE
// ====================================================================
void main_system_auth_success(void)
{
    if (xAuthSemaphore != NULL)
    {
        xSemaphoreGive(xAuthSemaphore);
        ESP_LOGI(TAG, "🎯 [BRIDGE] Da tiep nhan tin hieu tu BLE! Mo khoa xe cap toc...");
    }
}

void app_main(void)
{
    // 1. Khởi tạo cơ chế đồng bộ RTOS
    xStateMutex = xSemaphoreCreateMutex();
    xSimMutex = xSemaphoreCreateMutex();

    xWakeSemaphore = xSemaphoreCreateBinary();
    xAuthSemaphore = xSemaphoreCreateBinary();
    xCallSemaphore = xSemaphoreCreateBinary();
    xRemoteWakeSemaphore = xSemaphoreCreateBinary();
    xSleepAgainSemaphore = xSemaphoreCreateBinary();

    // 2. Khởi tạo toàn bộ phần cứng ngoại vi
    buzzer_init();
    relay_init();

    // Cấu hình chân ngắt từ cảm biến nghiêng MPU6050 (Mức CAO)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL};
    gpio_config(&io_conf);
    gpio_wakeup_enable(WAKEUP_GPIO_PIN, GPIO_INTR_HIGH_LEVEL);

    // 🔥 CẤU HÌNH NGẮT CHO CHÂN RX (LẤY TỪ THƯ VIỆN) ĐỂ NHẬN CUỘC GỌI KHI NGỦ (MỨC THẤP)
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << SIM_RX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL};
    gpio_config(&rx_conf);
    gpio_wakeup_enable(SIM_RX_PIN, GPIO_INTR_LOW_LEVEL);

    // Cấu hình chân SLEEP điều khiển ngủ module SIM
    gpio_config_t sleep_pin_conf = {
        .pin_bit_mask = (1ULL << SIM_SLEEP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&sleep_pin_conf);
    gpio_set_level(SIM_SLEEP_PIN, 0);

    esp_sleep_enable_gpio_wakeup();

    mpu6050_init();
    ble_driver_init();
    sim_a7600e_power_on();
    sim_a7600e_init(); // Cài Driver UART 1 lần duy nhất

    ESP_LOGI(TAG, "--- HE THONG PHAN PHIEU TASK HOAN THANH ---");

    // 3. Khởi chạy 5 Luồng xử lý độc lập
    xTaskCreate(mpu_monitor_task, "MPU_Task", 3072, NULL, 5, NULL);
    xTaskCreate(central_control_task, "Control_Task", 4096, NULL, 4, NULL);
    xTaskCreate(sim_gps_network_task, "SIM_Task", 4096, NULL, 4, NULL);
    xTaskCreate(ble_auth_task, "BLE_Task", 4096, NULL, 3, NULL);
    xTaskCreate(remote_find_task, "Find_Task", 4096, NULL, 5, NULL); // 🔥 Chạy Task 5 tìm xe
}

// ====================================================================
// TASK 1: GIÁM SÁT NGỦ NGẮT THUẦN TÚY
// ====================================================================
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
                    xSemaphoreTake(xStateMutex, portMAX_DELAY);
                    g_system_state = STATE_SLEEPING;
                    xSemaphoreGive(xStateMutex);

                    relay_off();
                    buzzer_off();
                    ble_driver_stop_advertising();
                    mpu6050_clear_interrupt();

                    // 🔥 ĐÃ SỬA: Bỏ chế độ máy bay. Kéo chân SLEEP lên cao để SIM vào giấc ngủ giữ sóng mạng.
                    gpio_set_level(SIM_SLEEP_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));

                    ESP_LOGE(TAG, "💤 HỆ THỐNG VÀO GIẤC NGỦ THUẦN TÚY (ĐANG CHỜ CUỘC GỌI TÌM XE CỦA BẠN)...");
                    esp_light_sleep_start();

                    // ====================================================
                    // --- MẠCH THỨC GIẤC TẠI ĐÂY (DO THẲNG XE HOẶC GỌI ĐIỆN) ---
                    // ====================================================
                    gpio_set_level(SIM_SLEEP_PIN, 0); // Kéo thấp chân SLEEP để gọi SIM tỉnh hẳn dậy nhận lệnh UART

                    // 🔍 ĐÁNH GIÁ XEM AI GỌI MẠCH DẬY?
                    uint64_t wakeup_pin = esp_sleep_get_gpio_wakeup_status();

                    if (wakeup_pin & (1ULL << SIM_RX_PIN))
                    {
                        // 📞 NGUỒN NGẮT DO CUỘC GỌI NHÁY MÁY TÌM XE
                        xSemaphoreGive(xRemoteWakeSemaphore); // Bắn cờ lệnh kích hoạt Task 5 chạy riêng

                        xSemaphoreTake(xSleepAgainSemaphore, portMAX_DELAY); // Đứng đợi Task 5 hoàn thành rồi mới quét tiếp
                        consecutive_tilt_count = TILT_THRESHOLD_COUNT;
                        continue;
                    }

                    // NGUỒN NGẮT VẬT LÝ (DỰNG THẲNG XE)
                    ESP_LOGW(TAG, "⚡ Kích hoạt phần cứng! Hoi sinh khoi I2C va Clear Interrupt...");
                    i2c_driver_delete(I2C_NUM_0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    mpu6050_init();
                    mpu6050_clear_interrupt();
                    consecutive_tilt_count = 0;
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
                    ESP_LOGI(TAG, "📢 Xe dung thang! Kich hoat TIEN TRINH XAC THUC VỚI APP...");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

// ====================================================================
// 🔥 TASK 5 (VIẾT MỚI): XỬ LÝ NHÁY MÁY TÌM XE TỪ XA VÀ TỰ ĐỘNG NGỦ LẠI
// ====================================================================
void remote_find_task(void *pvParameters)
{
    uint8_t uart_buf[64];

    while (1)
    {
        // Nằm im chờ cờ ngắt từ cuộc gọi nháy máy của chủ xe
        if (xSemaphoreTake(xRemoteWakeSemaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGW(TAG, "📞 [REMOTE FIND] Nhận tín hiệu xung UART từ cuộc gọi đến!");
            vTaskDelay(pdMS_TO_TICKS(500)); // Trễ nhẹ đợi SIM bắn nốt text "RING" vào buffer

            // 🔥 ĐÃ SỬA: Dùng SIM_UART_NUM toàn cục từ thư viện để tránh lỗi undeclared
            int len = uart_read_bytes(SIM_UART_NUM, uart_buf, sizeof(uart_buf) - 1, pdMS_TO_TICKS(100));

            if (len > 0)
            {
                uart_buf[len] = '\0';
                if (strstr((char *)uart_buf, "RING") != NULL)
                {
                    ESP_LOGI(TAG, "🎯 XÁC NHẬN CHUỖI 'RING'! Hú còi định hướng và đẩy vị trí...");

                    // Hú còi tít tít 3 phát ngắn để tìm xe trong hầm
                    for (int i = 0; i < 3; i++)
                    {
                        buzzer_on();
                        vTaskDelay(pdMS_TO_TICKS(150));
                        buzzer_off();
                        vTaskDelay(pdMS_TO_TICKS(150));
                    }

                    // Chuyển trạng thái sang OWNER_CONNECTED kích hoạt luồng SIM bật mạng
                    xSemaphoreTake(xStateMutex, portMAX_DELAY);
                    g_system_state = STATE_OWNER_CONNECTED;
                    xSemaphoreGive(xStateMutex);

                    // Giữ mạch thức 12 giây để SIM Task thu GPS và bắn Firebase
                    vTaskDelay(pdMS_TO_TICKS(12000));
                    ESP_LOGI(TAG, "🏁 Chu kỳ đẩy vị trí tìm xe hoàn tất!");
                }
            }

            // Đồng bộ đưa hệ thống về lại trạng thái ngủ ngủ đông
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_SLEEPING;
            xSemaphoreGive(xStateMutex);

            // Báo cờ giải thoát cho Task 1 đưa chip vào giấc ngủ Light Sleep tiếp tục
            xSemaphoreGive(xSleepAgainSemaphore);
        }
    }
}

// ====================================================================
// TASK 2: BỘ NÃO TRUNG TÂM (LOGIC MẠCH HỞ CHÂN NO)
// ====================================================================
void central_control_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xWakeSemaphore, portMAX_DELAY) == pdTRUE)
        {
            relay_off();
            ESP_LOGW(TAG, "🚨 Mach thuc giac! Da khoa cung he thong danh lua de cho xac thuc...");

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

// ====================================================================
// TASK 3: XỬ LÝ MẠNG TỐC ĐỘ CAO (ĐÃ KHẮC PHỤC BIẾN HỎNG)
// ====================================================================
void sim_gps_network_task(void *pvParameters)
{
    static bool s_sim_online = false;
    system_state_t last_state = STATE_SLEEPING;

    while (1)
    {
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        system_state_t current_state = g_system_state;
        xSemaphoreGive(xStateMutex);

        if (current_state != last_state)
        {
            s_sim_online = false;
        }

        if (current_state == STATE_VERIFYING)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        else if (current_state == STATE_OWNER_CONNECTED)
        {
            relay_on();

            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            if (!s_sim_online)
            {
                // 🔥 ĐÃ SỬA: Thay sim_uart_port bằng SIM_UART_NUM toàn cục sạch lỗi
                uart_flush_input(SIM_UART_NUM);
                sim_set_airplane_mode(false);
                vTaskDelay(pdMS_TO_TICKS(500));

                sim_send_cmd("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 2000);
                sim_send_cmd("AT+NETOPEN", "OK", 2000);

                if (sim_send_cmd("AT+NETOPEN?", "+NETOPEN: 1", 2000) == true)
                {
                    sim_gps_enable();
                    s_sim_online = true;
                    ESP_LOGI(TAG, "🌐 SIM ONLINE!");
                }
            }

            if (s_sim_online)
            {
                sim_gps_get_info();
                char *gps_data = sim_get_gps_location();
                sim_send_to_firebase(gps_data);
                xSemaphoreGive(xSimMutex);
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            else
            {
                xSemaphoreGive(xSimMutex);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
        else if (current_state == STATE_ALARM)
        {
            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            if (!s_sim_online)
            {
                // 🔥 ĐÃ SỬA: Dùng SIM_UART_NUM toàn cục sạch lỗi
                uart_flush_input(SIM_UART_NUM);
                sim_set_airplane_mode(false);
                vTaskDelay(pdMS_TO_TICKS(500));
                s_sim_online = true;
            }

            if (xSemaphoreTake(xCallSemaphore, 0) == pdTRUE)
            {
                ESP_LOGE(TAG, "📞 GOI DIEN BAO DONG!");
                sim_make_call(OWNER_PHONE_NUMBER);

                sim_send_cmd("AT+NETOPEN", "OK", 2000);
                sim_gps_enable();
            }

            sim_gps_get_info();
            char *gps_data = sim_get_gps_location();
            sim_send_to_firebase(gps_data);
            xSemaphoreGive(xSimMutex);

            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        last_state = current_state;
    }
}

// ====================================================================
// TASK 4: QUẢN LÝ PHÁT BLE VÀ XÁC THỰC VỚI FLUTTER APP
// ====================================================================
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