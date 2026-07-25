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

// VỊ TRÍ LƯU TRỮ DỰ PHÒNG
static double g_last_saved_lat = 0.0;
static double g_last_saved_lng = 0.0;

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

    gpio_set_level(SIM_SLEEP_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "-> Chân PWR (GPIO%d) lên HIGH (Giữ 3s)...", SIM_SLEEP_PIN);
    gpio_set_level(SIM_SLEEP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    gpio_set_level(SIM_SLEEP_PIN, 0);
    ESP_LOGI(TAG, "-> Đã thả chân PWR về LOW. Chờ SIM bám mạng ổn định trong 5s...");
    vTaskDelay(pdMS_TO_TICKS(5000));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xStateMutex = xSemaphoreCreateMutex();
    xSimMutex = xSemaphoreCreateMutex();

    xWakeSemaphore = xSemaphoreCreateBinary();
    xAuthSemaphore = xSemaphoreCreateBinary();
    xCallSemaphore = xSemaphoreCreateBinary();
    xRemoteWakeSemaphore = xSemaphoreCreateBinary();
    xSleepAgainSemaphore = xSemaphoreCreateBinary();
    xFirebaseDoneSemaphore = xSemaphoreCreateBinary();

    buzzer_init();
    relay_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL};
    gpio_config(&io_conf);
    gpio_wakeup_enable(WAKEUP_GPIO_PIN, GPIO_INTR_HIGH_LEVEL);

    gpio_config_t sleep_pin_conf = {
        .pin_bit_mask = (1ULL << SIM_SLEEP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&sleep_pin_conf);

    esp_sleep_enable_gpio_wakeup();

    power_on_sequence();
    mpu6050_init();
    ble_driver_init();
    sim_a7600e_init();

    // Khởi tạo GPS (Mặc định ngắt nguồn)
    neo6m_power_init();

    ESP_LOGI(TAG, "--- HE THONG TASK HOAN THANH ---");

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

                        // Đảm bảo GPS đã tắt triệt để trước khi vào giấc ngủ
                        neo6m_set_power(false);

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

// TASK 5: DẬP MÁY & CHUYỂN STATE_REMOTE_FINDING ĐỂ TASK 3 LẤY GPS
// TASK 5: DẬP MÁY & CHUYỂN STATE_REMOTE_FINDING ĐỂ TASK 3 LẤY GPS
void remote_find_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xRemoteWakeSemaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "[TASK 5] Nhận lệnh cập nhật vị trí từ xa...");

            // 1. Dập cuộc gọi ngắt cước
            if (xSemaphoreTake(xSimMutex, pdMS_TO_TICKS(2000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "[ATH] Tiến hành dập cuộc gọi...");
                uart_write_bytes(SIM_UART_NUM, "ATH\r\n", 5);
                vTaskDelay(pdMS_TO_TICKS(500));
                uart_flush_input(SIM_UART_NUM);
                xSemaphoreGive(xSimMutex);
            }

            // [MỚI]: Cho SIM và nguồn điện nghỉ hẳn 2 giây để ổn định điện áp sau khi ATH
            ESP_LOGI(TAG, "⏳ Cho SIM nghỉ 2s để ổn định nguồn điện và mạng thoại...");
            vTaskDelay(pdMS_TO_TICKS(2000));

            // Reset cờ Push Firebase cũ
            xSemaphoreTake(xFirebaseDoneSemaphore, 0);

            // 2. Chuyển state để Task 3 bắt đầu bật GPS và chốt vệ tinh
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_REMOTE_FINDING;
            xSemaphoreGive(xStateMutex);

            // 3. Đợi Task 3 chốt vệ tinh và gửi Firebase (Cho tối đa 60 giây vì Cold Start)
            ESP_LOGI(TAG, "Đang đợi Task 3 ép GPS khóa vệ tinh và Push Firebase (Tối đa 60s)...");
            if (xSemaphoreTake(xFirebaseDoneSemaphore, pdMS_TO_TICKS(60000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "Task 3 đã Push Firebase THÀNH CÔNG! Kết thúc chu trình.");
            }
            else
            {
                ESP_LOGE(TAG, "Timeout 60s! Không bắt được vệ tinh (Có thể đang ở tầng hầm). Bỏ qua!");
            }

            // Chắc chắn tắt GPS thêm một lần nữa cho an toàn trước khi ngủ
            neo6m_set_power(false);

            // 4. Đưa hệ thống quay trở lại giấc ngủ
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            g_system_state = STATE_SLEEPING;
            xSemaphoreGive(xStateMutex);

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

// TASK 3: THAO TÁC SIM VÀ GPS
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

        // =========================================================================
        // 1. NHÁY MÁY TÌM XE: ÉP BẮT TỌA ĐỘ THẬT -> GỬI 1 LẦN -> TẮT GPS
        // =========================================================================
        else if (current_state == STATE_REMOTE_FINDING)
        {
            // 1. Bật nguồn GPS
            if (!neo6m_is_powered())
            {
                neo6m_set_power(true);
            }

            ESP_LOGI(TAG, "🛰️ [PARKING_FIND] Đã BẬT GPS! Đang ép chờ khóa vệ tinh thực tế...");

            neo6m_gps_data_t new_gps = {0};
            bool fix_ok = false;
            int wait_sec = 0;

            // 2. Vòng lặp chờ bằng được tọa độ có valid == true (Tối đa 55 giây tránh timeout Task 5)
            while (wait_sec < 55)
            {
                wait_sec++;
                ESP_LOGI(TAG, "⏳ Đang dò tìm vệ tinh (Cold Start)... (%d giây)", wait_sec);

                if (neo6m_get_latest_fix(&new_gps, 1000) && new_gps.valid)
                {
                    fix_ok = true;
                    g_last_saved_lat = (double)new_gps.latitude;
                    g_last_saved_lng = (double)new_gps.longitude;
                    ESP_LOGI(TAG, "🎯 BẮT ĐƯỢC TỌA ĐỘ THẬT: Lat=%.6f, Lng=%.6f", g_last_saved_lat, g_last_saved_lng);
                    break; // Thoát vòng lặp chờ khi đã có tọa độ chuẩn
                }

                // Nếu đang chờ mà Task 5 hết timeout ép về SLEEPING thì tự văng ra
                xSemaphoreTake(xStateMutex, portMAX_DELAY);
                system_state_t check_state = g_system_state;
                xSemaphoreGive(xStateMutex);
                if (check_state != STATE_REMOTE_FINDING)
                {
                    break;
                }
            }

            if (fix_ok)
            {
                // 3. Chỉ khi bắt được vệ tinh mới Push lên Firebase
                xSemaphoreTake(xSimMutex, portMAX_DELAY);

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
                         g_last_saved_lat, g_last_saved_lng);

                ESP_LOGI(TAG, "🚀 [PARKING_FIND] Bắn JSON lên Firebase: %s", gps_payload);
                sim_send_to_firebase(gps_payload);
                xSemaphoreGive(xSimMutex);

                // 4. Bắn xong 1 phát duy nhất -> LẬP TỨC TẮT GPS TIẾT KIỆM ĐIỆN
                neo6m_set_power(false);
                ESP_LOGI(TAG, "🔌 Đã bắn Firebase xong -> TẮT NGUỒN GPS NGAY LẬP TỨC!");

                // Báo cho Task 5 biết đã hoàn thành để nó cho xe ngủ tiếp
                if (xFirebaseDoneSemaphore != NULL)
                {
                    xSemaphoreGive(xFirebaseDoneSemaphore);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        // =========================================================================
        // 2. CHỦ LÁI XE: STATE_OWNER_CONNECTED -> PUSH "OWNER_DRIVING"
        // =========================================================================
        else if (current_state == STATE_OWNER_CONNECTED)
        {
            if (!neo6m_is_powered())
            {
                neo6m_set_power(true);
            }

            if (neo6m_get_latest_fix(&gps_data, 2000) && gps_data.valid)
            {
                g_last_saved_lat = (double)gps_data.latitude;
                g_last_saved_lng = (double)gps_data.longitude;
            }

            double real_lat = gps_data.valid ? (double)gps_data.latitude : g_last_saved_lat;
            double real_lng = gps_data.valid ? (double)gps_data.longitude : g_last_saved_lng;

            xSemaphoreTake(xSimMutex, portMAX_DELAY);

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

        // =========================================================================
        // 3. TRỘM DẮT XE: STATE_ALARM -> PUSH "THEFT_ALARM"
        // =========================================================================
        else if (current_state == STATE_ALARM)
        {
            if (!neo6m_is_powered())
            {
                neo6m_set_power(true);
            }

            xSemaphoreTake(xSimMutex, portMAX_DELAY);

            if (xSemaphoreTake(xCallSemaphore, 0) == pdTRUE)
            {
                ESP_LOGE(TAG, "[TRỘM] GỌI ĐIỆN BÁO ĐỘNG CHO CHỦ XE!");
                sim_make_call(OWNER_PHONE_NUMBER);

                vTaskDelay(pdMS_TO_TICKS(6000));

                ESP_LOGI(TAG, "[TRỘM] Dập cuộc gọi thoại (ATH) để giải phóng mạng Data...");
                sim_hang_up();
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

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

            if (neo6m_get_latest_fix(&gps_data, 2000) && gps_data.valid)
            {
                g_last_saved_lat = (double)gps_data.latitude;
                g_last_saved_lng = (double)gps_data.longitude;
            }

            double real_lat = gps_data.valid ? (double)gps_data.latitude : g_last_saved_lat;
            double real_lng = gps_data.valid ? (double)gps_data.longitude : g_last_saved_lng;

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