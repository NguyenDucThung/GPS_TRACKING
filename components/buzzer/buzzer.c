#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BUZZER_DRIVER";

// Khởi tạo chân GPIO làm Output
void buzzer_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,            // Chế độ xuất tín hiệu
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Kéo xuống thấp lúc khởi động để còi không kêu bậy
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Trạng thái ban đầu: Tắt còi
    gpio_set_level(BUZZER_PIN, 0);
    ESP_LOGI(TAG, "Khoi tao GPIO %d cho coi 5V thanh cong.", BUZZER_PIN);
}

// Hàm bật còi (Xuất mức 1)
void buzzer_on(void) {
    gpio_set_level(BUZZER_PIN, 1);
}

// Hàm tắt còi (Xuất mức 0)
void buzzer_off(void) {
    gpio_set_level(BUZZER_PIN, 0);
}

// Hàm chớp còi kêu "Tít" một phát rồi tắt (Dùng báo hiệu rất tiện)
void buzzer_beep(uint32_t duration_ms) {
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_off();
}