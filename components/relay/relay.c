#include "relay.h"
#include "esp_log.h"

static const char *TAG = "RELAY_DRIVER";
static bool relay_state = false; // Biến static lưu trạng thái nội bộ của Relay

// Khởi tạo GPIO cho Relay
void relay_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,       // Cho phép cả xuất và đọc lại trạng thái chân
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Trạng thái ban đầu: Tắt Relay
    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
    relay_state = false;
    
    ESP_LOGI(TAG, "Khoi tao GPIO %d cho Relay thanh cong.", RELAY_PIN);
}

// Hàm bật Relay
void relay_on(void) {
    gpio_set_level(RELAY_PIN, RELAY_ON_LEVEL);
    relay_state = true;
    ESP_LOGD(TAG, "Relay -> ON");
}

// Hàm tắt Relay
void relay_off(void) {
    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
    relay_state = false;
    ESP_LOGD(TAG, "Relay -> OFF");
}

// Hàm đảo trạng thái Relay (Đang bật thành tắt, đang tắt thành bật)
void relay_toggle(void) {
    if (relay_state) {
        relay_off();
    } else {
        relay_on();
    }
}