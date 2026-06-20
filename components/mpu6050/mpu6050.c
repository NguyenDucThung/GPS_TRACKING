#include "mpu6050.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050_DRIVER";

// --- CÁC HÀM TRỢ GIÚP NỘI BỘ (STATIC) ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_bytes(uint8_t start_reg, uint8_t *buffer, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &start_reg, 1, buffer, len, pdMS_TO_TICKS(1000));
}

// --- TRIỂN KHAI CÁC HÀM CÔNG KHAI ---

esp_err_t mpu6050_init(void) {
    // 1. Khởi tạo cổng I2C phần cứng
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Cấu hình các thanh ghi tính năng ngắt của MPU6050
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x01);
    mpu6050_write_reg(MPU6050_MOT_THR, 10); // Ngưỡng nhạy ngắt chuyển động
    mpu6050_write_reg(MPU6050_MOT_DUR, 1);

    // Chân INT phát dạng xung (Pulse) rồi tự hạ về 0V
    mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x00);
    mpu6050_write_reg(MPU6050_INT_ENABLE, 0x40);

    ESP_LOGI(TAG, "MPU6050 I2C & Interrupt initialized successfully.");
    return ESP_OK;
}

esp_err_t mpu6050_get_pitch(float *pitch) {
    uint8_t data[6];
    if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6) == ESP_OK) {
        int16_t ay = (int16_t)((data[2] << 8) | data[3]);
        int16_t az = (int16_t)((data[4] << 8) | data[5]);
        
        // Tính toán góc nghiêng Pitch độc lập
        *pitch = atan2((float)ay, (float)az) * 180.0 / M_PI;
        return ESP_OK;
    }
    return ESP_FAIL;
}

void mpu6050_clear_interrupt(void) {
    uint8_t dummy;
    mpu6050_read_bytes(MPU6050_INT_STATUS, &dummy, 1);
}