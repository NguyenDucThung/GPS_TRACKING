#pragma once

#include "esp_err.h"
#include "driver/gpio.h"

// --- CẤU HÌNH PHẦN CỨNG (Sửa chân ở đây nếu thay đổi mạch) ---
#define I2C_MASTER_SCL_IO     9
#define I2C_MASTER_SDA_IO     8
#define WAKEUP_GPIO_PIN       GPIO_NUM_2  // Chân INT của MPU6050 nối vào đây

#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_FREQ_HZ    400000
#define MPU6050_ADDR          0x68

// --- ĐỊNH NGHĨA THANH GHI ---
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_INT_STATUS    0x3A
#define MPU6050_INT_PIN_CFG   0x37
#define MPU6050_INT_ENABLE    0x38
#define MPU6050_MOT_THR       0x1F
#define MPU6050_MOT_DUR       0x20
#define MPU6050_ACCEL_CONFIG  0x1C

// --- CÁC HÀM CÔNG KHAI ---
/**
 * @brief Khởi tạo ngoại vi I2C và cấu hình ngắt phát hiện chuyển động trên MPU6050
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Đọc dữ liệu từ cảm biến và tính toán góc nghiêng Pitch
 * @param pitch Biến con trỏ để lưu giá trị góc tính được
 * @return ESP_OK nếu đọc thành công
 */
esp_err_t mpu6050_get_pitch(float *pitch);

/**
 * @brief Đọc xóa cờ ngắt trên thanh ghi 0x3A (Bắt buộc gọi trước khi ngủ để xóa ngắt cũ)
 */
void mpu6050_clear_interrupt(void);