idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES mpu6050 a7600e neo6m buzzer relay ble_driver bt nvs_flash driver esp_driver_uart esp_driver_gpio
)