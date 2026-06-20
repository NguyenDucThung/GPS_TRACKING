idf_component_register(
    SRCS "mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio
    PRIV_REQUIRES driver
)