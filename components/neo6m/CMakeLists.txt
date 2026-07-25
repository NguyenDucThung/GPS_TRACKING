idf_component_register(
    SRCS "neo6m.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_gpio
)