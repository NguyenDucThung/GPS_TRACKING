idf_component_register(
    SRCS "sim_a7600e.c" 
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart
)