idf_component_register(
    SRCS "sht40.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)