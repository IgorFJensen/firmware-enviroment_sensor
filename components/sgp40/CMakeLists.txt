idf_component_register(
    SRCS "sgp41.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
             "sensirion_gas"
)