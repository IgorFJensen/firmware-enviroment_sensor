#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} dps310_calib_data_t;

esp_err_t dps310_init(i2c_master_bus_handle_t bus);
esp_err_t dps310_read(float *temperature, float *pressure);
