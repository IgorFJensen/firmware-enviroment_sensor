#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "sensirion_gas.h"

esp_err_t sgp40_init(i2c_master_bus_handle_t bus);
esp_err_t sgp40_read_raw(uint16_t *voc_raw);
esp_err_t sgp40_measure_compensated(float humidity_percent, float temperature_celsius, uint16_t *voc_raw);
esp_err_t sgp40_get_voc_index(float humidity_percent, float temperature_celsius, int32_t *voc_index);
