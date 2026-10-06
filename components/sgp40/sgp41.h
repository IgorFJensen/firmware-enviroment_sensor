/* SGP41 VOC + NOx */
#pragma once
#ifndef SGP41_H
#define SGP41_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "sensirion_gas.h"

esp_err_t sgp41_init(i2c_master_bus_handle_t bus);

/* Repetir por aproximadamente 10 s antes do modo normal de NOx. */
esp_err_t sgp41_execute_conditioning(
    float humidity_percent,
    float temperature_celsius,
    uint16_t *voc_raw);

esp_err_t sgp41_measure_raw(
    float humidity_percent,
    float temperature_celsius,
    uint16_t *voc_raw,
    uint16_t *nox_raw);

/* Chamar em cadencia de aproximadamente 1 Hz. */
esp_err_t sgp41_get_indices(
    float humidity_percent,
    float temperature_celsius,
    int32_t *voc_index,
    int32_t *nox_index);

esp_err_t sgp41_self_test(uint16_t *test_result);

#endif
