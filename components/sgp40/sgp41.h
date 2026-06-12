/*
    Based on sgp40.c by Igor Jensen - UFES - LAEEC
    Adapted for SGP41 (VOC + NOx)
*/

#pragma once
#ifndef SGP41_H
#define SGP41_H
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "sensirion_gas.h"

esp_err_t sgp41_init(i2c_master_bus_handle_t bus);

// Roda o ciclo de condicionamento do NOx (deve ser chamado por ~10s antes do uso normal)
esp_err_t sgp41_execute_conditioning(float humidity_percent, float temperature_celsius, uint16_t *voc_raw);

// Leitura raw compensada de VOC e NOx
esp_err_t sgp41_measure_raw(float humidity_percent, float temperature_celsius, uint16_t *voc_raw, uint16_t *nox_raw);

// Atualiza ambos os algoritmos e retorna os índices VOC e NOx
esp_err_t sgp41_get_indices(float humidity_percent, float temperature_celsius, int32_t *voc_index, int32_t *nox_index);

esp_err_t sgp41_self_test(uint16_t *test_result);

#endif