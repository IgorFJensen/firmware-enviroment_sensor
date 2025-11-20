/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#pragma once
#ifndef DPS310_H
#define DPS310_H
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


/**
 * @brief Inicializa o sensor DPS310.
 * @param bus Handle do barramento I2C.
 * @return ESP_OK se OK, erro caso contrário.
 */
esp_err_t dps310_init(i2c_master_bus_handle_t bus);

/**
 * @brief Lê temperatura e pressão já compensadas.
 * @param temperature Ponteiro para receber temperatura (°C).
 * @param pressure Ponteiro para receber pressão (Pa).
 * @return ESP_OK se OK, erro se falhar.
 */
esp_err_t dps310_read(float *temperature, float *pressure);
#endif