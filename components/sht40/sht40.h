/*
    SHT40 Header - Igor Jensen - UFES - LAEEC
*/

#pragma once
#ifndef SHT40_H
#define SHT40_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT40_SENSOR_ADDR         0x44  // Endereço padrão do SHT40

// Comandos SHT4x (8 bits)
#define SHT4X_CMD_MEAS_HIGH_PREC  0xFD
#define SHT4X_CMD_MEAS_MED_PREC   0xF6
#define SHT4X_CMD_MEAS_LOW_PREC   0xE0
#define SHT4X_CMD_READ_SERIAL     0x89
#define SHT4X_CMD_SOFT_RESET      0x94

typedef struct {
    float temperature;
    float humidity;
} sht40_reading_t;

esp_err_t sht40_init(i2c_master_bus_handle_t bus_handle);
esp_err_t sht40_read_data(sht40_reading_t *reading);
esp_err_t sht40_read_serial(uint32_t *serial);

#ifdef __cplusplus
}
#endif

#endif