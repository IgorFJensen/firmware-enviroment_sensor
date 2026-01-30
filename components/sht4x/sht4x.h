/*
    Adaptado por Agenor Andre
    Baseado em biblioteca original de Igor Jensen - UFES - LAEEC
*/

#pragma once
#ifndef SHT4X_H
#define SHT4X_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Endereço I2C do sensor SHT4x.
 */
#define SHT4X_SENSOR_ADDR        0x44

/**
 * @brief Comandos do sensor SHT4x.
 */
#define SHT4X_CMD_MEAS_HIGH_PREC 0xFD   // High precision, no heater
#define SHT4X_CMD_MEAS_MED_PREC  0xF6
#define SHT4X_CMD_MEAS_LOW_PREC  0xE0

#define SHT4X_CMD_READ_ID        0x89
#define SHT4X_CMD_SOFT_RESET     0x94

/**
 * @brief Estrutura para armazenar as leituras do sensor SHT4x.
 */
typedef struct {
    float temperature;  /**< Temperatura em graus Celsius */
    float humidity;     /**< Umidade relativa em porcentagem */
} sht4x_reading_t;

/**
 * @brief Inicializa o sensor SHT4x.
 */
esp_err_t sht4x_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Lê temperatura e umidade do sensor SHT4x.
 */
esp_err_t sht4x_read_data(sht4x_reading_t *reading);

/**
 * @brief Lê o ID do sensor SHT4x.
 */
esp_err_t sht4x_read_id(uint32_t *device_id);

#ifdef __cplusplus
}
#endif

#endif // SHT4X_H
