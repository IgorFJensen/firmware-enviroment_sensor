/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#pragma once
#ifndef VEML7700_H
#define VEML7700_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Endereço I2C do VEML7700
#define VEML7700_I2C_ADDR 0x10

// Endereços dos registradores do VEML7700
#define VEML7700_ALS_CONF_REG         0x00 // Configuração do ALS
#define VEML7700_ALS_WH_REG           0x01 // Threshold alto
#define VEML7700_ALS_LL_REG           0x02 // Threshold baixo
#define VEML7700_PSM_REG              0x03 // Modo de economia
#define VEML7700_ALS_DATA_REG         0x04 // Dados ALS
#define VEML7700_WHITE_DATA_REG       0x05 // Canal branco
#define VEML7700_INTERRUPT_STATUS_REG 0x06 // Status de interrupção

// Ganho do ALS (bits 12:11)
typedef enum {
    VEML7700_GAIN_X1_8 = 0x00, // Ganho 1/8
    VEML7700_GAIN_X1_4 = 0x01, // Ganho 1/4
    VEML7700_GAIN_X1   = 0x02, // Ganho 1 (default)
    VEML7700_GAIN_X2   = 0x03  // Ganho 2
} veml7700_gain_t;

// Tempo de integração (bits 9:6)
typedef enum {
    VEML7700_IT_100MS = 0x00, // 100ms (padrão)
    VEML7700_IT_200MS = 0x01,
    VEML7700_IT_400MS = 0x02,
    VEML7700_IT_800MS = 0x03,
    VEML7700_IT_50MS  = 0x08,
    VEML7700_IT_25MS  = 0x09,
} veml7700_it_t;

// Habilita interrupção (bit 1)
typedef enum {
    VEML7700_INTERRUPT_DISABLE = 0x00,
    VEML7700_INTERRUPT_ENABLE  = 0x01
} veml7700_interrupt_enable_t;

// Liga/desliga ALS (bit 0)
typedef enum {
    VEML7700_POWER_ON  = 0x00, // Ativo
    VEML7700_POWER_OFF = 0x01  // Desligado
} veml7700_power_state_t;


// Modo de economia de energia (bits 2:1)
typedef enum {
    VEML7700_PSM_MODE1 = 0x00, // Padrão
    VEML7700_PSM_MODE2 = 0x01,
    VEML7700_PSM_MODE3 = 0x02,
    VEML7700_PSM_MODE4 = 0x03  // Menor consumo
} veml7700_power_save_mode_t;

// Enable do modo economia (bit 0)
typedef enum {
    VEML7700_PSM_DISABLE = 0x00,
    VEML7700_PSM_ENABLE  = 0x01
} veml7700_power_save_enable_t;

// Flags do registrador de interrupção
#define VEML7700_INT_LOW_THRESHOLD_FLAG  (1 << 15)
#define VEML7700_INT_HIGH_THRESHOLD_FLAG (1 << 14)


/**
 * @brief Inicializa o sensor VEML7700.
 *
 * @param i2c_handle Handle do barramento I2C já configurado.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t veml7700_init(i2c_master_bus_handle_t i2c_handle);

/**
 * @brief Configura o threshold alto de interrupção.
 */
esp_err_t veml7700_set_high_threshold(uint16_t threshold);

/**
 * @brief Configura o threshold baixo de interrupção.
 */
esp_err_t veml7700_set_low_threshold(uint16_t threshold);

/**
 * @brief Ajusta o modo de economia de energia.
 */
esp_err_t veml7700_set_power_save_mode(veml7700_power_save_mode_t mode);

/**
 * @brief Habilita/desabilita o modo de economia.
 */
esp_err_t veml7700_enable_power_save(veml7700_power_save_enable_t enable);

/**
 * @brief Lê o valor bruto do ALS.
 */
esp_err_t veml7700_read_als(uint16_t *als_data);

/**
 * @brief Lê o canal branco.
 */
esp_err_t veml7700_read_white(uint16_t *white_data);

/**
 * @brief Lê o registrador de status de interrupção.
 */
esp_err_t veml7700_get_interrupt_status(uint16_t *status);

/**
 * @brief Converte o valor ALS bruto em Lux.
 *
 * @param als_raw Valor 16 bits lido do sensor.
 * @return Lux equivalente.
 */
float veml7700_raw_to_lux(uint16_t als_raw);

/**
 * @brief Leitura genérica de registrador 16 bits.
 */
esp_err_t veml7700_read_reg(uint8_t reg_addr, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif // VEML7700_H
