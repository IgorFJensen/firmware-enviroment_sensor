#ifndef AS7341_H
#define AS7341_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS7341_I2C_ADDR         0x39

/**
 * @brief Valores de ganho do sensor (datasheet p.37)
 */
typedef enum {
    AS7341_GAIN_0_5X = 0,   ///< Ganho 0.5x
    AS7341_GAIN_1X,          ///< Ganho 1x
    AS7341_GAIN_2X,          ///< Ganho 2x
    AS7341_GAIN_4X,          ///< Ganho 4x
    AS7341_GAIN_8X,          ///< Ganho 8x
    AS7341_GAIN_16X,         ///< Ganho 16x
    AS7341_GAIN_32X,         ///< Ganho 32x
    AS7341_GAIN_64X,         ///< Ganho 64x
    AS7341_GAIN_128X,        ///< Ganho 128x
    AS7341_GAIN_256X,        ///< Ganho 256x
    AS7341_GAIN_512X         ///< Ganho 512x (máximo)
} as7341_gain_t;

/**
 * @brief Estrutura para armazenar dados espectrais
 */
typedef struct {
    uint16_t f1;     ///< Canal F1 (415nm)
    uint16_t f2;     ///< Canal F2 (445nm)
    uint16_t f3;     ///< Canal F3 (480nm)
    uint16_t f4;     ///< Canal F4 (515nm)
    uint16_t f5;     ///< Canal F5 (555nm)
    uint16_t f6;     ///< Canal F6 (590nm)
    uint16_t f7;     ///< Canal F7 (630nm)
    uint16_t f8;     ///< Canal F8 (680nm)
    uint16_t clear;  ///< Canal Clear (sem filtro)
    uint16_t nir;    ///< Canal NIR (Near Infrared)
} as7341_spectral_data_t;

/**
 * @brief Inicializa o sensor AS7341
 * @param i2c_handle Handle do barramento I2C
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_init(i2c_master_bus_handle_t i2c_handle);

/**
 * @brief Desinicializa o sensor AS7341
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_deinit(void);

/**
 * @brief Lê todos os canais espectrais
 * @param data Ponteiro para estrutura de dados espectrais
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_read_all_channels(as7341_spectral_data_t *data);

/**
 * @brief Configura o tempo de integração
 * @param atime Valor do registrador ATIME (0-255)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_set_integration_time(uint8_t atime);

/**
 * @brief Configura o ganho do sensor
 * @param gain Valor de ganho (ver as7341_gain_t)
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_set_gain(as7341_gain_t gain);

/**
 * @brief Habilita/desabilita modo de baixo consumo
 * @param enable true para habilitar
 * @return ESP_OK em caso de sucesso
 */
esp_err_t as7341_set_low_power_mode(bool enable);

#ifdef __cplusplus
}
#endif

#endif // AS7341_H