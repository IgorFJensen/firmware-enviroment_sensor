#ifndef SHTC1_H
#define SHTC1_H

#include "esp_err.h"
#include "driver/i2c_master.h" 

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Endereço I2C do sensor SHTC1.
 */
#define SHTC1_SENSOR_ADDR       0x70

/**
 * @brief Comandos do sensor SHTC1.
 */
#define SHTC1_CMD_MEAS_T_H_LP   0x609C // Low Power mode, clock stretching enabled, Temp first
#define SHTC1_CMD_MEAS_T_H_HP   0x7866 // High Power mode, clock stretching enabled, Temp first
#define SHTC1_CMD_ID            0xEFC8

/**
 * @brief Estrutura para armazenar as leituras do sensor SHTC1.
 */
typedef struct {
    float temperature;  /**< Temperatura em graus Celsius */
    float humidity;     /**< Umidade relativa em porcentagem */
} shtc1_reading_t;

/**
 * @brief Inicializa o sensor SHTC1.
 *
 * Esta função deve ser chamada após a inicialização do barramento I2C Master.
 * Ela configura o dispositivo SHTC1 no barramento e o deixa pronto para uso.
 *
 * @param bus_handle Handle do barramento I2C Master já inicializado.
 * @return ESP_OK se a inicialização for bem-sucedida, ou um código de erro caso contrário.
 */
esp_err_t shtc1_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Lê a temperatura e umidade do sensor SHTC1.
 *
 * Esta função envia o comando de medição, espera a conversão e lê os dados.
 *
 * @param reading Ponteiro para a estrutura shtc1_reading_t onde as leituras serão armazenadas.
 * @return ESP_OK se a leitura for bem-sucedida, ou um código de erro caso contrário.
 */
esp_err_t shtc1_read_data(shtc1_reading_t *reading);

/**
 * @brief Lê o ID do sensor SHTC1.
 *
 * @param device_id Ponteiro para armazenar o ID do dispositivo.
 * @return ESP_OK se a leitura for bem-sucedida, ou um código de erro caso contrário.
 */
esp_err_t shtc1_read_id(uint16_t *device_id);

#ifdef __cplusplus
}
#endif

#endif // SHTC1_H