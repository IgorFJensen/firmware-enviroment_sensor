/* Based on sgp40.c by Igor Jensen - UFES - LAEEC
    Adapted for SGP41 (VOC + NOx)
    Refactored for permanent power stabilization (GPIO19 Always ON)
*/

#include "sgp41.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensirion_gas.h"

static const char *TAG = "SGP41";
static i2c_master_dev_handle_t sgp41_handle = NULL;

static GasIndexAlgorithmParams voc_algorithm_params;
static GasIndexAlgorithmParams nox_algorithm_params;

static bool algorithms_initialized = false;

#define SGP41_ADDR          0x59
#define I2C_TIMEOUT_MS      1000

static uint8_t sgp_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

static esp_err_t _sgp41_init_algorithms() {
    /* * CRITICAL FIX: Protect math baseline history during Light Sleep wakeups.
     * Do NOT re-run GasIndexAlgorithm_init if it has already been initialized once since power-on.
     */
    if (algorithms_initialized) {
        ESP_LOGI(TAG, "Preserving adaptive VOC/NOx algorithm history across sleep cycles.");
        return ESP_OK;
    }
    
    GasIndexAlgorithm_init(&voc_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&nox_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    algorithms_initialized = true;
    ESP_LOGI(TAG, "Sensirion Gas Index Algorithms initialized for the first time.");
    return ESP_OK;
}

esp_err_t sgp41_init(i2c_master_bus_handle_t bus) {
    // Safely unregister old device handle to prevent ESP-IDF virtual infrastructure collisions
    if (sgp41_handle != NULL) {
        i2c_master_bus_rm_device(sgp41_handle);
        sgp41_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP41_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &sgp41_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar device SGP41 ao bus I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = _sgp41_init_algorithms();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar algoritmos VOC/NOx: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t sgp41_self_test(uint16_t *test_result) {
    if (!test_result) return ESP_ERR_INVALID_ARG;

    uint8_t cmd[2] = {0x28, 0x0E};
    esp_err_t err = i2c_master_transmit(sgp41_handle, cmd, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(320)); // self-test demora ~320ms

    uint8_t rx_buf[3];
    err = i2c_master_receive(sgp41_handle, rx_buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) return ESP_ERR_INVALID_CRC;

    *test_result = (rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp41_execute_conditioning(float humidity_percent, float temperature_celsius, uint16_t *voc_raw) {
    if (!voc_raw) return ESP_ERR_INVALID_ARG;
    if (sgp41_handle == NULL) {
        ESP_LOGE(TAG, "SGP41 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rh_val = (uint16_t)(humidity_percent * 65535.0f / 100.0f);
    uint16_t t_val  = (uint16_t)((temperature_celsius + 45.0f) * 65535.0f / 175.0f);

    uint8_t cmd[8];
    cmd[0] = 0x26; 
    cmd[1] = 0x12; 

    cmd[2] = (rh_val >> 8) & 0xFF;
    cmd[3] = rh_val & 0xFF;
    cmd[4] = sgp_crc8(&cmd[2], 2);

    cmd[5] = (t_val >> 8) & 0xFF;
    cmd[6] = t_val & 0xFF;
    cmd[7] = sgp_crc8(&cmd[5], 2);

    esp_err_t err = i2c_master_transmit(sgp41_handle, cmd, 8, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar ExecuteConditioning: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); 

    uint8_t rx_buf[3];
    err = i2c_master_receive(sgp41_handle, rx_buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao receber dados ExecuteConditioning: %s", esp_err_to_name(err));
        return err;
    }

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CRC inválido para SGP41 ExecuteConditioning!");
        return ESP_ERR_INVALID_CRC;
    }

    *voc_raw = (rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp41_measure_raw(float humidity_percent, float temperature_celsius, uint16_t *voc_raw, uint16_t *nox_raw) {
    if (!voc_raw || !nox_raw) return ESP_ERR_INVALID_ARG;
    if (sgp41_handle == NULL) {
        ESP_LOGE(TAG, "SGP41 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rh_val = (uint16_t)(humidity_percent * 65535.0f / 100.0f);
    uint16_t t_val  = (uint16_t)((temperature_celsius + 45.0f) * 65535.0f / 175.0f);

    uint8_t cmd[8];
    cmd[0] = 0x26; 
    cmd[1] = 0x19; 

    cmd[2] = (rh_val >> 8) & 0xFF; 
    cmd[3] = rh_val & 0xFF;        
    cmd[4] = sgp_crc8(&cmd[2], 2); 

    cmd[5] = (t_val >> 8) & 0xFF;  
    cmd[6] = t_val & 0xFF;         
    cmd[7] = sgp_crc8(&cmd[5], 2); 

    esp_err_t err = i2c_master_transmit(sgp41_handle, cmd, 8, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar comando MeasureRawSignals: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); 

    uint8_t rx_buf[6];
    err = i2c_master_receive(sgp41_handle, rx_buf, 6, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao receber dados MeasureRawSignals: %s", esp_err_to_name(err));
        return err;
    }

    if (sgp_crc8(&rx_buf[0], 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CRC inválido para SGP41 raw VOC!");
        return ESP_ERR_INVALID_CRC;
    }
    if (sgp_crc8(&rx_buf[3], 2) != rx_buf[5]) {
        ESP_LOGE(TAG, "CRC inválido para SGP41 raw NOx!");
        return ESP_ERR_INVALID_CRC;
    }

    *voc_raw = (rx_buf[0] << 8) | rx_buf[1];
    *nox_raw = (rx_buf[3] << 8) | rx_buf[4];
    return ESP_OK;
}

esp_err_t sgp41_get_indices(float humidity_percent, float temperature_celsius, int32_t *voc_index, int32_t *nox_index) {
    if (!voc_index || !nox_index) return ESP_ERR_INVALID_ARG;
    if (!algorithms_initialized) {
        ESP_LOGE(TAG, "Algoritmos VOC/NOx não inicializados. Chame sgp41_init() primeiro.");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t voc_raw, nox_raw;
    esp_err_t err = sgp41_measure_raw(humidity_percent, temperature_celsius, &voc_raw, &nox_raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao obter raw VOC/NOx do SGP41: %s", esp_err_to_name(err));
        return err;
    }

    GasIndexAlgorithm_process(&voc_algorithm_params, (int32_t)voc_raw, voc_index);
    GasIndexAlgorithm_process(&nox_algorithm_params, (int32_t)nox_raw, nox_index);

    return ESP_OK;
}