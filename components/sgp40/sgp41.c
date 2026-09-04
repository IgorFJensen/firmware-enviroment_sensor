/*
 * SGP41 driver - VOC + NOx
 * Ajustes desta versao:
 *   - device I2C em 100 kHz para aumentar margem eletrica
 *   - retry curto em transacoes
 *   - preserva historico adaptativo do Gas Index Algorithm
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

#define SGP41_ADDR             0x59
#define SGP41_I2C_HZ           100000
#define I2C_TIMEOUT_MS         1000
#define SGP41_RETRY_COUNT      3
#define SGP41_RETRY_DELAY_MS   5

static uint8_t sgp_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t sgp41_command_read_retry(
    const uint8_t *cmd,
    size_t cmd_len,
    uint8_t *rx,
    size_t rx_len,
    uint32_t conversion_ms,
    const char *operation)
{
    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 1; attempt <= SGP41_RETRY_COUNT; ++attempt) {
        last_err = i2c_master_transmit(
            sgp41_handle,
            cmd,
            cmd_len,
            I2C_TIMEOUT_MS);

        if (last_err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(conversion_ms));
            last_err = i2c_master_receive(
                sgp41_handle,
                rx,
                rx_len,
                I2C_TIMEOUT_MS);
        }

        if (last_err == ESP_OK) {
            return ESP_OK;
        }

        if (attempt < SGP41_RETRY_COUNT) {
            ESP_LOGW(TAG,
                     "%s falhou (%s), retry %d/%d",
                     operation,
                     esp_err_to_name(last_err),
                     attempt + 1,
                     SGP41_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(SGP41_RETRY_DELAY_MS));
        }
    }

    ESP_LOGE(TAG,
             "%s falhou apos %d tentativas: %s",
             operation,
             SGP41_RETRY_COUNT,
             esp_err_to_name(last_err));
    return last_err;
}

static esp_err_t sgp41_init_algorithms(void)
{
    if (algorithms_initialized) {
        ESP_LOGI(TAG, "Preservando historico adaptativo VOC/NOx.");
        return ESP_OK;
    }

    GasIndexAlgorithm_init(&voc_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&nox_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    algorithms_initialized = true;

    ESP_LOGI(TAG, "Sensirion Gas Index Algorithms inicializados (periodo nominal 1 s).");
    return ESP_OK;
}

esp_err_t sgp41_init(i2c_master_bus_handle_t bus)
{
    if (sgp41_handle != NULL) {
        i2c_master_bus_rm_device(sgp41_handle);
        sgp41_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP41_ADDR,
        .scl_speed_hz = SGP41_I2C_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &sgp41_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha adicionando SGP41 ao I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = sgp41_init_algorithms();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "SGP41 registrado em %d Hz", SGP41_I2C_HZ);
    return ESP_OK;
}

esp_err_t sgp41_self_test(uint16_t *test_result)
{
    if (test_result == NULL || sgp41_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t cmd[2] = {0x28, 0x0E};
    uint8_t rx_buf[3];

    esp_err_t err = sgp41_command_read_retry(
        cmd, sizeof(cmd), rx_buf, sizeof(rx_buf), 320, "SelfTest");
    if (err != ESP_OK) {
        return err;
    }

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    *test_result = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp41_execute_conditioning(
    float humidity_percent,
    float temperature_celsius,
    uint16_t *voc_raw)
{
    if (voc_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sgp41_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rh_val = (uint16_t)(humidity_percent * 65535.0f / 100.0f);
    uint16_t t_val = (uint16_t)((temperature_celsius + 45.0f) * 65535.0f / 175.0f);

    uint8_t cmd[8];
    cmd[0] = 0x26;
    cmd[1] = 0x12;
    cmd[2] = (uint8_t)(rh_val >> 8);
    cmd[3] = (uint8_t)(rh_val & 0xFF);
    cmd[4] = sgp_crc8(&cmd[2], 2);
    cmd[5] = (uint8_t)(t_val >> 8);
    cmd[6] = (uint8_t)(t_val & 0xFF);
    cmd[7] = sgp_crc8(&cmd[5], 2);

    uint8_t rx_buf[3];
    esp_err_t err = sgp41_command_read_retry(
        cmd, sizeof(cmd), rx_buf, sizeof(rx_buf), 50, "ExecuteConditioning");
    if (err != ESP_OK) {
        return err;
    }

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CRC invalido no ExecuteConditioning");
        return ESP_ERR_INVALID_CRC;
    }

    *voc_raw = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp41_measure_raw(
    float humidity_percent,
    float temperature_celsius,
    uint16_t *voc_raw,
    uint16_t *nox_raw)
{
    if (voc_raw == NULL || nox_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sgp41_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rh_val = (uint16_t)(humidity_percent * 65535.0f / 100.0f);
    uint16_t t_val = (uint16_t)((temperature_celsius + 45.0f) * 65535.0f / 175.0f);

    uint8_t cmd[8];
    cmd[0] = 0x26;
    cmd[1] = 0x19;
    cmd[2] = (uint8_t)(rh_val >> 8);
    cmd[3] = (uint8_t)(rh_val & 0xFF);
    cmd[4] = sgp_crc8(&cmd[2], 2);
    cmd[5] = (uint8_t)(t_val >> 8);
    cmd[6] = (uint8_t)(t_val & 0xFF);
    cmd[7] = sgp_crc8(&cmd[5], 2);

    uint8_t rx_buf[6];
    esp_err_t err = sgp41_command_read_retry(
        cmd, sizeof(cmd), rx_buf, sizeof(rx_buf), 50, "MeasureRawSignals");
    if (err != ESP_OK) {
        return err;
    }

    if (sgp_crc8(&rx_buf[0], 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CRC invalido para raw VOC");
        return ESP_ERR_INVALID_CRC;
    }
    if (sgp_crc8(&rx_buf[3], 2) != rx_buf[5]) {
        ESP_LOGE(TAG, "CRC invalido para raw NOx");
        return ESP_ERR_INVALID_CRC;
    }

    *voc_raw = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    *nox_raw = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
    return ESP_OK;
}

esp_err_t sgp41_get_indices(
    float humidity_percent,
    float temperature_celsius,
    int32_t *voc_index,
    int32_t *nox_index)
{
    if (voc_index == NULL || nox_index == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!algorithms_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t voc_raw = 0;
    uint16_t nox_raw = 0;
    esp_err_t err = sgp41_measure_raw(
        humidity_percent,
        temperature_celsius,
        &voc_raw,
        &nox_raw);

    if (err != ESP_OK) {
        return err;
    }

    GasIndexAlgorithm_process(&voc_algorithm_params, (int32_t)voc_raw, voc_index);
    GasIndexAlgorithm_process(&nox_algorithm_params, (int32_t)nox_raw, nox_index);
    return ESP_OK;
}
