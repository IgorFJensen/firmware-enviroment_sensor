/*
    Adaptado por Agenor Andre
    Baseado em biblioteca original de Igor Jensen - UFES - LAEEC
*/

#include "sht4x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHT4X";
static i2c_master_dev_handle_t sht4x_device_handle = NULL;

/**
 * @brief Verifica CRC (polinômio 0x31)
 */
static uint8_t sht4x_check_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t poly = 0x31;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ poly) : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Envia comando de 8 bits ao sensor
 */
static esp_err_t sht4x_send_command(uint8_t command)
{
    if (sht4x_device_handle == NULL) {
        ESP_LOGE(TAG, "Device handle não inicializado");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_master_transmit(
        sht4x_device_handle,
        &command,
        1,
        -1
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro enviando comando 0x%02X", command);
    }

    return ret;
}

/**
 * @brief Inicializa o sensor
 */
esp_err_t sht4x_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT4X_SENSOR_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(
        bus_handle,
        &dev_config,
        &sht4x_device_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo SHT4x");
        return ret;
    }

    ESP_LOGI(TAG, "Sensor SHT4x inicializado");

    uint32_t device_id;
    if (sht4x_read_id(&device_id) == ESP_OK) {
        ESP_LOGI(TAG, "ID: 0x%08lX", device_id);
    }

    return ESP_OK;
}

/**
 * @brief Leitura de temperatura e umidade
 */
esp_err_t sht4x_read_data(sht4x_reading_t *reading)
{
    if (!reading)
        return ESP_ERR_INVALID_ARG;

    if (sht4x_device_handle == NULL)
        return ESP_ERR_INVALID_STATE;

    esp_err_t ret = sht4x_send_command(SHT4X_CMD_MEAS_HIGH_PREC);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(25));

    uint8_t raw_data[6];
    ret = i2c_master_receive(
        sht4x_device_handle,
        raw_data,
        sizeof(raw_data),
        -1
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro lendo dados");
        return ret;
    }

    if (sht4x_check_crc(&raw_data[0], 2) != raw_data[2])
        return ESP_FAIL;

    if (sht4x_check_crc(&raw_data[3], 2) != raw_data[5])
        return ESP_FAIL;

    uint16_t raw_temp = (raw_data[0] << 8) | raw_data[1];
    uint16_t raw_hum  = (raw_data[3] << 8) | raw_data[4];

    reading->temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    reading->humidity    = 100.0f * ((float)raw_hum  / 65535.0f);

    return ESP_OK;
}

/**
 * @brief Leitura do ID do sensor
 */
esp_err_t sht4x_read_id(uint32_t *device_id)
{
    if (!device_id)
        return ESP_ERR_INVALID_ARG;

    if (sht4x_device_handle == NULL)
        return ESP_ERR_INVALID_STATE;

    esp_err_t ret = sht4x_send_command(SHT4X_CMD_READ_ID);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t raw_id[6];
    ret = i2c_master_receive(
        sht4x_device_handle,
        raw_id,
        sizeof(raw_id),
        -1
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro lendo ID");
        return ret;
    }

    if (sht4x_check_crc(&raw_id[0], 2) != raw_id[2])
        return ESP_FAIL;

    if (sht4x_check_crc(&raw_id[3], 2) != raw_id[5])
        return ESP_FAIL;

    *device_id =
        ((uint32_t)raw_id[0] << 24) |
        ((uint32_t)raw_id[1] << 16) |
        ((uint32_t)raw_id[3] << 8)  |
        raw_id[4];

    return ESP_OK;
}
