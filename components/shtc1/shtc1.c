/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#include "shtc1.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHTC1";
i2c_master_dev_handle_t shtc1_device_handle = NULL;


//Verifica CRC dos dados recebidos
static uint8_t shtc1_check_crc(uint8_t *data, uint8_t len)
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


//Envia comando 16 bits ao sensor
static esp_err_t shtc1_send_command(uint16_t command)
{
    esp_err_t ret;
    uint8_t cmd_bytes[2];

    cmd_bytes[0] = (command >> 8) & 0xFF;
    cmd_bytes[1] = command & 0xFF;

    if (shtc1_device_handle == NULL) {
        ESP_LOGE(TAG, "Device handle não inicializado");
        return ESP_ERR_INVALID_STATE;
    }

    ret = i2c_master_transmit(shtc1_device_handle, cmd_bytes, sizeof(cmd_bytes), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro enviando comando 0x%04X", command);
    }
    return ret;
}


//Inicializa o sensor e adiciona ao barramento I2C
esp_err_t shtc1_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t ret;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC1_SENSOR_ADDR,
        .scl_speed_hz = 400000,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &shtc1_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo");
        return ret;
    }

    ESP_LOGI(TAG, "Sensor inicializado");

    uint16_t device_id;
    ret = shtc1_read_id(&device_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ID: 0x%04X", device_id);
    }

    return ESP_OK;
}


//Faz leitura de temperatura e umidade
esp_err_t shtc1_read_data(shtc1_reading_t *reading)
{
    esp_err_t ret;
    uint8_t raw_data[6];

    if (!reading)
        return ESP_ERR_INVALID_ARG;

    if (shtc1_device_handle == NULL)
        return ESP_ERR_INVALID_STATE;

    ret = shtc1_send_command(SHTC1_CMD_MEAS_T_H_LP);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(15));

    ret = i2c_master_receive(shtc1_device_handle, raw_data, sizeof(raw_data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro lendo dados");
        return ret;
    }

    if (shtc1_check_crc(&raw_data[0], 2) != raw_data[2])
        return ESP_FAIL;

    if (shtc1_check_crc(&raw_data[3], 2) != raw_data[5])
        return ESP_FAIL;

    uint16_t raw_temperature = (raw_data[0] << 8) | raw_data[1];
    uint16_t raw_humidity = (raw_data[3] << 8) | raw_data[4];

    reading->temperature = -45.0 + 175.0 * ((float)raw_temperature / 65535.0);
    reading->humidity = 100.0 * ((float)raw_humidity / 65535.0);

    return ESP_OK;
}


//Lê ID interno do sensor
esp_err_t shtc1_read_id(uint16_t *device_id)
{
    esp_err_t ret;
    uint8_t raw_id[3];

    if (!device_id)
        return ESP_ERR_INVALID_ARG;

    if (shtc1_device_handle == NULL)
        return ESP_ERR_INVALID_STATE;

    ret = shtc1_send_command(SHTC1_CMD_ID);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(1));

    ret = i2c_master_receive(shtc1_device_handle, raw_id, sizeof(raw_id), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro lendo ID");
        return ret;
    }

    if (shtc1_check_crc(&raw_id[0], 2) != raw_id[2])
        return ESP_FAIL;

    *device_id = (raw_id[0] << 8) | raw_id[1];

    return ESP_OK;
}
