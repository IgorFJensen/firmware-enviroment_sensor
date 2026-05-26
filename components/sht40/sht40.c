/*
    Refactored for SHT40 - Igor Jensen - UFES - LAEEC
    Date: 2026-05-13
*/

#include "sht40.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHT40";
i2c_master_dev_handle_t sht40_device_handle = NULL;

// CRC-8 para SHT4x: polinômio 0x31 (x8 + x5 + x4 + 1), init 0xFF
static uint8_t sht40_check_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

// Inicializa o sensor SHT40
esp_err_t sht40_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT40_SENSOR_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &sht40_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar SHT40");
        return ret;
    }

    ESP_LOGI(TAG, "SHT40 inicializado no endereco 0x%02X", SHT40_SENSOR_ADDR);
    
    // Teste de leitura de Serial Number para confirmar comunicação
    uint32_t serial;
    if (sht40_read_serial(&serial) == ESP_OK) {
        ESP_LOGI(TAG, "Serial Number: 0x%08X", (unsigned int)serial);
    }

    return ESP_OK;
}

// Lê dados de temperatura e umidade
esp_err_t sht40_read_data(sht40_reading_t *reading)
{
    if (!reading || !sht40_device_handle) return ESP_ERR_INVALID_ARG;

    // SHT4x usa comandos de 1 byte. 0xFD é "High Precision"
    uint8_t cmd = SHT4X_CMD_MEAS_HIGH_PREC;
    uint8_t raw_data[6];

    // Envia comando de medição
    esp_err_t ret = i2c_master_transmit(sht40_device_handle, &cmd, 1, -1);
    if (ret != ESP_OK) return ret;

    // Tempo máximo para conversão High Precision é de 8.2ms
    vTaskDelay(pdMS_TO_TICKS(20));

    // Recebe 6 bytes (T_msb, T_lsb, T_crc, H_msb, H_lsb, H_crc)
    ret = i2c_master_receive(sht40_device_handle, raw_data, 6, -1);
    if (ret != ESP_OK) return ret;

    // Validação de CRC
    if (sht40_check_crc(&raw_data[0], 2) != raw_data[2] || 
        sht40_check_crc(&raw_data[3], 2) != raw_data[5]) {
        ESP_LOGE(TAG, "Erro de CRC na leitura");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t t_ticks = (raw_data[0] << 8) | raw_data[1];
    uint16_t rh_ticks = (raw_data[3] << 8) | raw_data[4];

    // Fórmulas conforme Datasheet SHT4x
    reading->temperature = -45.0f + 175.0f * (float)t_ticks / 65535.0f;
    reading->humidity = -6.0f + 125.0f * (float)rh_ticks / 65535.0f;

    // Clipping de umidade (SHT4x pode ler levemente fora de 0-100% devido a precisão)
    if (reading->humidity > 100.0f) reading->humidity = 100.0f;
    if (reading->humidity < 0.0f) reading->humidity = 0.0f;

    return ESP_OK;
}

// Lê o Serial Number único do sensor
esp_err_t sht40_read_serial(uint32_t *serial)
{
    uint8_t cmd = SHT4X_CMD_READ_SERIAL;
    uint8_t raw[6];
    
    esp_err_t ret = i2c_master_transmit(sht40_device_handle, &cmd, 1, -1);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(1));

    ret = i2c_master_receive(sht40_device_handle, raw, 6, -1);
    if (ret != ESP_OK) return ret;

    if (sht40_check_crc(&raw[0], 2) != raw[2] || sht40_check_crc(&raw[3], 2) != raw[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    *serial = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16) | ((uint32_t)raw[3] << 8) | raw[4];
    return ESP_OK;
}