#include "shtc1.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHTC1_DRIVER";
i2c_master_dev_handle_t shtc1_device_handle = NULL; // Handle estático para o dispositivo SHTC1

// A função de verificação CRC permanece a mesma
static uint8_t shtc1_check_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t poly = 0x31; // CRC-8 polynomial

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// A função shtc1_send_command agora usa o handle estático
static esp_err_t shtc1_send_command(uint16_t command)
{
    esp_err_t ret;
    uint8_t cmd_bytes[2];
    cmd_bytes[0] = (command >> 8) & 0xFF; // MSB
    cmd_bytes[1] = command & 0xFF;     // LSB

    if (shtc1_device_handle == NULL) {
        ESP_LOGE(TAG, "SHTC1 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    ret = i2c_master_transmit(shtc1_device_handle, cmd_bytes, sizeof(cmd_bytes), -1); // Timeout infinito
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar comando 0x%04X: %s", command, esp_err_to_name(ret));
    }
    return ret;
}

// ESTA É A ÚNICA DEFINIÇÃO DE shtc1_init QUE DEVE ESTAR PRESENTE
esp_err_t shtc1_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t ret;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC1_SENSOR_ADDR,
        .scl_speed_hz = 100000, // 100 KHz
    };
    
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &shtc1_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo I2C SHTC1: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Sensor SHTC1 inicializado e dispositivo I2C adicionado.");


    uint16_t device_id;
    ret = shtc1_read_id(&device_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ID do sensor SHTC1: 0x%04X", device_id);
    } else {
        ESP_LOGW(TAG, "Não foi possível ler o ID do sensor SHTC1.");
    }
    
    return ESP_OK;
}

esp_err_t shtc1_read_data(shtc1_reading_t *reading)
{
    esp_err_t ret;
    uint8_t raw_data[6]; // 2 bytes temp, 1 byte CRC, 2 bytes hum, 1 byte CRC

    if (!reading) {
        return ESP_ERR_INVALID_ARG;
    }
    if (shtc1_device_handle == NULL) {
        ESP_LOGE(TAG, "SHTC1 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    // Envia o comando de medição (Low Power)
    ret = shtc1_send_command(SHTC1_CMD_MEAS_T_H_LP);
    if (ret != ESP_OK) {
        return ret;
    }

    // O SHTC1 leva cerca de 13.5ms (Low Power) ou 6.5ms (High Power) para converter.
    // Usamos um atraso um pouco maior para garantir.
    vTaskDelay(pdMS_TO_TICKS(15));

    // A transação agora é com i2c_master_receive
    ret = i2c_master_receive(shtc1_device_handle, raw_data, sizeof(raw_data), -1); // Timeout infinito
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler dados do sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verifica o CRC da temperatura
    if (shtc1_check_crc(&raw_data[0], 2) != raw_data[2]) {
        ESP_LOGE(TAG, "CRC da temperatura inválido!");
        return ESP_FAIL;
    }

    // Verifica o CRC da umidade
    if (shtc1_check_crc(&raw_data[3], 2) != raw_data[5]) {
        ESP_LOGE(TAG, "CRC da umidade inválido!");
        return ESP_FAIL;
    }

    // Converte os dados brutos para temperatura e umidade
    uint16_t raw_temperature = (raw_data[0] << 8) | raw_data[1];
    uint16_t raw_humidity = (raw_data[3] << 8) | raw_data[4];

    // Fórmula de conversão do SHTC1 (ver datasheet)
    reading->temperature = -45.0 + 175.0 * ((float)raw_temperature / 65535.0);
    reading->humidity = 100.0 * ((float)raw_humidity / 65535.0);

    return ESP_OK;
}

esp_err_t shtc1_read_id(uint16_t *device_id)
{
    esp_err_t ret;
    uint8_t raw_id[3]; // 2 bytes ID, 1 byte CRC

    if (!device_id) {
        return ESP_ERR_INVALID_ARG;
    }
    if (shtc1_device_handle == NULL) {
        ESP_LOGE(TAG, "SHTC1 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    ret = shtc1_send_command(SHTC1_CMD_ID);
    if (ret != ESP_OK) {
        return ret;
    }

    // Pequeno atraso para o sensor processar o comando
    vTaskDelay(pdMS_TO_TICKS(1));

    // A transação agora é com i2c_master_receive
    ret = i2c_master_receive(shtc1_device_handle, raw_id, sizeof(raw_id), -1); // Timeout infinito
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler ID do sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verifica o CRC do ID
    if (shtc1_check_crc(&raw_id[0], 2) != raw_id[2]) {
        ESP_LOGE(TAG, "CRC do ID inválido!");
        return ESP_FAIL;
    }

    *device_id = (raw_id[0] << 8) | raw_id[1];

    return ESP_OK;
}
