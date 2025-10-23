#include "sgp40.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensirion_gas.h"

static const char *TAG = "SGP40";
static i2c_master_dev_handle_t sgp40_handle;

static GasIndexAlgorithmParams voc_algorithm_params;
static bool voc_algorithm_initialized = false; // Flag para garantir inicialização única

#define SGP40_ADDR          0x59
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

static esp_err_t _sgp40_init_voc_algorithm() { // Renomeada para ser estática e interna
    if (voc_algorithm_initialized) {
        ESP_LOGW(TAG, "Algoritmo VOC Index já inicializado internamente.");
        return ESP_OK;
    }
    GasIndexAlgorithm_init(&voc_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    voc_algorithm_initialized = true;
    ESP_LOGI(TAG, "Algoritmo VOC Index da Sensirion inicializado internamente.");
    return ESP_OK;
}

esp_err_t sgp40_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP40_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &sgp40_handle);
    err = _sgp40_init_voc_algorithm();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar algoritmo VOC: %s", esp_err_to_name(err));
        return err; // Pode retornar erro ou apenas logar, dependendo da criticidade
    }
    return i2c_master_bus_add_device(bus, &dev_cfg, &sgp40_handle);
    
}

esp_err_t sgp40_read_raw(uint16_t *voc_raw) {
    if (!voc_raw) return ESP_ERR_INVALID_ARG;

    uint8_t cmd[8] = {
        0x26, 0x0F,
        0x66, 0x66, 0x00,  // Umidade 0x6666 + CRC preenchido abaixo
        0x66, 0x66, 0x00   // Temperatura 0x6666 + CRC preenchido abaixo
    };
    cmd[4] = sgp_crc8(&cmd[2], 2);
    cmd[7] = sgp_crc8(&cmd[5], 2);

    esp_err_t err = i2c_master_transmit(sgp40_handle, cmd, 8, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(30)); // tempo de medição

    uint8_t rx_buf[3];
    err = i2c_master_receive(sgp40_handle, rx_buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) return ESP_ERR_INVALID_CRC;

    *voc_raw = (rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp40_measure_compensated(float humidity_percent, float temperature_celsius, uint16_t *voc_raw) {
    if (!voc_raw) return ESP_ERR_INVALID_ARG;
    if (sgp40_handle == NULL) {
        ESP_LOGE(TAG, "SGP40 device handle não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    // Converta umidade e temperatura para o formato esperado pelo SGP40
    // Umidade: [pct_rh * 100]
    // Temperatura: [deg_c * 200]
    uint16_t rh_val = (uint16_t)(humidity_percent * 100.0);
    uint16_t t_val = (uint16_t)(temperature_celsius * 200.0);

    uint8_t cmd[8];
    cmd[0] = 0x26; // MeasureRaw command MSB
    cmd[1] = 0x0F; // MeasureRaw command LSB

    cmd[2] = (rh_val >> 8) & 0xFF; // RH MSB
    cmd[3] = rh_val & 0xFF;        // RH LSB
    cmd[4] = sgp_crc8(&cmd[2], 2); // CRC para umidade

    cmd[5] = (t_val >> 8) & 0xFF;  // T MSB
    cmd[6] = t_val & 0xFF;         // T LSB
    cmd[7] = sgp_crc8(&cmd[5], 2); // CRC para temperatura

    esp_err_t err = i2c_master_transmit(sgp40_handle, cmd, 8, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar comando MeasureRaw (compensado): %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); // Tempo de medição do SGP40

    uint8_t rx_buf[3];
    err = i2c_master_receive(sgp40_handle, rx_buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao receber dados MeasureRaw (compensado): %s", esp_err_to_name(err));
        return err;
    }

    if (sgp_crc8(rx_buf, 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CRC inválido para SGP40 raw VOC (compensado)!");
        return ESP_ERR_INVALID_CRC;
    }

    *voc_raw = (rx_buf[0] << 8) | rx_buf[1];
    return ESP_OK;
}

esp_err_t sgp40_get_voc_index(float humidity_percent, float temperature_celsius, int32_t *voc_index) {
    if (!voc_index) return ESP_ERR_INVALID_ARG;
    if (!voc_algorithm_initialized) {
        ESP_LOGE(TAG, "Algoritmo VOC Index não inicializado. Chame sgp40_init() para inicializar.");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_voc_compensated;
    esp_err_t err;

    err = sgp40_measure_compensated(humidity_percent, temperature_celsius, &raw_voc_compensated);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao obter raw VOC compensado do SGP40: %s", esp_err_to_name(err));
        return err;
    }

    GasIndexAlgorithm_process(&voc_algorithm_params, (int32_t)raw_voc_compensated, voc_index);

    return ESP_OK;
}