#include "as7341.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Tags para logs
static const char *TAG = "AS7341";

// Definições de registradores (do datasheet)
#define AS7341_REG_ENABLE       0x80
#define AS7341_REG_ATIME        0x81
#define AS7341_REG_ASTEP_L      0xCA
#define AS7341_REG_ASTEP_H      0xCB
#define AS7341_REG_CFG1         0xAA
#define AS7341_REG_CFG0         0xA9
#define AS7341_REG_CFG8         0xB1
#define AS7341_REG_CFG12        0xB5
#define AS7341_REG_STATUS       0x93
#define AS7341_REG_CH0_DATA_L   0x95
#define AS7341_REG_CH0_DATA_H   0x96
#define AS7341_REG_CH1_DATA_L   0x97
#define AS7341_REG_CH1_DATA_H   0x98
#define AS7341_REG_CH2_DATA_L   0x99
#define AS7341_REG_CH2_DATA_H   0x9A
#define AS7341_REG_CH3_DATA_L   0x9B
#define AS7341_REG_CH3_DATA_H   0x9C
#define AS7341_REG_CH4_DATA_L   0x9D
#define AS7341_REG_CH4_DATA_H   0x9E
#define AS7341_REG_CH5_DATA_L   0x9F
#define AS7341_REG_CH5_DATA_H   0xA0
#define AS7341_REG_SMUX_CFG_0   0x00
#define AS7341_REG_CONTROL      0xFA

// Bits do registrador ENABLE
#define AS7341_ENABLE_PON       (1 << 0)
#define AS7341_ENABLE_SP_EN     (1 << 1)
#define AS7341_ENABLE_SMUX_EN   (1 << 4)

// Bits do registrador STATUS
#define AS7341_STATUS_ASPECTR_INT (1 << 7)

// Bits do registrador CONTROL
#define AS7341_CONTROL_SMUX_CMD  (1 << 3)
#define AS7341_CONTROL_ADC_INIT  (1 << 0)

// Definições SMUX
#define AS7341_PHOTODIODE_NONE   0x00
#define AS7341_PHOTODIODE_F1     0x01
#define AS7341_PHOTODIODE_F2     0x20
#define AS7341_PHOTODIODE_F3     0x30
#define AS7341_PHOTODIODE_F4     0x50
#define AS7341_PHOTODIODE_F5     0x03
#define AS7341_PHOTODIODE_F6     0x35
#define AS7341_PHOTODIODE_F7     0x51
#define AS7341_PHOTODIODE_F8     0x42
#define AS7341_PHOTODIODE_CLEAR  0x04
#define AS7341_PHOTODIODE_NIR    0x06

// Handle do dispositivo I2C
static i2c_master_dev_handle_t as7341_handle = NULL;

// Funções auxiliares
static esp_err_t as7341_write_reg(uint8_t reg, uint8_t value);
static esp_err_t as7341_read_reg(uint8_t reg, uint8_t *value);
static esp_err_t as7341_read_reg16(uint8_t reg, uint16_t *value);
static esp_err_t as7341_enable_features(bool pon, bool sp_en, bool smux_en);
static esp_err_t as7341_apply_smux_config(const uint8_t *config, size_t size);
static esp_err_t as7341_read_spectral_data_6ch(as7341_spectral_data_t *data, bool first_set);

esp_err_t as7341_init(i2c_master_bus_handle_t i2c_bus) {
    if (as7341_handle != NULL) {
        ESP_LOGE(TAG, "Sensor já inicializado");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS7341_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &as7341_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verificar comunicação lendo o ID do chip
    uint8_t chip_id;
    ret = as7341_read_reg(0x92, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler ID do chip");
        goto error;
    }

    if (chip_id != 0x24) {
        ESP_LOGE(TAG, "ID do chip incorreto. Esperado 0x24, obtido 0x%02X", chip_id);
        ret = ESP_FAIL;
        goto error;
    }

    // Configuração inicial
    ret = as7341_enable_features(true, false, false); // Power on
    if (ret != ESP_OK) goto error;
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = as7341_set_gain(AS7341_GAIN_64X);
    if (ret != ESP_OK) goto error;

    ret = as7341_set_integration_time(50); // ~50ms
    if (ret != ESP_OK) goto error;

    ESP_LOGI(TAG, "AS7341 inicializado com sucesso");
    return ESP_OK;

error:
    as7341_deinit();
    return ret;
}

esp_err_t as7341_deinit(void) {
    if (as7341_handle != NULL) {
        esp_err_t ret = i2c_master_bus_rm_device(as7341_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao remover dispositivo I2C: %s", esp_err_to_name(ret));
            return ret;
        }
        as7341_handle = NULL;
    }
    return ESP_OK;
}

esp_err_t as7341_read_all_channels(as7341_spectral_data_t *data) {
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    // Primeira leitura: F1-F4, NIR, Clear
    esp_err_t ret = as7341_read_spectral_data_6ch(data, true);
    if (ret != ESP_OK) return ret;

    // Segunda leitura: F5-F8, NIR, Clear
    return as7341_read_spectral_data_6ch(data, false);
}

esp_err_t as7341_set_integration_time(uint8_t atime) {
    return as7341_write_reg(AS7341_REG_ATIME, atime);
}

esp_err_t as7341_set_gain(as7341_gain_t gain) {
    if (gain > AS7341_GAIN_512X) gain = AS7341_GAIN_512X;
    return as7341_write_reg(AS7341_REG_CFG1, (uint8_t)gain);
}

esp_err_t as7341_set_low_power_mode(bool enable) {
    uint8_t cfg0;
    esp_err_t ret = as7341_read_reg(AS7341_REG_CFG0, &cfg0);
    if (ret != ESP_OK) return ret;

    cfg0 = enable ? (cfg0 | 0x10) : (cfg0 & ~0x10);
    return as7341_write_reg(AS7341_REG_CFG0, cfg0);
}

// Implementações das funções auxiliares
static esp_err_t as7341_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(as7341_handle, buf, sizeof(buf), 1000 / portTICK_PERIOD_MS);
}

static esp_err_t as7341_read_reg(uint8_t reg, uint8_t *value) {
    return i2c_master_transmit_receive(as7341_handle, &reg, 1, value, 1, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t as7341_read_reg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(as7341_handle, &reg, 1, buf, sizeof(buf), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        *value = (buf[1] << 8) | buf[0];
    }
    return ret;
}

static esp_err_t as7341_enable_features(bool pon, bool sp_en, bool smux_en) {
    uint8_t value = 0;
    if (pon) value |= AS7341_ENABLE_PON;
    if (sp_en) value |= AS7341_ENABLE_SP_EN;
    if (smux_en) value |= AS7341_ENABLE_SMUX_EN;
    return as7341_write_reg(AS7341_REG_ENABLE, value);
}

static esp_err_t as7341_apply_smux_config(const uint8_t *config, size_t size) {
    for (size_t i = 0; i < size; i++) {
        esp_err_t ret = as7341_write_reg(AS7341_REG_SMUX_CFG_0 + i, config[i]);
        if (ret != ESP_OK) return ret;
    }
    return as7341_write_reg(AS7341_REG_CONTROL, AS7341_CONTROL_SMUX_CMD);
}

static esp_err_t as7341_read_spectral_data_6ch(as7341_spectral_data_t *data, bool first_set) {
    static const uint8_t smux_config1[] = {
        AS7341_PHOTODIODE_F1, AS7341_PHOTODIODE_F2, AS7341_PHOTODIODE_F3, AS7341_PHOTODIODE_F4,
        AS7341_PHOTODIODE_NIR, AS7341_PHOTODIODE_CLEAR
    };

    static const uint8_t smux_config2[] = {
        AS7341_PHOTODIODE_F5, AS7341_PHOTODIODE_F6, AS7341_PHOTODIODE_F7, AS7341_PHOTODIODE_F8,
        AS7341_PHOTODIODE_NIR, AS7341_PHOTODIODE_CLEAR
    };

    esp_err_t ret;

    // 1. Configurar SMUX
    ret = as7341_enable_features(true, false, true);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    ret = as7341_apply_smux_config(first_set ? smux_config1 : smux_config2, 6);
    if (ret != ESP_OK) return ret;

    // 2. Iniciar medição
    ret = as7341_enable_features(true, true, true);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    ret = as7341_write_reg(AS7341_REG_CONTROL, AS7341_CONTROL_ADC_INIT);
    if (ret != ESP_OK) return ret;

    // 3. Esperar conclusão
    uint8_t status;
    uint32_t timeout = 100; // ms
    while (timeout--) {
        ret = as7341_read_reg(AS7341_REG_STATUS, &status);
        if (ret != ESP_OK) break;
        if (status & AS7341_STATUS_ASPECTR_INT) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!(status & AS7341_STATUS_ASPECTR_INT)) {
        return ESP_ERR_TIMEOUT;
    }

    // 4. Ler dados
    if (first_set) {
        ret = as7341_read_reg16(AS7341_REG_CH0_DATA_L, &data->f1);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH1_DATA_L, &data->f2);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH2_DATA_L, &data->f3);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH3_DATA_L, &data->f4);
        if (ret != ESP_OK) return ret;
    } else {
        ret = as7341_read_reg16(AS7341_REG_CH0_DATA_L, &data->f5);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH1_DATA_L, &data->f6);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH2_DATA_L, &data->f7);
        if (ret != ESP_OK) return ret;
        ret = as7341_read_reg16(AS7341_REG_CH3_DATA_L, &data->f8);
        if (ret != ESP_OK) return ret;
    }

    // Ler NIR e Clear em ambos os conjuntos
    ret = as7341_read_reg16(AS7341_REG_CH4_DATA_L, &data->nir);
    if (ret != ESP_OK) return ret;
    ret = as7341_read_reg16(AS7341_REG_CH5_DATA_L, &data->clear);
    
    // 5. Desativar medição
    as7341_enable_features(true, false, false);
    return ret;
}