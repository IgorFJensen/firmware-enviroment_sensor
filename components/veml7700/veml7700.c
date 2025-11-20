/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
*/

#include <stdio.h>
#include <string.h>
#include "veml7700.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VEML7700";

// Handles internos do I2C
static i2c_master_bus_handle_t veml7700_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t veml7700_i2c_dev_handle = NULL;

// Configurações atuais do sensor (usadas na conversão para Lux)
static veml7700_gain_t current_gain = VEML7700_GAIN_X1;
static veml7700_it_t current_it = VEML7700_IT_100MS;
static veml7700_interrupt_enable_t current_int_en = VEML7700_INTERRUPT_DISABLE;
static veml7700_power_state_t current_power_state = VEML7700_POWER_OFF;
static veml7700_power_save_mode_t current_psm_mode = VEML7700_PSM_MODE1;
static veml7700_power_save_enable_t current_psm_en = VEML7700_PSM_DISABLE;

/* Escreve em registrador 16 bits */
static esp_err_t veml7700_write_reg(uint8_t reg_addr, uint16_t data) {
    if (!veml7700_i2c_dev_handle) {
        ESP_LOGE(TAG, "Handle do sensor não inicializado.");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx_buf[3] = {
        reg_addr,
        (uint8_t)(data & 0xFF),
        (uint8_t)(data >> 8),
    };

    esp_err_t ret = i2c_master_transmit(
        veml7700_i2c_dev_handle,
        tx_buf,
        sizeof(tx_buf),
        1000 / portTICK_PERIOD_MS
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao escrever reg 0x%02X", reg_addr);
    }
    return ret;
}

/* Lê e remonta ALS_CONF se falhar */
static esp_err_t veml7700_read_modify_write_conf_reg(uint16_t *current_val) {
    esp_err_t ret = veml7700_read_reg(VEML7700_ALS_CONF_REG, current_val);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao ler ALS_CONF, reconstruindo valor interno.");
        *current_val = 0;
        *current_val |= (current_it << 6);
        *current_val |= (current_gain << 11);
        *current_val |= (current_int_en << 1);
        *current_val |= (current_power_state << 0);
    }
    return ESP_OK;
}

/* Ajusta ganho */
esp_err_t veml7700_set_gain(veml7700_gain_t gain) {
    uint16_t conf;
    veml7700_read_modify_write_conf_reg(&conf);

    conf &= ~(0x03 << 11);
    conf |= (gain << 11);

    esp_err_t ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf);
    if (ret == ESP_OK) current_gain = gain;
    return ret;
}

/* Ajusta tempo de integração */
esp_err_t veml7700_set_integration_time(veml7700_it_t it) {
    uint16_t conf;
    veml7700_read_modify_write_conf_reg(&conf);

    conf &= ~(0x0F << 6);
    conf |= (it << 6);

    esp_err_t ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf);
    if (ret == ESP_OK) current_it = it;
    return ret;
}

/* Habilita / desabilita interrupção */
esp_err_t veml7700_set_interrupt_enable(veml7700_interrupt_enable_t enable) {
    uint16_t conf;
    veml7700_read_modify_write_conf_reg(&conf);

    if (enable)
        conf |= (1 << 1);
    else
        conf &= ~(1 << 1);

    esp_err_t ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf);
    if (ret == ESP_OK) current_int_en = enable;
    return ret;
}

/* Liga / desliga sensor */
esp_err_t veml7700_set_power_state(veml7700_power_state_t state) {
    uint16_t conf;
    veml7700_read_modify_write_conf_reg(&conf);

    if (state == VEML7700_POWER_OFF)
        conf |= (1 << 0);
    else
        conf &= ~(1 << 0);

    esp_err_t ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf);
    if (ret == ESP_OK) {
        current_power_state = state;
        if (state == VEML7700_POWER_ON)
            vTaskDelay(pdMS_TO_TICKS(5)); // tempo mínimo
    }
    return ret;
}

/* Lê registrador 16 bits */
esp_err_t veml7700_read_reg(uint8_t reg_addr, uint16_t *data) {
    if (!veml7700_i2c_dev_handle) return ESP_ERR_INVALID_STATE;

    uint8_t rx[2];
    esp_err_t ret = i2c_master_transmit_receive(
        veml7700_i2c_dev_handle,
        &reg_addr,
        1,
        rx,
        2,
        1000 / portTICK_PERIOD_MS
    );

    if (ret == ESP_OK) {
        *data = rx[0] | (rx[1] << 8);
    } else {
        ESP_LOGE(TAG, "Erro ao ler reg 0x%02X", reg_addr);
    }
    return ret;
}

/* Inicialização */
esp_err_t veml7700_init(i2c_master_bus_handle_t i2c_handle) {
    if (!i2c_handle) return ESP_ERR_INVALID_ARG;

    veml7700_i2c_bus_handle = i2c_handle;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML7700_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    if (i2c_master_bus_add_device(i2c_handle, &cfg, &veml7700_i2c_dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar device VEML7700.");
        return ESP_FAIL;
    }

    veml7700_set_power_state(VEML7700_POWER_ON);
    vTaskDelay(pdMS_TO_TICKS(5));

    veml7700_set_gain(VEML7700_GAIN_X1);
    veml7700_set_integration_time(VEML7700_IT_50MS);
    veml7700_set_interrupt_enable(VEML7700_INTERRUPT_DISABLE);

    veml7700_enable_power_save(VEML7700_PSM_ENABLE);
    veml7700_set_power_save_mode(VEML7700_PSM_MODE4);

    ESP_LOGI(TAG, "VEML7700 pronto.");
    return ESP_OK;
}

/* Thresholds */
esp_err_t veml7700_set_high_threshold(uint16_t t) {
    return veml7700_write_reg(VEML7700_ALS_WH_REG, t);
}

esp_err_t veml7700_set_low_threshold(uint16_t t) {
    return veml7700_write_reg(VEML7700_ALS_LL_REG, t);
}

/* Ajustes do PSM */
static esp_err_t veml7700_read_modify_write_psm_reg(uint16_t *v) {
    if (veml7700_read_reg(VEML7700_PSM_REG, v) != ESP_OK) {
        *v = (current_psm_mode << 1) | current_psm_en;
    }
    return ESP_OK;
}

esp_err_t veml7700_set_power_save_mode(veml7700_power_save_mode_t mode) {
    uint16_t v;
    veml7700_read_modify_write_psm_reg(&v);

    v &= ~(0x03 << 1);
    v |= mode << 1;

    esp_err_t ret = veml7700_write_reg(VEML7700_PSM_REG, v);
    if (ret == ESP_OK) current_psm_mode = mode;
    return ret;
}

esp_err_t veml7700_enable_power_save(veml7700_power_save_enable_t en) {
    uint16_t v;
    veml7700_read_modify_write_psm_reg(&v);

    if (en) v |= 1;
    else    v &= ~1;

    esp_err_t ret = veml7700_write_reg(VEML7700_PSM_REG, v);
    if (ret == ESP_OK) current_psm_en = en;
    return ret;
}

/* Leituras ALS e White */
esp_err_t veml7700_read_als(uint16_t *d) {
    vTaskDelay(pdMS_TO_TICKS(5));
    return veml7700_read_reg(VEML7700_ALS_DATA_REG, d);
}

esp_err_t veml7700_read_white(uint16_t *d) {
    vTaskDelay(pdMS_TO_TICKS(5));
    return veml7700_read_reg(VEML7700_WHITE_DATA_REG, d);
}

esp_err_t veml7700_get_interrupt_status(uint16_t *s) {
    return veml7700_read_reg(VEML7700_INTERRUPT_STATUS_REG, s);
}

/* Conversão RAW -> Lux */
float veml7700_raw_to_lux(uint16_t raw) {
    float gain = (current_gain == VEML7700_GAIN_X1_8) ? 0.125 :
                 (current_gain == VEML7700_GAIN_X1_4) ? 0.25  :
                 (current_gain == VEML7700_GAIN_X1)   ? 1.0   : 2.0;

    float it = (current_it == VEML7700_IT_25MS) ? 25 :
               (current_it == VEML7700_IT_50MS) ? 50 :
               (current_it == VEML7700_IT_100MS) ? 100 :
               (current_it == VEML7700_IT_200MS) ? 200 :
               (current_it == VEML7700_IT_400MS) ? 400 : 800;

    const float BASE_LSB = 0.00452;
    float res = (BASE_LSB * (800.0 / it)) / gain;

    return raw * res;
}
