#include <stdio.h>
#include <string.h>
#include "veml7700.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VEML7700";
static i2c_master_bus_handle_t veml7700_i2c_bus_handle = NULL; // Internal handle
static i2c_master_dev_handle_t veml7700_i2c_dev_handle = NULL; // Device handle for VEML7700

// Current sensor configuration (needed for Lux conversion and modifying registers)
static veml7700_gain_t current_gain = VEML7700_GAIN_X1;
static veml7700_it_t current_it = VEML7700_IT_100MS;
static veml7700_interrupt_enable_t current_int_en = VEML7700_INTERRUPT_DISABLE;
static veml7700_power_state_t current_power_state = VEML7700_POWER_OFF;
static veml7700_power_save_mode_t current_psm_mode = VEML7700_PSM_MODE1;
static veml7700_power_save_enable_t current_psm_en = VEML7700_PSM_DISABLE;

// Helper function to write to a 16-bit register
static esp_err_t veml7700_write_reg(uint8_t reg_addr, uint16_t data) {
    if (veml7700_i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "VEML7700 device handle not initialized. Call veml7700_init first.");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx_buf[3];
    tx_buf[0] = reg_addr;
    tx_buf[1] = (uint8_t)(data & 0xFF);         // Low byte
    tx_buf[2] = (uint8_t)((data >> 8) & 0xFF);  // High byte

    // Corrected i2c_master_transmit usage for ESP-IDF v5.x
    esp_err_t ret = i2c_master_transmit(veml7700_i2c_dev_handle, tx_buf, sizeof(tx_buf), 1000 / portTICK_PERIOD_MS); // Timeout in ticks
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X (data 0x%04X), error: %s", reg_addr, data, esp_err_to_name(ret));
    }
    return ret;
}


// Function to read and update the ALS_CONF_REG value
static esp_err_t veml7700_read_modify_write_conf_reg(uint16_t *current_val) {
    esp_err_t ret = veml7700_read_reg(VEML7700_ALS_CONF_REG, current_val);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ALS_CONF_REG. Reconstructing from internal state.");
        *current_val = 0;
        *current_val |= (uint16_t)current_it << 6; // ALS_IT bits [9:6]
        *current_val |= (uint16_t)current_gain << 11; // ALS_GAIN bits [12:11]
        *current_val |= (uint16_t)current_int_en << 1; // INT_EN bit [1]
        *current_val |= (uint16_t)current_power_state << 0; // SD bit [0]
    }
    return ESP_OK;
}

esp_err_t veml7700_set_gain(veml7700_gain_t gain) {
    uint16_t conf_val;
    esp_err_t ret = veml7700_read_modify_write_conf_reg(&conf_val);
    if (ret != ESP_OK) return ret;

    conf_val &= ~((uint16_t)0x03 << 11); // Clear bits 12 and 11
    conf_val |= (uint16_t)gain << 11;    // Set new gain

    ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf_val);
    if (ret == ESP_OK) {
        current_gain = gain;
    }
    return ret;
}

esp_err_t veml7700_set_integration_time(veml7700_it_t it) {
    uint16_t conf_val;
    esp_err_t ret = veml7700_read_modify_write_conf_reg(&conf_val);
    if (ret != ESP_OK) return ret;

    conf_val &= ~((uint16_t)0x0F << 6); // Clear bits 9, 8, 7, 6
    conf_val |= (uint16_t)it << 6;       // Set new IT

    ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf_val);
    if (ret == ESP_OK) {
        current_it = it;
    }
    return ret;
}

esp_err_t veml7700_set_interrupt_enable(veml7700_interrupt_enable_t enable) {
    uint16_t conf_val;
    esp_err_t ret = veml7700_read_modify_write_conf_reg(&conf_val);
    if (ret != ESP_OK) return ret;

    if (enable == VEML7700_INTERRUPT_ENABLE) {
        conf_val |= (1 << 1); // Set INT_EN bit to 1
    } else {
        conf_val &= ~(1 << 1); // Clear INT_EN bit to 0
    }

    ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf_val);
    if (ret == ESP_OK) {
        current_int_en = enable;
    }
    return ret;
}

esp_err_t veml7700_set_power_state(veml7700_power_state_t state) {
    uint16_t conf_val;
    esp_err_t ret = veml7700_read_modify_write_conf_reg(&conf_val);
    if (ret != ESP_OK) return ret;

    if (state == VEML7700_POWER_OFF) {
        conf_val |= (1 << 0); // Set SD bit to 1 (shutdown)
    } else {
        conf_val &= ~(1 << 0); // Clear SD bit to 0 (active)
    }

    ret = veml7700_write_reg(VEML7700_ALS_CONF_REG, conf_val);
    if (ret == ESP_OK) {
        current_power_state = state;
        if (state == VEML7700_POWER_ON) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    return ret;
}

// Helper function to read from a 16-bit register
esp_err_t veml7700_read_reg(uint8_t reg_addr, uint16_t *data) {
    if (veml7700_i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "VEML7700 device handle not initialized. Call veml7700_init first.");
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rx_buf[2];

    // Corrected i2c_master_transmit_receive usage for ESP-IDF v5.x
    esp_err_t ret = i2c_master_transmit_receive(veml7700_i2c_dev_handle, &reg_addr, 1, rx_buf, sizeof(rx_buf), 1000 / portTICK_PERIOD_MS); // Timeout in ticks
    if (ret == ESP_OK) {
        // Data is LSB first, then MSB (Little Endian)
        *data = (uint16_t)rx_buf[0] | ((uint16_t)rx_buf[1] << 8);
    } else {
        ESP_LOGE(TAG, "Failed to read register 0x%02X, error: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}


esp_err_t veml7700_init(i2c_master_bus_handle_t i2c_handle) {
    if (i2c_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    veml7700_i2c_bus_handle = i2c_handle; // Store the bus handle

    // Create an I2C device handle for VEML7700 on the specified bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit address length
        .device_address = VEML7700_I2C_ADDR,   // Corrected: Use .device_address
        .scl_speed_hz = 400000,                // Standard I2C speed (100 KHz). You can increase this to 400 KHz if needed.
    };
    esp_err_t ret = i2c_master_bus_add_device(veml7700_i2c_bus_handle, &dev_cfg, &veml7700_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add VEML7700 I2C device to bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Initializing VEML7700 sensor at 0x%02X", VEML7700_I2C_ADDR);

    // Initial power on (clears SD bit 0)
    ret = veml7700_set_power_state(VEML7700_POWER_ON);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on VEML7700.");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Give some time after power on

    // Set default gain and integration time and interrupt enable
    ret = veml7700_set_gain(VEML7700_GAIN_X1);
    if (ret != ESP_OK) return ret;

    ret = veml7700_set_integration_time(VEML7700_IT_50MS);
    if (ret != ESP_OK) return ret;

    ret = veml7700_set_interrupt_enable(VEML7700_INTERRUPT_DISABLE);
    if (ret != ESP_OK) return ret;

    // Disable power saving mode by default
    ret = veml7700_enable_power_save(VEML7700_PSM_ENABLE);
    if (ret != ESP_OK) return ret;
    ret = veml7700_set_power_save_mode(VEML7700_PSM_MODE4); // Set to default mode 1
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "VEML7700 initialized successfully (Gain: X1, IT: 800ms).");
    return ESP_OK;
}

esp_err_t veml7700_set_high_threshold(uint16_t threshold) {
    return veml7700_write_reg(VEML7700_ALS_WH_REG, threshold);
}

esp_err_t veml7700_set_low_threshold(uint16_t threshold) {
    return veml7700_write_reg(VEML7700_ALS_LL_REG, threshold);
}

static esp_err_t veml7700_read_modify_write_psm_reg(uint16_t *current_val) {
    esp_err_t ret = veml7700_read_reg(VEML7700_PSM_REG, current_val);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read PSM_REG. Reconstructing from internal state.");
        *current_val = 0;
        *current_val |= (uint16_t)current_psm_mode << 1; // PSM bits [2:1]
        *current_val |= (uint16_t)current_psm_en << 0;   // PSM_EN bit [0]
    }
    return ESP_OK;
}

esp_err_t veml7700_set_power_save_mode(veml7700_power_save_mode_t mode) {
    uint16_t psm_val;
    esp_err_t ret = veml7700_read_modify_write_psm_reg(&psm_val);
    if (ret != ESP_OK) return ret;

    psm_val &= ~((uint16_t)0x03 << 1); // Clear bits 2 and 1
    psm_val |= (uint16_t)mode << 1;    // Set new PSM mode

    ret = veml7700_write_reg(VEML7700_PSM_REG, psm_val);
    if (ret == ESP_OK) {
        current_psm_mode = mode;
    }
    return ret;
}

esp_err_t veml7700_enable_power_save(veml7700_power_save_enable_t enable) {
    uint16_t psm_val;
    esp_err_t ret = veml7700_read_modify_write_psm_reg(&psm_val);
    if (ret != ESP_OK) return ret;

    if (enable == VEML7700_PSM_ENABLE) {
        psm_val |= (1 << 0); // Set PSM_EN bit to 1
    } else {
        psm_val &= ~(1 << 0); // Clear PSM_EN bit to 0
    }

    ret = veml7700_write_reg(VEML7700_PSM_REG, psm_val);
    if (ret == ESP_OK) {
        current_psm_en = enable;
    }
    return ret;
}

esp_err_t veml7700_read_als(uint16_t *als_data) {
    vTaskDelay(pdMS_TO_TICKS(5));
    return veml7700_read_reg(VEML7700_ALS_DATA_REG, als_data);
}

esp_err_t veml7700_read_white(uint16_t *white_data) {
    vTaskDelay(pdMS_TO_TICKS(5));
    return veml7700_read_reg(VEML7700_WHITE_DATA_REG, white_data);
}

esp_err_t veml7700_get_interrupt_status(uint16_t *status) {
    return veml7700_read_reg(VEML7700_INTERRUPT_STATUS_REG, status);
}


float veml7700_raw_to_lux(uint16_t als_raw) {
    float gain_value = 0.0;
    switch (current_gain) {
        case VEML7700_GAIN_X1_8: gain_value = 0.125; break;
        case VEML7700_GAIN_X1_4: gain_value = 0.25;  break;
        case VEML7700_GAIN_X1:   gain_value = 1.0;   break;
        case VEML7700_GAIN_X2:   gain_value = 2.0;   break;
        default:
            ESP_LOGE(TAG, "Invalid current gain setting for Lux conversion.");
            return 0.0;
    }

    float it_ms = 0.0;
    switch (current_it) {
        case VEML7700_IT_25MS:  it_ms = 25.0;  break;
        case VEML7700_IT_50MS:  it_ms = 50.0;  break;
        case VEML7700_IT_100MS: it_ms = 100.0; break;
        case VEML7700_IT_200MS: it_ms = 200.0; break;
        case VEML7700_IT_400MS: it_ms = 400.0; break;
        case VEML7700_IT_800MS: it_ms = 800.0; break;
        default:
            ESP_LOGE(TAG, "Invalid current integration time setting for Lux conversion.");
            return 0.0;
    }

    const float BASE_LSB = 0.00452;
    float effective_resolution = (BASE_LSB * (800.0 / it_ms)) / gain_value;
    float lux = (float)als_raw * effective_resolution;

    return lux;
}