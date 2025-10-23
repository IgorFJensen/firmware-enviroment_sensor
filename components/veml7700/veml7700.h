#ifndef VEML7700_H
#define VEML7700_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Slave Address for VEML7700
#define VEML7700_I2C_ADDR 0x10

// VEML7700 Command Code Addresses (Registers)
#define VEML7700_ALS_CONF_REG        0x00 // ALS configuration register
#define VEML7700_ALS_WH_REG          0x01 // ALS high threshold windows setting
#define VEML7700_ALS_LL_REG          0x02 // ALS low threshold windows setting
#define VEML7700_PSM_REG             0x03 // Power saving mode register
#define VEML7700_ALS_DATA_REG        0x04 // ALS high resolution output data
#define VEML7700_WHITE_DATA_REG      0x05 // White channel output data
#define VEML7700_INTERRUPT_STATUS_REG 0x06 // Interrupt status register

// --- Bit Definitions for ALS_CONF_REG (0x00) ---
// ALS Gain (ALS_GAIN) - Bits 12:11
typedef enum {
    VEML7700_GAIN_X1_8 = 0x00, // ALS Gain = 1/8
    VEML7700_GAIN_X1_4 = 0x01, // ALS Gain = 1/4
    VEML7700_GAIN_X1   = 0x02, // ALS Gain = 1 (default)
    VEML7700_GAIN_X2   = 0x03  // ALS Gain = 2
} veml7700_gain_t;

// ALS Integration Time (ALS_IT) - Bits 9:6
typedef enum {
    VEML7700_IT_100MS = 0x00, // ALS Integration Time = 100ms (default)
    VEML7700_IT_200MS = 0x01, // ALS Integration Time = 200ms
    VEML7700_IT_400MS = 0x02, // ALS Integration Time = 400ms
    VEML7700_IT_800MS = 0x03, // ALS Integration Time = 800ms
    VEML7700_IT_50MS  = 0x08, // ALS Integration Time = 50ms
    VEML7700_IT_25MS  = 0x09, // ALS Integration Time = 25ms
} veml7700_it_t;

// ALS Interrupt Enable (INT_EN) - Bit 1
typedef enum {
    VEML7700_INTERRUPT_DISABLE = 0x00,
    VEML7700_INTERRUPT_ENABLE  = 0x01
} veml7700_interrupt_enable_t;

// ALS Shutdown (SD) - Bit 0
typedef enum {
    VEML7700_POWER_ON  = 0x00, // ALS active
    VEML7700_POWER_OFF = 0x01  // ALS shutdown
} veml7700_power_state_t;


// --- Bit Definitions for PSM_REG (0x03) ---
// Power saving mode (PSM) - Bits 2:1
typedef enum {
    VEML7700_PSM_MODE1 = 0x00, // Default, faster refresh
    VEML7700_PSM_MODE2 = 0x01,
    VEML7700_PSM_MODE3 = 0x02,
    VEML7700_PSM_MODE4 = 0x03  // Slowest refresh, max power saving
} veml7700_power_save_mode_t;

// Power saving mode enable setting (PSM_EN) - Bit 0
typedef enum {
    VEML7700_PSM_DISABLE = 0x00,
    VEML7700_PSM_ENABLE  = 0x01
} veml7700_power_save_enable_t;

// --- Bit Definitions for INTERRUPT_STATUS_REG (0x06) ---
#define VEML7700_INT_LOW_THRESHOLD_FLAG  (1 << 15) // Bit 15: Low threshold interrupt flag
#define VEML7700_INT_HIGH_THRESHOLD_FLAG (1 << 14) // Bit 14: High threshold interrupt flag


/**
 * @brief Initializes the VEML7700 sensor.
 * Stores the I2C bus handle and sets default sensor configurations.
 *
 * @param i2c_handle The I2C master bus handle created in your main application.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_init(i2c_master_bus_handle_t i2c_handle);

/**
 * @brief Sets the high threshold for ALS interrupt.
 *
 * @param threshold 16-bit raw ALS value for the high threshold.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_set_high_threshold(uint16_t threshold);

/**
 * @brief Sets the low threshold for ALS interrupt.
 *
 * @param threshold 16-bit raw ALS value for the low threshold.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_set_low_threshold(uint16_t threshold);

/**
 * @brief Sets the power saving mode (PSM).
 *
 * @param mode The desired power saving mode (e.g., VEML7700_PSM_MODE1).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_set_power_save_mode(veml7700_power_save_mode_t mode);

/**
 * @brief Enables or disables the power saving mode.
 *
 * @param enable VEML7700_PSM_ENABLE to enable, VEML7700_PSM_DISABLE to disable.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_enable_power_save(veml7700_power_save_enable_t enable);

/**
 * @brief Reads the raw ambient light sensor (ALS) data.
 *
 * @param als_data Pointer to a uint16_t variable to store the raw ALS data.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_read_als(uint16_t *als_data);

/**
 * @brief Reads the raw white light data.
 *
 * @param white_data Pointer to a uint16_t variable to store the raw white light data.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_read_white(uint16_t *white_data);

/**
 * @brief Reads the interrupt status register.
 *
 * @param status Pointer to a uint16_t variable to store the raw status register value.
 * Use VEML7700_INT_LOW_THRESHOLD_FLAG and VEML7700_INT_HIGH_THRESHOLD_FLAG
 * to check individual flags.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t veml7700_get_interrupt_status(uint16_t *status);

/**
 * @brief Converts raw ALS data to Lux.
 * This function uses the current stored gain and integration time for conversion.
 *
 * @param als_raw The raw ALS data read from the sensor.
 * @return The calculated Lux value. Returns 0.0 if gain/IT settings are invalid.
 */
float veml7700_raw_to_lux(uint16_t als_raw);


esp_err_t veml7700_read_reg(uint8_t reg_addr, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif // VEML7700_H