#ifndef SPH0645_H
#define SPH0645_H

#include "esp_err.h"
#include "driver/i2s_std.h" // Using standard I2S driver for PDM input

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for the SPH0645 microphone.
 * This configures the I2S peripheral for PDM input.
 */
typedef struct {
    int i2s_port;           /*!< I2S port number (e.g., I2S_NUM_0) */
    int i2s_data_pin;       /*!< GPIO pin connected to the SPH0645 DATA line */
    int i2s_clk_pin;        /*!< GPIO pin connected to the SPH0645 CLK line */
    uint32_t sample_rate_hz; /*!< Desired PCM sample rate in Hz (e.g., 16000, 44100) */
    uint32_t buffer_size_bytes; /*!< Size of the I2S read buffer in bytes */
    uint32_t buffer_count;   /*!< Number of I2S buffers */
} sph0645_config_t;

/**
 * @brief Initializes the SPH0645 microphone using the I2S peripheral.
 *
 * This function sets up the I2S peripheral in PDM mode to receive audio
 * data from the SPH0645 microphone.
 *
 * @param[in] config Pointer to the SPH0645 configuration structure.
 * @return
 * - ESP_OK on success
 * - ESP_FAIL if initialization fails
 * - Other ESP_ERR_ codes from underlying I2S functions
 */
esp_err_t sph0645_init(const sph0645_config_t *config);

/**
 * @brief Reads a block of PCM samples from the SPH0645 microphone.
 *
 * This function reads a specified number of bytes from the I2S buffer,
 * which contains the PDM-converted-to-PCM audio data. The data will be
 * 16-bit signed integers (short).
 *
 * @param[out] buffer Pointer to the buffer where the audio samples will be stored.
 * @param[in] buffer_size_bytes The size of the buffer in bytes.
 * @param[out] bytes_read Pointer to a variable that will store the number of bytes actually read.
 * @param[in] timeout_ms Timeout for the read operation in milliseconds.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_TIMEOUT if the read operation times out
 * - Other ESP_ERR_ codes from underlying I2S functions
 */
esp_err_t sph0645_read_samples(void *buffer, size_t buffer_size_bytes, size_t *bytes_read, TickType_t timeout_ticks);

/**
 * @brief Deinitializes the SPH0645 microphone and releases I2S resources.
 *
 * @return
 * - ESP_OK on success
 * - ESP_FAIL if deinitialization fails
 */
esp_err_t sph0645_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SPH0645_H
