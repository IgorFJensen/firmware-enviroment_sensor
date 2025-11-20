/*
    Made by Igor Jensen - UFES - LAEEC
    20/11/2025
    -unused
*/

#include "sph0645.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h" // For GPIO configuration if needed, though I2S handles it

static const char *TAG_SPH0645 = "SPH0645";
static i2s_chan_handle_t rx_chan; // I2S RX channel handle

esp_err_t sph0645_init(const sph0645_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG_SPH0645, "SPH0645 configuration is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // I2S channel configuration
    i2s_chan_config_t chan_cfg = {
        .id = config->i2s_port,
        .role = I2S_ROLE_MASTER, // Microphone is a serve, ESP32 is master
        .dma_desc_num = config->buffer_count,
        .dma_frame_num = config->buffer_size_bytes / (sizeof(int16_t) * 2), // Frame size for 16-bit stereo (or 2 channels)
        .auto_clear = true, // Auto clear DMA Rx data if not read in time
    };

    // I2S standard mode configuration for PDM input
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2sClkConfig_Default(config->sample_rate_hz),
        .slot_cfg = I2sSlotConfig_Default(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), // PDM usually converts to 16-bit mono PCM
        .action = {
            .rx = {.pin_cfg = I2sPinConfig_Default(config->i2s_clk_pin, config->i2s_data_pin, I2S_GPIO_UNUSED, I2S_GPIO_UNUSED)},
            .tx = {.pin_cfg = I2sPinConfig_Default(I2S_GPIO_UNUSED, I2S_GPIO_UNUSED, I2S_GPIO_UNUSED, I2S_GPIO_UNUSED)},
        }
    };

    // For PDM microphones, the I2S peripheral handles the PDM to PCM conversion.
    // The SPH0645 outputs PDM data. The ESP32's I2S peripheral can be configured
    // to receive PDM input and convert it to PCM.
    // The `i2s_std_config_t` is used, but internally for PDM, we need to set the PDM mode.
    // This is typically handled by `i2s_channel_init_pdm_rx_mode` or similar.
    // However, for simplicity and common usage, we'll try with `i2s_channel_init_std_mode`
    // and rely on the `i2s_std_config_t` to implicitly handle PDM if the `slot_cfg` and `clk_cfg`
    // are set appropriately, or if the I2S peripheral has a dedicated PDM input path.
    // For SPH0645, it's a PDM microphone, so we should use PDM specific functions if available,
    // or ensure the standard mode correctly interprets PDM.
    // ESP-IDF v5.x has `i2s_channel_init_pdm_rx_mode`. Let's use that for clarity.

    // Using PDM RX specific configuration
    i2s_pdm_rx_clk_config_t pdm_rx_clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(config->sample_rate_hz);
    i2s_pdm_rx_slot_config_t pdm_rx_slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    i2s_pdm_rx_gpio_config_t pdm_rx_gpio_cfg = {
        .clk = config->i2s_clk_pin,
        .din = config->i2s_data_pin,
        .invert_flags = {
            .clk_in = false,
        },
    };

    ESP_LOGI(TAG_SPH0645, "Initializing SPH0645 on I2S port %d, DATA: %d, CLK: %d, Sample Rate: %lu Hz",
             config->i2s_port, config->i2s_data_pin, config->i2s_clk_pin, config->sample_rate_hz);

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPH0645, "Failed to create I2S RX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_clk_cfg, &pdm_rx_slot_cfg, &pdm_rx_gpio_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPH0645, "Failed to initialize I2S PDM RX mode: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

    ret = i2s_channel_enable(rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPH0645, "Failed to enable I2S RX channel: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_chan);
        return ret;
    }

    ESP_LOGI(TAG_SPH0645, "SPH0645 I2S initialized successfully.");
    return ESP_OK;
}

esp_err_t sph0645_read_samples(void *buffer, size_t buffer_size_bytes, size_t *bytes_read, TickType_t timeout_ticks) {
    if (!rx_chan || !buffer || !bytes_read) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2s_channel_read(rx_chan, buffer, buffer_size_bytes, bytes_read, timeout_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPH0645, "Failed to read I2S samples: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sph0645_deinit(void) {
    if (rx_chan) {
        esp_err_t ret = i2s_channel_disable(rx_chan);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SPH0645, "Failed to disable I2S RX channel: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = i2s_del_channel(rx_chan);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SPH0645, "Failed to delete I2S RX channel: %s", esp_err_to_name(ret));
            return ret;
        }
        rx_chan = NULL;
        ESP_LOGI(TAG_SPH0645, "SPH0645 I2S deinitialized successfully.");
    }
    return ESP_OK;
}