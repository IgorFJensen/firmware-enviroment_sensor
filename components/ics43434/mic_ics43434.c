#include "mic_ics43434.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MIC_ICS43434";

// Handle do canal I2S de recepção
static i2s_chan_handle_t rx_chan = NULL;

// Pinos definidos
#define MIC_BCLK_PIN  6  // SCK
#define MIC_WS_PIN    7  // WS
#define MIC_DATA_PIN  5  // SD (Data In no ESP32)

esp_err_t mic_ics43434_init(void) {
    // Aloca um novo canal RX atuando como Master
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao alocar canal I2S");
        return err;
    }

    // Configuração do modo I2S Standard (Philips)
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000), // Taxa de amostragem: 16 kHz. Altere conforme a necessidade do processamento.
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK_PIN,
            .ws   = MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_DATA_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    // Assume que o pino L/R do microfone físico está em GND (Canal Esquerdo)
    // Se estiver em VDD, mude para I2S_STD_SLOT_RIGHT
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar modo padrão do I2S");
        return err;
    }

    err = i2s_channel_enable(rx_chan);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Microfone I2S ICS-43434 inicializado com sucesso!");
    } else {
        ESP_LOGE(TAG, "Falha ao habilitar o canal I2S");
    }

    return err;
}

esp_err_t mic_ics43434_read(int32_t *buffer, size_t buffer_len_bytes, size_t *bytes_read) {
    if (rx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Lê os dados via DMA para o buffer. O timeout de 1000ms evita que a task trave permanentemente.
    return i2s_channel_read(rx_chan, buffer, buffer_len_bytes, bytes_read, pdMS_TO_TICKS(1000));
}