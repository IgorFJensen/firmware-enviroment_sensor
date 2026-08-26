#include "audio_wakenet.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"

#include "mic_ics43434.h"

static const char *TAG = "WAKE_NET";

/*
 * O ICS-43434 entrega audio de 24 bits dentro do word I2S de 32 bits.
 * raw >> 16 transforma o sinal em PCM16. Como microfones digitais desse tipo
 * normalmente entregam fala com amplitude relativamente baixa, aplicamos um
 * ganho digital inicial de 8x antes do WakeNet. Se houver clipping frequente,
 * reduza para 4; se a fala ficar muito baixa, experimente 16.
 */
#define WAKENET_DIGITAL_GAIN 16
#define WAKENET_TASK_STACK_SIZE 8192
#define WAKENET_TASK_PRIORITY   5

static volatile float s_latest_mic_db = 0.0f;
static TaskHandle_t s_task_handle = NULL;

static int16_t pcm24_to_pcm16(int32_t raw)
{
    int32_t sample = raw >> 16;
    sample *= WAKENET_DIGITAL_GAIN;

    if (sample > INT16_MAX) {
        sample = INT16_MAX;
    } else if (sample < INT16_MIN) {
        sample = INT16_MIN;
    }

    return (int16_t)sample;
}

static void update_mic_level(const int32_t *raw, int samples)
{
    if (samples <= 0) {
        return;
    }

    double sum_squares = 0.0;
    for (int i = 0; i < samples; ++i) {
        /* Mantem a mesma escala usada anteriormente em sensor_task.c. */
        int32_t sample24 = raw[i] >> 8;
        sum_squares += (double)sample24 * (double)sample24;
    }

    double rms = sqrt(sum_squares / (double)samples);
    s_latest_mic_db = (rms > 0.0) ? (float)(20.0 * log10(rms)) : 0.0f;
}

static esp_err_t read_exact_i2s(int32_t *buffer, size_t samples)
{
    size_t total_bytes = 0;
    const size_t wanted_bytes = samples * sizeof(int32_t);

    while (total_bytes < wanted_bytes) {
        size_t bytes_read = 0;
        esp_err_t err = mic_ics43434_read(
            (int32_t *)((uint8_t *)buffer + total_bytes),
            wanted_bytes - total_bytes,
            &bytes_read);

        if (err != ESP_OK) {
            return err;
        }
        if (bytes_read == 0) {
            return ESP_ERR_TIMEOUT;
        }

        total_bytes += bytes_read;
    }

    return ESP_OK;
}

static void wakenet_task(void *arg)
{
    (void)arg;

    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL) {
        ESP_LOGE(TAG, "Nao foi possivel carregar a particao de modelos 'model'");
        goto task_fail;
    }

    ESP_LOGI(TAG, "Modelos ESP-SR encontrados na flash: %d", models->num);
    for (int i = 0; i < models->num; ++i) {
        ESP_LOGI(TAG, "  [%d] %s", i, models->model_name[i]);
    }

    char *model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp");
    if (model_name == NULL) {
        ESP_LOGE(TAG, "WakeNet 'Hi ESP' nao encontrado na particao de modelos");
        esp_srmodel_deinit(models);
        goto task_fail;
    }

    const esp_wn_iface_t *wakenet = esp_wn_handle_from_name(model_name);
    if (wakenet == NULL) {
        ESP_LOGE(TAG, "Falha ao obter interface do WakeNet para %s", model_name);
        esp_srmodel_deinit(models);
        goto task_fail;
    }

    model_iface_data_t *model_data = wakenet->create(model_name, DET_MODE_90);
    if (model_data == NULL) {
        ESP_LOGE(TAG, "Falha ao criar modelo WakeNet %s", model_name);
        esp_srmodel_deinit(models);
        goto task_fail;
    }

    const int sample_rate = wakenet->get_samp_rate(model_data);
    const int chunk_samples = wakenet->get_samp_chunksize(model_data);

    ESP_LOGI(TAG, "WakeNet pronto: %s | sample rate=%d Hz | chunk=%d amostras",
             model_name, sample_rate, chunk_samples);

    if (sample_rate != 16000) {
        ESP_LOGW(TAG, "O microfone esta configurado em 16 kHz, mas o modelo pediu %d Hz",
                 sample_rate);
    }

    int32_t *raw_buffer = heap_caps_malloc(
        (size_t)chunk_samples * sizeof(int32_t), MALLOC_CAP_8BIT);
    int16_t *pcm_buffer = heap_caps_malloc(
        (size_t)chunk_samples * sizeof(int16_t), MALLOC_CAP_8BIT);

    if (raw_buffer == NULL || pcm_buffer == NULL) {
        ESP_LOGE(TAG, "Sem memoria para buffers de audio do WakeNet");
        free(raw_buffer);
        free(pcm_buffer);
        wakenet->destroy(model_data);
        esp_srmodel_deinit(models);
        goto task_fail;
    }

    ESP_LOGI(TAG, "Escutando continuamente. Diga: Hi ESP");

    while (1) {
        esp_err_t err = read_exact_i2s(raw_buffer, (size_t)chunk_samples);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha lendo I2S: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        update_mic_level(raw_buffer, chunk_samples);

        for (int i = 0; i < chunk_samples; ++i) {
            pcm_buffer[i] = pcm24_to_pcm16(raw_buffer[i]);
        }

        wakenet_state_t state = wakenet->detect(model_data, pcm_buffer);
        if (state == WAKENET_DETECTED) {
            ESP_LOGW(TAG, "====================================");
            ESP_LOGW(TAG, ">>> WAKE WORD DETECTADA: HI ESP! <<<");
            ESP_LOGW(TAG, "====================================");
        }
    }

    /* Nao deve chegar aqui no funcionamento normal. */
    free(raw_buffer);
    free(pcm_buffer);
    wakenet->destroy(model_data);
    esp_srmodel_deinit(models);

task_fail:
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t audio_wakenet_start(void)
{
    if (s_task_handle != NULL) {
        return ESP_OK;
    }

    esp_err_t err = mic_ics43434_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar ICS-43434: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t created = xTaskCreate(
        wakenet_task,
        "wakenet",
        WAKENET_TASK_STACK_SIZE,
        NULL,
        WAKENET_TASK_PRIORITY,
        &s_task_handle);

    if (created != pdPASS) {
        s_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

float audio_wakenet_get_db(void)
{
    return s_latest_mic_db;
}
