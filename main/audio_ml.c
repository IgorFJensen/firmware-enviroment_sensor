#include "audio_ml.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "mic_ics43434.h"
#include "tinyml_runtime.h"

static const char *TAG = "AUDIO_ML";

#define AUDIO_ML_CHUNK_SAMPLES       512
#define AUDIO_ML_WINDOW_SAMPLES      16000
#define AUDIO_ML_BUFFER_COUNT        2

/*
 * 1 = inferência a cada 1 segundo
 * 2 = inferência a cada 2 segundos
 */
#define AUDIO_ML_INFERENCE_DIVIDER   2

#define AUDIO_CAPTURE_STACK_SIZE     4096
#define AUDIO_CAPTURE_PRIORITY       6

#define AUDIO_INFER_STACK_SIZE       8192
#define AUDIO_INFER_PRIORITY         3

#define AUDIO_ML_DIGITAL_GAIN        1

/*
 * LED de risco.
 *
 * GPIO15 em nível alto liga o LED.
 * Caso seu LED funcione invertido, troque os níveis abaixo.
 */
#define AUDIO_ML_RISK_LED_GPIO       GPIO_NUM_15
#define AUDIO_ML_LED_ACTIVE_LEVEL    1
#define AUDIO_ML_LED_INACTIVE_LEVEL  0

static volatile float s_latest_mic_db = 0.0f;

static TaskHandle_t s_capture_task_handle = NULL;
static TaskHandle_t s_infer_task_handle = NULL;

static QueueHandle_t s_free_buffers = NULL;
static QueueHandle_t s_ready_buffers = NULL;

static int16_t *s_pcm_windows = NULL;

static volatile uint32_t s_dropped_chunks = 0;
static volatile uint32_t s_skipped_windows = 0;

static bool s_led_risk_state = false;

static inline int16_t pcm24_to_pcm16(int32_t raw)
{
    /*
     * ICS-43434:
     * 24 bits válidos alinhados na parte superior do slot de 32 bits.
     */
    int32_t sample = raw >> 16;

    sample *= AUDIO_ML_DIGITAL_GAIN;

    if (sample > INT16_MAX) {
        sample = INT16_MAX;
    } else if (sample < INT16_MIN) {
        sample = INT16_MIN;
    }

    return (int16_t)sample;
}

static void update_mic_level(
    const int32_t *raw,
    size_t samples)
{
    if (raw == NULL || samples == 0) {
        return;
    }

    double sum_squares = 0.0;

    for (size_t i = 0; i < samples; ++i) {
        const int32_t sample24 = raw[i] >> 8;

        sum_squares +=
            (double)sample24 *
            (double)sample24;
    }

    const double mean_square =
        sum_squares / (double)samples;

    const double rms = sqrt(mean_square);

    if (rms > 0.0) {
        s_latest_mic_db =
            (float)(20.0 * log10(rms));
    } else {
        s_latest_mic_db = 0.0f;
    }
}

static esp_err_t read_exact_i2s(
    int32_t *buffer,
    size_t samples)
{
    if (buffer == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_bytes = 0;

    const size_t wanted_bytes =
        samples * sizeof(int32_t);

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

static int16_t *window_ptr(uint8_t index)
{
    return &s_pcm_windows[
        (size_t)index * AUDIO_ML_WINDOW_SAMPLES
    ];
}

static void risk_led_init(void)
{
    gpio_reset_pin(AUDIO_ML_RISK_LED_GPIO);

    esp_err_t err = gpio_set_direction(
        AUDIO_ML_RISK_LED_GPIO,
        GPIO_MODE_OUTPUT);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Falha configurando LED GPIO%d: %s",
            AUDIO_ML_RISK_LED_GPIO,
            esp_err_to_name(err));
        return;
    }

    gpio_set_level(
        AUDIO_ML_RISK_LED_GPIO,
        AUDIO_ML_LED_INACTIVE_LEVEL);

    s_led_risk_state = false;

    ESP_LOGI(
        TAG,
        "LED de risco configurado no GPIO%d",
        AUDIO_ML_RISK_LED_GPIO);
}

static void risk_led_update(void)
{
    tinyml_risk_status_t status = {0};

    esp_err_t err =
        tinyml_runtime_get_status(&status);

    if (err != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Falha lendo estado para LED: %s",
            esp_err_to_name(err));
        return;
    }

    if (status.risk_active == s_led_risk_state) {
        return;
    }

    s_led_risk_state = status.risk_active;

    gpio_set_level(
        AUDIO_ML_RISK_LED_GPIO,
        s_led_risk_state
            ? AUDIO_ML_LED_ACTIVE_LEVEL
            : AUDIO_ML_LED_INACTIVE_LEVEL);

    ESP_LOGI(
        TAG,
        "LED de risco: %s",
        s_led_risk_state
            ? "LIGADO"
            : "DESLIGADO");
}

static void audio_infer_task(void *arg)
{
    (void)arg;

    uint8_t buffer_index = 0;
    uint32_t received_windows = 0;

    ESP_LOGI(
        TAG,
        "Inferencia configurada: 1 a cada %d janela(s)",
        AUDIO_ML_INFERENCE_DIVIDER);

    while (1) {
        if (xQueueReceive(
                s_ready_buffers,
                &buffer_index,
                portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ++received_windows;

        /*
         * Com divisor 2:
         *
         * Janela 1: processada
         * Janela 2: ignorada
         * Janela 3: processada
         * Janela 4: ignorada
         */
        const bool should_run_inference =
            (((received_windows - 1U) %
              AUDIO_ML_INFERENCE_DIVIDER) == 0U);

        if (!should_run_inference) {
            ++s_skipped_windows;

            xQueueSend(
                s_free_buffers,
                &buffer_index,
                portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        esp_err_t err =
            tinyml_runtime_infer_pcm16(
                window_ptr(buffer_index),
                AUDIO_ML_WINDOW_SAMPLES);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Inferencia TinyML falhou: %s",
                esp_err_to_name(err));
        } else {
            /*
             * Atualiza o LED imediatamente após a IA atualizar
             * o estado temporal de risco.
             */
            risk_led_update();
        }

        /*
         * Libera o buffer somente depois da inferência.
         */
        xQueueSend(
            s_free_buffers,
            &buffer_index,
            portMAX_DELAY);

        /*
         * Dá tempo para a tarefa IDLE executar e alimentar
         * o Task Watchdog.
         */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void audio_capture_task(void *arg)
{
    (void)arg;

    int32_t *raw_buffer = heap_caps_malloc(
        AUDIO_ML_CHUNK_SAMPLES * sizeof(int32_t),
        MALLOC_CAP_8BIT);

    if (raw_buffer == NULL) {
        ESP_LOGE(
            TAG,
            "Sem memoria para buffer I2S");

        s_capture_task_handle = NULL;

        vTaskDelete(NULL);
        return;
    }

    uint8_t current_buffer = 0;

    bool have_buffer =
        (xQueueReceive(
             s_free_buffers,
             &current_buffer,
             portMAX_DELAY) == pdTRUE);

    size_t window_fill = 0;

    ESP_LOGI(
        TAG,
        "Captura continua ativa: 16 kHz PCM16, "
        "chunk=%d, double-buffer=%d x %d amostras",
        AUDIO_ML_CHUNK_SAMPLES,
        AUDIO_ML_BUFFER_COUNT,
        AUDIO_ML_WINDOW_SAMPLES);

    while (1) {
        esp_err_t err = read_exact_i2s(
            raw_buffer,
            AUDIO_ML_CHUNK_SAMPLES);

        if (err != ESP_OK) {
            ESP_LOGW(
                TAG,
                "Falha lendo I2S: %s",
                esp_err_to_name(err));

            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        update_mic_level(
            raw_buffer,
            AUDIO_ML_CHUNK_SAMPLES);

        size_t src = 0;

        while (src < AUDIO_ML_CHUNK_SAMPLES) {
            if (!have_buffer) {
                if (xQueueReceive(
                        s_free_buffers,
                        &current_buffer,
                        0) == pdTRUE) {
                    have_buffer = true;
                    window_fill = 0;
                } else {
                    /*
                     * A inferência ainda não liberou um buffer.
                     * O I2S continua sendo lido para não travar
                     * o DMA, mas este trecho é descartado.
                     */
                    ++s_dropped_chunks;

                    if ((s_dropped_chunks % 50U) == 1U) {
                        ESP_LOGW(
                            TAG,
                            "ML atrasado: descartando audio. "
                            "chunks perdidos=%lu",
                            (unsigned long)s_dropped_chunks);
                    }

                    break;
                }
            }

            const size_t space =
                AUDIO_ML_WINDOW_SAMPLES -
                window_fill;

            const size_t remaining =
                AUDIO_ML_CHUNK_SAMPLES -
                src;

            const size_t copy_count =
                (space < remaining)
                    ? space
                    : remaining;

            int16_t *dst =
                window_ptr(current_buffer);

            for (size_t i = 0; i < copy_count; ++i) {
                dst[window_fill + i] =
                    pcm24_to_pcm16(
                        raw_buffer[src + i]);
            }

            window_fill += copy_count;
            src += copy_count;

            if (window_fill ==
                AUDIO_ML_WINDOW_SAMPLES) {
                if (xQueueSend(
                        s_ready_buffers,
                        &current_buffer,
                        0) != pdTRUE) {
                    ESP_LOGW(
                        TAG,
                        "Fila TinyML cheia; janela descartada");

                    xQueueSend(
                        s_free_buffers,
                        &current_buffer,
                        portMAX_DELAY);
                }

                have_buffer = false;
                window_fill = 0;
            }
        }
    }
}

esp_err_t audio_ml_start(void)
{
    if (s_capture_task_handle != NULL ||
        s_infer_task_handle != NULL) {
        return ESP_OK;
    }

    risk_led_init();

    esp_err_t err =
        mic_ics43434_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Falha ao inicializar ICS-43434: %s",
            esp_err_to_name(err));

        return err;
    }

    err = tinyml_runtime_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Falha ao inicializar runtime TinyML: %s",
            esp_err_to_name(err));

        return err;
    }

    s_pcm_windows = heap_caps_calloc(
        AUDIO_ML_BUFFER_COUNT *
            AUDIO_ML_WINDOW_SAMPLES,
        sizeof(int16_t),
        MALLOC_CAP_8BIT);

    if (s_pcm_windows == NULL) {
        ESP_LOGE(
            TAG,
            "Sem memoria para double-buffer de audio (%u bytes)",
            (unsigned)(
                AUDIO_ML_BUFFER_COUNT *
                AUDIO_ML_WINDOW_SAMPLES *
                sizeof(int16_t)));

        return ESP_ERR_NO_MEM;
    }

    s_free_buffers = xQueueCreate(
        AUDIO_ML_BUFFER_COUNT,
        sizeof(uint8_t));

    s_ready_buffers = xQueueCreate(
        AUDIO_ML_BUFFER_COUNT,
        sizeof(uint8_t));

    if (s_free_buffers == NULL ||
        s_ready_buffers == NULL) {
        ESP_LOGE(
            TAG,
            "Falha criando filas de audio");

        if (s_free_buffers != NULL) {
            vQueueDelete(s_free_buffers);
            s_free_buffers = NULL;
        }

        if (s_ready_buffers != NULL) {
            vQueueDelete(s_ready_buffers);
            s_ready_buffers = NULL;
        }

        free(s_pcm_windows);
        s_pcm_windows = NULL;

        return ESP_ERR_NO_MEM;
    }

    for (uint8_t i = 0;
         i < AUDIO_ML_BUFFER_COUNT;
         ++i) {
        xQueueSend(
            s_free_buffers,
            &i,
            0);
    }

    BaseType_t created = xTaskCreate(
        audio_infer_task,
        "tinyml_infer",
        AUDIO_INFER_STACK_SIZE,
        NULL,
        AUDIO_INFER_PRIORITY,
        &s_infer_task_handle);

    if (created != pdPASS) {
        s_infer_task_handle = NULL;

        ESP_LOGE(
            TAG,
            "Falha criando tarefa de inferencia");

        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(
        audio_capture_task,
        "audio_capture",
        AUDIO_CAPTURE_STACK_SIZE,
        NULL,
        AUDIO_CAPTURE_PRIORITY,
        &s_capture_task_handle);

    if (created != pdPASS) {
        s_capture_task_handle = NULL;

        vTaskDelete(s_infer_task_handle);
        s_infer_task_handle = NULL;

        ESP_LOGE(
            TAG,
            "Falha criando tarefa de captura");

        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "Pipeline iniciado: capture prio=%d, "
        "infer prio=%d, inferencia a cada %d segundos",
        AUDIO_CAPTURE_PRIORITY,
        AUDIO_INFER_PRIORITY,
        AUDIO_ML_INFERENCE_DIVIDER);

    return ESP_OK;
}

float audio_ml_get_db(void)
{
    return s_latest_mic_db;
}

esp_err_t audio_ml_get_risk_status(
    audio_ml_risk_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tinyml_risk_status_t runtime_status = {0};

    esp_err_t err =
        tinyml_runtime_get_status(
            &runtime_status);

    if (err != ESP_OK) {
        return err;
    }

    out_status->probability =
        runtime_status.probability;

    out_status->history_average =
        runtime_status.history_average;

    out_status->risk_active =
        runtime_status.risk_active;

    out_status->positive_votes =
        runtime_status.positive_votes;

    out_status->history_count =
        runtime_status.history_count;

    out_status->inference_count =
        runtime_status.inference_count;

    return ESP_OK;
}