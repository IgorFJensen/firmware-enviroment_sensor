/*
 * Alerta imediato de risco detectado pela IA de audio.
 * Made by Igor Jensen - UFES - LAEEC
 */
#include "risk_alert.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "audio_ml.h"
#include "mqtt_handler.h"

static const char *TAG = "RISK_ALERT";

/*
 * Prioridades atuais: inferencia=3, sensores=4, OpenThread=5, captura=6.
 * O alerta fica acima dos sensores e da inferencia para interrompe-los.
 */
#define RISK_ALERT_TASK_PRIORITY      5
#define RISK_ALERT_TASK_STACK         4096

#define NOTIFY_RISK_START             (1UL << 0)
#define NOTIFY_RISK_END               (1UL << 1)

#define ALERT_PAYLOAD_MAX_LEN         256
#define ALERT_PUBLISH_TIMEOUT_MS      5000
#define ALERT_MAX_ATTEMPTS            3
#define ALERT_RETRY_DELAY_MS          1000
#define ALERT_AUDIO_PAUSE_TIMEOUT_MS  1500
#define ALERT_PRE_TX_QUIET_MS         20
#define ALERT_POST_TX_GUARD_MS        100

static TaskHandle_t s_alert_task = NULL;
static SemaphoreHandle_t s_sensor_gate = NULL;
static uint32_t s_alert_sequence = 0;

void risk_alert_sensor_gate_enter(void)
{
    if (s_sensor_gate != NULL) {
        xSemaphoreTake(s_sensor_gate, portMAX_DELAY);
    }
}

void risk_alert_sensor_gate_exit(void)
{
    if (s_sensor_gate != NULL) {
        xSemaphoreGive(s_sensor_gate);
    }
}

/* Roda na task de inferencia: so notifica, nunca bloqueia. */
static void on_risk_change(bool risk_active, void *ctx)
{
    (void)ctx;

    if (s_alert_task != NULL) {
        xTaskNotify(
            s_alert_task,
            risk_active ? NOTIFY_RISK_START : NOTIFY_RISK_END,
            eSetBits);
    }
}

static int build_payload(char *buffer, size_t size, bool risk_active)
{
    audio_ml_risk_status_t audio = {0};
    const bool valid = (audio_ml_get_risk_status(&audio) == ESP_OK);

    ++s_alert_sequence;

    return snprintf(
        buffer,
        size,
        "{"
        "\"device\":\"esp32c6_environment\","
        "\"event\":\"%s\","
        "\"alert_seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRIu64 ","
        "\"audio_risk\":%s,"
        "\"audio_probability\":%.3f,"
        "\"audio_average\":%.3f,"
        "\"audio_votes\":%u,"
        "\"audio_history\":%u"
        "}",
        risk_active ? "audio_risk_start" : "audio_risk_end",
        s_alert_sequence,
        (uint64_t)(esp_timer_get_time() / 1000LL),
        risk_active ? "true" : "false",
        valid ? audio.probability : 0.0f,
        valid ? audio.history_average : 0.0f,
        valid ? audio.positive_votes : 0U,
        valid ? audio.history_count : 0U);
}

static void send_alert(bool risk_active)
{
    char payload[ALERT_PAYLOAD_MAX_LEN];
    const int len = build_payload(payload, sizeof(payload), risk_active);

    if (len < 0 || (size_t)len >= sizeof(payload)) {
        ESP_LOGE(TAG, "Payload do alerta excedeu %u bytes",
                 (unsigned)sizeof(payload));
        return;
    }

    const int64_t detected_us = esp_timer_get_time();

    for (int attempt = 1; attempt <= ALERT_MAX_ATTEMPTS; ++attempt) {
        if (!mqtt_is_connected()) {
            ESP_LOGW(TAG, "Alerta %d/%d: MQTT desconectado, nova tentativa em %d ms",
                     attempt, ALERT_MAX_ATTEMPTS, ALERT_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(ALERT_RETRY_DELAY_MS));
            continue;
        }

        /* 1. Interrompe os sensores (espera no maximo a leitura atual). */
        risk_alert_sensor_gate_enter();

        /* 2. Pausa o audio, como no envio das medias. */
        const bool audio_paused =
            (audio_ml_pause(ALERT_AUDIO_PAUSE_TIMEOUT_MS) == ESP_OK);
        if (!audio_paused) {
            ESP_LOGW(TAG, "Audio nao pausou; enviando alerta mesmo assim");
        }

        vTaskDelay(pdMS_TO_TICKS(ALERT_PRE_TX_QUIET_MS));

        /* 3. Envia o alerta. */
        int msg_id = -1;
        const esp_err_t err = mqtt_publish_alert(
            payload, (size_t)len, ALERT_PUBLISH_TIMEOUT_MS, &msg_id);

        vTaskDelay(pdMS_TO_TICKS(ALERT_POST_TX_GUARD_MS));

        /* 4. Libera audio e sensores. */
        if (audio_paused) {
            audio_ml_resume();
        }
        risk_alert_sensor_gate_exit();

        if (err == ESP_OK) {
            ESP_LOGW(TAG, "ALERTA %s enviado e confirmado em %" PRId64 " ms (msg_id=%d)",
                     risk_active ? "RISCO" : "FIM DO RISCO",
                     (esp_timer_get_time() - detected_us) / 1000LL,
                     msg_id);
            return;
        }

        ESP_LOGW(TAG, "Alerta %d/%d falhou: %s",
                 attempt, ALERT_MAX_ATTEMPTS, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(ALERT_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "Alerta %s NAO foi enviado apos %d tentativas",
             risk_active ? "RISCO" : "FIM DO RISCO", ALERT_MAX_ATTEMPTS);
}

static void risk_alert_task(void *arg)
{
    (void)arg;

    while (1) {
        uint32_t events = 0;
        xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

        /* Se inicio e fim chegarem juntos, envia na ordem em que ocorreram. */
        if ((events & NOTIFY_RISK_START) != 0U) {
            ESP_LOGW(TAG, "IA detectou RISCO: interrompendo sensores para enviar alerta");
            send_alert(true);
        }

        if ((events & NOTIFY_RISK_END) != 0U) {
            send_alert(false);
        }
    }
}

esp_err_t risk_alert_start(void)
{
    if (s_alert_task != NULL) {
        return ESP_OK;
    }

    s_sensor_gate = xSemaphoreCreateMutex();
    if (s_sensor_gate == NULL) {
        ESP_LOGE(TAG, "Sem memoria para o portao dos sensores");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(risk_alert_task,
                    "risk_alert",
                    RISK_ALERT_TASK_STACK,
                    NULL,
                    RISK_ALERT_TASK_PRIORITY,
                    &s_alert_task) != pdPASS) {
        vSemaphoreDelete(s_sensor_gate);
        s_sensor_gate = NULL;
        ESP_LOGE(TAG, "Falha criando task de alerta");
        return ESP_ERR_NO_MEM;
    }

    audio_ml_set_risk_callback(on_risk_change, NULL);

    ESP_LOGI(TAG, "Alerta de risco ativo: topico sensors/alert, prioridade %d",
             RISK_ALERT_TASK_PRIORITY);
    return ESP_OK;
}
