#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicializa o ICS-43434 e o pipeline TinyML desacoplado.
 *
 * Task 1: captura I2S continuamente e preenche double-buffer.
 * Task 2: processa Log-Mel + TFLite Micro sem bloquear a captura.
 */
esp_err_t audio_ml_start(void);

/**
 * Pausa cooperativamente captura I2S e inferencia. A funcao so retorna ESP_OK
 * quando nenhuma das duas tarefas esta lendo ou processando audio.
 *
 * As pausas sao contadas: cada audio_ml_pause() com sucesso precisa de um
 * audio_ml_resume(). O audio so volta quando todas as pausas forem liberadas
 * (ex.: envio das medias e envio de alerta ao mesmo tempo).
 */
esp_err_t audio_ml_pause(uint32_t timeout_ms);

/** Retoma captura e inferencia depois de audio_ml_pause(). */
void audio_ml_resume(void);

/** Nivel relativo do microfone; nao e dB SPL calibrado. */
float audio_ml_get_db(void);

typedef struct {
    float probability;
    float history_average;
    bool risk_active;
    uint8_t positive_votes;
    uint8_t history_count;
    uint32_t inference_count;
} audio_ml_risk_status_t;

/** Retorna probabilidade, votos e estado temporal mais recentes. */
esp_err_t audio_ml_get_risk_status(audio_ml_risk_status_t *out_status);

/**
 * Callback chamado pela task de inferencia quando o estado de risco muda
 * (NORMAL -> RISCO ou RISCO -> NORMAL). Deve ser rapido e nao bloquear:
 * o uso esperado e apenas notificar outra task.
 */
typedef void (*audio_ml_risk_callback_t)(bool risk_active, void *ctx);

/** Registra o callback de mudanca de risco. Passe NULL para remover. */
void audio_ml_set_risk_callback(audio_ml_risk_callback_t callback, void *ctx);

#ifdef __cplusplus
}
#endif
