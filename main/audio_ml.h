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

#ifdef __cplusplus
}
#endif
