#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float probability;
    float history_average;
    bool risk_active;
    uint8_t positive_votes;
    uint8_t history_count;
    uint32_t inference_count;
} tinyml_risk_status_t;

/** Inicializa o DSP Log-Mel e o modelo V6.1 INT8. */
esp_err_t tinyml_runtime_init(void);

/**
 * Processa exatamente um segundo de audio PCM16 mono a 16 kHz.
 * A decisao final usa 3 de 5 janelas e histerese.
 */
esp_err_t tinyml_runtime_infer_pcm16(const int16_t *pcm, size_t sample_count);

/** Copia de forma atomica o ultimo resultado disponivel. */
esp_err_t tinyml_runtime_get_status(tinyml_risk_status_t *out_status);

#ifdef __cplusplus
}
#endif

