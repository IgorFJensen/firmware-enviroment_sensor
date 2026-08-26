#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicializa o ICS-43434 e cria a task que alimenta o WakeNet continuamente.
 * O mesmo fluxo de audio tambem e usado para atualizar o nivel do microfone,
 * evitando duas tasks concorrendo pelo mesmo canal I2S.
 */
esp_err_t audio_wakenet_start(void);

/**
 * Retorna a ultima medida de nivel calculada com a mesma escala aproximada
 * usada anteriormente por read_microphone_db().
 */
float audio_wakenet_get_db(void);

#ifdef __cplusplus
}
#endif
