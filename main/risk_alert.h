#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Cria a task de alerta e registra o callback da IA.
 *
 * Quando a IA muda para RISCO, a task:
 *   1. fecha o portao dos sensores (a leitura em andamento termina e a
 *      proxima fica esperando);
 *   2. pausa o audio;
 *   3. publica o alerta em sensors/alert (QoS 1);
 *   4. libera audio e sensores, que voltam ao ciclo normal.
 * Quando o risco termina, um alerta de fim e publicado da mesma forma.
 *
 * Deve ser chamada antes de audio_ml_start().
 */
esp_err_t risk_alert_start(void);

/**
 * Portao dos sensores. A task de sensores chama enter/exit em volta de cada
 * leitura I2C; a task de alerta segura o portao durante o envio.
 * Sem risk_alert_start(), as funcoes nao fazem nada.
 */
void risk_alert_sensor_gate_enter(void);
void risk_alert_sensor_gate_exit(void);

#ifdef __cplusplus
}
#endif
