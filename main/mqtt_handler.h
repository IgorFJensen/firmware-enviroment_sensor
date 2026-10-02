#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void mqtt_app_start(void);
bool mqtt_is_connected(void);

/**
 * Publica em sensors/data com QoS 1 e retain, aguardando a confirmacao do
 * broker. Enquanto esta funcao estiver bloqueada, a task chamadora nao inicia
 * novas leituras de sensores.
 */
esp_err_t mqtt_publish_sensor_data(
    const char *payload,
    size_t len,
    uint32_t timeout_ms,
    int *out_msg_id);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_HANDLER_H */
