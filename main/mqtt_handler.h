#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <stdbool.h>
#include "mqtt_client.h"
#include "esp_err.h"

// Inicia a configuração do MQTT
void mqtt_app_start(void);

// Verifica se está conectado para evitar erros de publicação
bool mqtt_is_connected(void);

// Publica dados no tópico padrão
int mqtt_publish_sensor_data(const char *payload, int len);

#endif // MQTT_HANDLER_H